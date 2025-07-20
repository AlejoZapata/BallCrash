/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/sem.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "ws2812.pio.h"

#define FRAC_BITS 4
#define NUM_PIXELS 64
#define WS2812_PIN_BASE 2

// Check the pin is compatible with the platform
#if WS2812_PIN_BASE >= NUM_BANK0_GPIOS
#error Attempting to use a pin>=32 on a platform that does not support it
#endif

// horrible temporary hack to avoid changing pattern code
static uint8_t *current_strip_out;
static bool current_strip_4color;

/**
 * \brief Envía un píxel al buffer de salida.
 * \details Inserta los bytes GRB de \p pixel_grb en el buffer apuntado
 *          por \c current_strip_out y avanza el puntero. Si \c current_strip_4color
 *          está habilitado, añade un byte extra para el canal blanco.
 *
 * \param pixel_grb Valor empaquetado GRB de 24 bits.
 */
static inline void put_pixel(uint32_t pixel_grb) {
    *current_strip_out++ = pixel_grb & 0xFFu;          /**< Azul (low byte) */
    *current_strip_out++ = (pixel_grb >> 8u) & 0xFFu;  /**< Rojo */
    *current_strip_out++ = (pixel_grb >> 16u) & 0xFFu; /**< Verde */
    if (current_strip_4color) {
        *current_strip_out++ = 0;                      /**< Blanco (sin ajustar) */
    }
}

/**
 * \brief Empaqueta componentes R, G, B en un valor de 32 bits GRB.
 * \param r Componente roja (0–255).
 * \param g Componente verde (0–255).
 * \param b Componente azul (0–255).
 * \return Valor empaquetado GRB.
 */
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 8) |
           ((uint32_t)g << 16) |
           (uint32_t)b;
}

/**
 * \brief Patrón “Snakes!”: franjas móviles de color rojo, verde, azul.
 * \details Desplaza bloques de 10 LEDs en rojo, 10 en verde, 10 en azul,
 *          dejando el resto apagado.
 * \param len Número de LEDs en la tira.
 * \param t   Contador de frames (se desplaza medio LED cada 2 ticks).
 */
void pattern_snakes(uint len, uint t) {
    for (uint i = 0; i < len; ++i) {
        uint x = (i + (t >> 1)) % NUM_PIXELS;
        if (x < 10)
            put_pixel(urgb_u32(0xFF, 0x00, 0x00));  /**< Rojo */
        else if (x < 25 && x >= 15)
            put_pixel(urgb_u32(0x00, 0xFF, 0x00));  /**< Verde */
        else if (x < 40 && x >= 30)
            put_pixel(urgb_u32(0x00, 0x00, 0xFF));  /**< Azul */
        else
            put_pixel(0);                            /**< Apagado */
    }
}

/**
 * \brief Patrón “Random data”: muestra bytes aleatorios.
 * \details Solo actualiza cada 8 ticks de \p t; en otros momentos no hace nada.
 * \param len Número de LEDs en la tira.
 * \param t   Contador de frames.
 */
void pattern_random(uint len, uint t) {
    if (t % 8) return;
    for (uint i = 0; i < len; ++i) {
        put_pixel(rand());
    }
}

/**
 * \brief Patrón “Sparkles”: puntos blancos aleatorios sobre fondo apagado.
 * \details Solo actualiza cada 8 ticks de \p t; en otros momentos no hace nada.
 * \param len Número de LEDs en la tira.
 * \param t   Contador de frames.
 */
void pattern_sparkle(uint len, uint t) {
    if (t % 8) return;
    for (uint i = 0; i < len; ++i) {
        put_pixel((rand() % 16) ? 0 : 0xFFFFFFu);
    }
}

/**
 * \brief Patrón “Greys”: barra de gris que recorre la tira.
 * \details Utiliza un máximo de 100 niveles para reducir consumo,
 *          incrementando el gris por LED en cada tick.
 * \param len Número de LEDs en la tira.
 * \param t   Contador de frames (valor de gris).
 */
void pattern_greys(uint len, uint t) {
    const uint max = 100;
    t %= max;
    for (uint i = 0; i < len; ++i) {
        put_pixel(t * 0x010101u);
        if (++t >= max) t = 0;
    }
}

/**
 * \brief Patrón “Solid”: toda la tira de un gris fijo.
 * \param len Número de LEDs en la tira.
 * \param t   Contador de frames (no usado).
 */
void pattern_solid(uint len, uint t) {
    (void)t;
    for (uint i = 0; i < len; ++i) {
        put_pixel(0x010101u);
    }
}

/**
 * \brief Patrón “Fade”: fundido de gris controlado por \c level.
 * \details Ajusta la intensidad global con corrección de error
 *          para suavizar el paso entre niveles.
 * \param len Número de LEDs en la tira.
 * \param t   Contador de frames (usado para velocidad de fundido).
 */
void pattern_fade(uint len, uint t) {
    const uint shift = FRAC_BITS;
    uint max = 16 << shift;
    uint slow_t = level % max;
    static int error = 0;
    slow_t = slow_t * max + error;
    error  = slow_t & ((1u << shift) - 1);
    slow_t >>= shift;
    slow_t *= 0x010101u;

    for (uint i = 0; i < len; ++i) {
        put_pixel(slow_t);
    }
}

/**
 * \def VALUE_PLANE_COUNT
 * \brief Número de planos de bits para cada valor (8 bits + bits fraccionales).
 */
#define VALUE_PLANE_COUNT (8 + FRAC_BITS)

/**
 * \struct value_bits_t
 * \brief Almacena los planos de bits de múltiples tiras de LEDs.
 *
 * Cada plano en \c planes[] contiene el N‑ésimo bit de todos los valores
 * (R/G/B/W) de cada tira, almacenado MSB primero.
 */
typedef struct {
    uint32_t planes[VALUE_PLANE_COUNT]; /**< Planos de bits MSB primero. */
} value_bits_t;

/**
 * \brief Suma los planos fraccionales de error a los planos base.
 * \details Añade con acarreo los \c FRAC_BITS planos fraccionales desde \c e y \c s,
 *          y luego propaga el acarreo a través de los 8 planos de bits enteros.
 *
 * \param d Puntero al destino donde se escriben los planos resultantes.
 * \param s Puntero a los planos originales (source).
 * \param e Puntero a los planos de error a agregar.
 */
void add_error(value_bits_t *d, const value_bits_t *s, const value_bits_t *e) {
    uint32_t carry_plane = 0;
    /* Acarreo en planos fraccionales */
    for (int p = VALUE_PLANE_COUNT - 1; p >= 8; p--) {
        uint32_t e_plane = e->planes[p];
        uint32_t s_plane = s->planes[p];
        d->planes[p]   = (e_plane ^ s_plane) ^ carry_plane;
        carry_plane    = (e_plane & s_plane) | (carry_plane & (s_plane ^ e_plane));
    }
    /* Propagación de acarreo en planos enteros */
    for (int p = 7; p >= 0; p--) {
        uint32_t s_plane = s->planes[p];
        d->planes[p]    = s_plane ^ carry_plane;
        carry_plane    &= s_plane;
    }
}

/**
 * \struct strip_t
 * \brief Describe una tira de LEDs y su factor de brillo.
 *
 * \var strip_t::data
 *   Puntero a los valores de color (bytes) de la tira.
 * \var strip_t::data_len
 *   Longitud de \c data en bytes.
 * \var strip_t::frac_brightness
 *   Brillo fraccional (256 = 1.0).
 */
typedef struct {
    uint8_t *data;
    uint    data_len;
    uint    frac_brightness;
} strip_t;

/**
 * \brief Transforma valores de color a planos de bits para DMA.
 * \details Para cada posición:
 *   - Ajusta el valor por el brillo de cada tira y el brillo global.
 *   - Descompone el valor resultante en bits y marca los correspondientes
 *     en los planos de \c values[v].
 *
 * \param strips          Array de punteros a \c strip_t.
 * \param num_strips      Número de tiras en \c strips.
 * \param values          Array de \c value_bits_t donde se guardan los planos.
 * \param value_length    Número de valores (ej. NUM_PIXELS*4).
 * \param frac_brightness Brillo global (256 = 1.0).
 */
void transform_strips(strip_t **strips, uint num_strips,
                      value_bits_t *values, uint value_length,
                      uint frac_brightness) {
    for (uint v = 0; v < value_length; v++) {
        memset(&values[v], 0, sizeof(values[v]));
        for (uint i = 0; i < num_strips; i++) {
            if (v < strips[i]->data_len) {
                /* Ajuste de brillo individual y global */
                uint32_t val = (strips[i]->data[v] * strips[i]->frac_brightness) >> 8u;
                val = (val * frac_brightness) >> 8u;
                /* Descomponer en bits y asignar a planos */
                for (int j = 0; j < VALUE_PLANE_COUNT && val; j++, val >>= 1u) {
                    if (val & 1u) {
                        values[v].planes[VALUE_PLANE_COUNT - 1 - j] |= 1u << i;
                    }
                }
            }
        }
    }
}

/**
 * \brief Aplica dithering a una serie de valores propagando errores.
 * \details Llama a \c add_error para cada índice, acumulando el error
 *          previo en \c old_state y escribiendo en \c state.
 *
 * \param colors       Valores deseados antes del dithering.
 * \param state        Estados actuales donde se acumulan errores.
 * \param old_state    Estados previos usados para error de propagación.
 * \param value_length Número de elementos en los arrays.
 */
void dither_values(const value_bits_t *colors,
                   value_bits_t *state,
                   const value_bits_t *old_state,
                   uint value_length) {
    for (uint i = 0; i < value_length; i++) {
        add_error(state + i, colors + i, old_state + i);
    }
}

/* Buffers de color y estado para doble buffering */
static value_bits_t colors[NUM_PIXELS * 4];     /**< Colores solicitados. */
static value_bits_t states[2][NUM_PIXELS * 4];  /**< Estados para doble buffering. */

/* Datos de ejemplo para dos tiras */
static uint8_t strip0_data[NUM_PIXELS * 3];     /**< Tira 0: RGB. */
static uint8_t strip1_data[NUM_PIXELS * 4];     /**< Tira 1: RGBW. */

/* Configuración de las tiras */
strip_t strip0 = {
    .data           = strip0_data,
    .data_len       = sizeof(strip0_data),
    .frac_brightness= 0x40
};

strip_t strip1 = {
    .data           = strip1_data,
    .data_len       = sizeof(strip1_data),
    .frac_brightness= 0x100
};

strip_t *strips[] = {
    &strip0,
    &strip1
};

//**
* \def DMA_CHANNEL
* \brief Canal DMA principal para envío de planos de bits.
*/
#define DMA_CHANNEL 0

/**
* \def DMA_CB_CHANNEL
* \brief Canal DMA en cadena para reconfigurar el canal principal.
*/
#define DMA_CB_CHANNEL 1

/** \def DMA_CHANNEL_MASK
* \brief Máscara de bit para \c DMA_CHANNEL. */
#define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)
/** \def DMA_CB_CHANNEL_MASK
* \brief Máscara de bit para \c DMA_CB_CHANNEL. */
#define DMA_CB_CHANNEL_MASK (1u << DMA_CB_CHANNEL)
/** \def DMA_CHANNELS_MASK
* \brief Máscara combinada de ambos canales DMA. */
#define DMA_CHANNELS_MASK (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

/**
* \brief Tabla de punteros a los inicios de cada fragmento DMA.
* \details El elemento [value_length] se deja a cero como terminador nulo.
*/
static uintptr_t fragment_start[NUM_PIXELS * 4 + 1];

/**
* \var reset_delay_complete_sem
* \brief Semáforo que indica cuándo es seguro iniciar un nuevo burst DMA.
*/
static struct semaphore reset_delay_complete_sem;

/**
* \var reset_delay_alarm_id
* \brief Identificador de alarma para gestionar el retardo de reset.
*/
alarm_id_t reset_delay_alarm_id;

/**
* \brief Callback de alarma al completar el retardo de reset.
* \param id       Identificador de la alarma (no usado).
* \param user_data Puntero de usuario (no usado).
* \return 0 para no reprogramar la alarma.
*/
int64_t reset_delay_complete(__unused alarm_id_t id, __unused void *user_data) {
   reset_delay_alarm_id = 0;
   sem_release(&reset_delay_complete_sem);
   return 0;
}

/**
* \brief Handler de interrupción DMA cuando termina la transferencia.
* \details Limpia la bandera de interrupción y programa la alarma de reset
*          de tira WS2812 tras 400 µs.
*/
void __isr dma_complete_handler() {
   if (dma_hw->ints0 & DMA_CHANNEL_MASK) {
       dma_hw->ints0 = DMA_CHANNEL_MASK;  // Limpiar IRQ
       if (reset_delay_alarm_id) {
           cancel_alarm(reset_delay_alarm_id);
       }
       reset_delay_alarm_id = add_alarm_in_us(400, reset_delay_complete, NULL, true);
   }
}

/**
* \brief Inicializa y configura los canales DMA para salida paralela WS2812.
* \param pio Instancia PIO donde corre el programa WS2812.
* \param sm  State machine de PIO utilizada.
*/
void dma_init(PIO pio, uint sm) {
   dma_claim_mask(DMA_CHANNELS_MASK);

   // Configurar canal principal: envía fragmentos de 8 palabras
   dma_channel_config channel_config = dma_channel_get_default_config(DMA_CHANNEL);
   channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true));
   channel_config_set_chain_to(&channel_config, DMA_CB_CHANNEL);
   channel_config_set_irq_quiet(&channel_config, true);
   dma_channel_configure(DMA_CHANNEL,
                         &channel_config,
                         &pio->txf[sm],  // destino: FIFO PIO
                         NULL,           // fuente configurada por cadena
                         8,              // 8 palabras por fragmento
                         false);

   // Configurar canal en cadena: apunta al siguiente fragmento
   dma_channel_config chain_config = dma_channel_get_default_config(DMA_CB_CHANNEL);
   dma_channel_configure(DMA_CB_CHANNEL,
                         &chain_config,
                         &dma_channel_hw_addr(DMA_CHANNEL)->al3_read_addr_trig,
                         NULL,
                         1,
                         false);

   irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
   dma_channel_set_irq0_enabled(DMA_CHANNEL, true);
   irq_set_enabled(DMA_IRQ_0, true);
}

/**
* \brief Carga los punteros a los planos de bits en la lista de fragmentos DMA.
* \param bits         Array de \c value_bits_t con los planos de bits MSB primero.
* \param value_length Número de valores (ej. NUM_PIXELS*4).
*/
void output_strips_dma(value_bits_t *bits, uint value_length) {
   for (uint i = 0; i < value_length; i++) {
       fragment_start[i] = (uintptr_t)bits[i].planes;
   }
   fragment_start[value_length] = 0;  // Terminador nulo
   dma_channel_hw_addr(DMA_CB_CHANNEL)->al3_read_addr_trig = (uintptr_t)fragment_start;
}


int main() {
    //set_sys_clock_48();
    stdio_init_all();
    printf("WS2812 parallel using pin %d\n", WS2812_PIN_BASE);

    PIO pio;
    uint sm;
    uint offset;

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_parallel_program, &pio, &sm, &offset, WS2812_PIN_BASE, count_of(strips), true);
    hard_assert(success);

    ws2812_parallel_program_init(pio, sm, offset, WS2812_PIN_BASE, count_of(strips), 800000);

    sem_init(&reset_delay_complete_sem, 1, 1); // initially posted so we don't block first time
    dma_init(pio, sm);
    int t = 0;
    while (1) {
        int pat = rand() % count_of(pattern_table);
        int dir = (rand() >> 30) & 1 ? 1 : -1;
        if (rand() & 1) dir = 0;
        puts(pattern_table[pat].name);
        puts(dir == 1 ? "(forward)" : dir ? "(backward)" : "(still)");
        int brightness = 0;
        uint current = 0;
        for (int i = 0; i < 1000; ++i) {
            current_strip_out = strip0.data;
            current_strip_4color = false;
            pattern_table[pat].pat(NUM_PIXELS, t);
            current_strip_out = strip1.data;
            current_strip_4color = true;
            pattern_table[pat].pat(NUM_PIXELS, t);

            transform_strips(strips, count_of(strips), colors, NUM_PIXELS * 4, brightness);
            dither_values(colors, states[current], states[current ^ 1], NUM_PIXELS * 4);
            sem_acquire_blocking(&reset_delay_complete_sem);
            output_strips_dma(states[current], NUM_PIXELS * 4);

            current ^= 1;
            t += dir;
            brightness++;
            if (brightness == (0x20 << FRAC_BITS)) brightness = 0;
        }
        memset(&states, 0, sizeof(states)); // clear out errors
    }

    // This will free resources and unload our program
    pio_remove_program_and_unclaim_sm(&ws2812_parallel_program, pio, sm, offset);
}
