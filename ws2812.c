/**
 * \file ws2812_futbolito.c
 * \brief Animaciones WS2812 para un “futbolito” con botones de gol.
 *
 * Este programa corre en Raspberry Pi Pico y muestra:
 *   - Un patrón continuo tipo “futbolito” con colores suaves girando.
 *   - Un patrón de celebración de gol al pulsar el botón rojo o azul.
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #include "pico/stdlib.h"
 #include "hardware/pio.h"
 #include "hardware/clocks.h"
 #include "hardware/gpio.h"
 #include "ws2812.pio.h"
 
 #define IS_RGBW        false      /**< \brief Formato RGBW deshabilitado. */
 #define NUM_PIXELS     60         /**< \brief Número total de LEDs en la tira. */
 #define GOL_BOTON_ROJO 14         /**< \brief GPIO del botón de gol ROJO. */
 #define GOL_BOTON_AZUL 13         /**< \brief GPIO del botón de gol AZUL. */
 
 #ifdef PICO_DEFAULT_WS2812_PIN
   #define WS2812_PIN PICO_DEFAULT_WS2812_PIN
 #else
   #define WS2812_PIN 2             /**< \brief GPIO por defecto para WS2812. */
 #endif
 
 #if WS2812_PIN >= NUM_BANK0_GPIOS
   #error "Pin WS2812 no soportado (>=32)"
 #endif
 
 /**
  * \brief Envía un valor empaquetado GRB al FIFO del state machine.
  * \param pio         Instancia PIO donde corre el programa WS2812.
  * \param sm          State machine de PIO utilizada.
  * \param pixel_grb   Valor GRB empaquetado (24 bits) del color.
  */
 static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
     pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
 }
 
 /**
  * \brief Empaqueta componentes R, G, B en un valor de 32 bits GRB.
  * \param r  Componente roja (0–255).
  * \param g  Componente verde (0–255).
  * \param b  Componente azul (0–255).
  * \return Valor empaquetado GRB.
  */
 static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
     return ((uint32_t)r << 8)  |
            ((uint32_t)g << 16) |
            (uint32_t)b;
 }
 
 /**
  * \brief Empaqueta componentes R, G, B, W en un valor de 32 bits GRBW.
  * \param r  Componente roja (0–255).
  * \param g  Componente verde (0–255).
  * \param b  Componente azul (0–255).
  * \param w  Componente blanca (0–255).
  * \return Valor empaquetado GRBW.
  */
 static inline uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
     return ((uint32_t)r << 8)  |
            ((uint32_t)g << 16) |
            ((uint32_t)w << 24) |
            (uint32_t)b;
 }
 
 /**
  * \brief Patrón continuo tipo “futbolito” con colores suaves girando.
  * \param pio  Instancia PIO donde corre el programa WS2812.
  * \param sm   State machine de PIO utilizada.
  * \param len  Número total de LEDs (NUM_PIXELS).
  * \param t    Contador de frames que desplaza el patrón.
  */
 void pattern_futbolito(PIO pio, uint sm, uint len, uint t) {
     for (uint i = 0; i < len; ++i) {
         float pos = (i + t) / 5.0f; 
         float r   = (sinf(pos)       + 1.0f) * 127.5f;
         float g   = (sinf(pos + 2.0f) + 1.0f) * 127.5f;
         float b   = (sinf(pos + 4.0f) + 1.0f) * 127.5f;
         put_pixel(pio, sm, urgb_u32((uint8_t)r, (uint8_t)g, (uint8_t)b));
     }
 }
 
 /**
  * \brief Patrón de celebración de gol (pico de color + decaimiento).
  * \param pio  Instancia PIO donde corre el programa WS2812.
  * \param sm   State machine de PIO utilizada.
  * \param len  Número total de LEDs (NUM_PIXELS).
  * \param t    Contador de frames que controla la cabeza del pico.
  * \param r    Componente roja (0–255) del color de gol.
  * \param g    Componente verde (0–255) del color de gol.
  * \param b    Componente azul (0–255) del color de gol.
  */
 void pattern_gol(PIO pio, uint sm, uint len, uint t,
                  uint8_t r, uint8_t g, uint8_t b) {
     int head = t % len;
     for (uint i = 0; i < len; ++i) {
         int dist = abs((int)i - head);
         if (dist == 0)       put_pixel(pio, sm, urgb_u32(r, g, b));
         else if (dist == 1)  put_pixel(pio, sm, urgb_u32(r/2, g/2, b/2));
         else if (dist == 2)  put_pixel(pio, sm, urgb_u32(r/4, g/4, b/4));
         else                 put_pixel(pio, sm, 0);
     }
 }
 
 /**
  * \brief Punto de entrada del programa.
  * \details Inicializa STDIO, botones de gol, PIO/WS2812,
  *          y luego ejecuta un bucle infinito:
  *            - Al pulsar el botón rojo: 500 frames de pattern_gol(...,255,0,0)
  *            - Al pulsar el botón azul:  500 frames de pattern_gol(...,0,0,255)
  *            - En otro caso: pattern_futbolito + retardo de 50 ms.
  *
  * \return Nunca retorna, ya que el bucle es infinito.
  */
 int main(void) {
     stdio_init_all();
     printf("WS2812 Futbolito LED\n");
 
     /* Configurar botones de gol */
     gpio_init(GOL_BOTON_ROJO);
     gpio_set_dir (GOL_BOTON_ROJO, GPIO_IN);
     gpio_pull_up(GOL_BOTON_ROJO);
 
     gpio_init(GOL_BOTON_AZUL);
     gpio_set_dir (GOL_BOTON_AZUL, GPIO_IN);
     gpio_pull_up(GOL_BOTON_AZUL);
 
     /* Iniciar PIO y WS2812 */
     PIO pio;
     uint sm, offset;
     bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
                        &ws2812_program, &pio, &sm, &offset,
                        WS2812_PIN, 1, true);
     hard_assert(success);
     ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
 
     int t = 0;
     while (1) {
         if (!gpio_get(GOL_BOTON_ROJO)) {
             for (int i = 0; i < 500; ++i) {
                 pattern_gol(pio, sm, NUM_PIXELS, t, 255, 0, 0);
                 sleep_ms(10);
                 ++t;
             }
         }
         else if (!gpio_get(GOL_BOTON_AZUL)) {
             for (int i = 0; i < 500; ++i) {
                 pattern_gol(pio, sm, NUM_PIXELS, t, 0, 0, 255);
                 sleep_ms(10);
                 ++t;
             }
         }
         else {
             pattern_futbolito(pio, sm, NUM_PIXELS, t);
             sleep_ms(50);
             ++t;
         }
     }
 
     /* Nunca llega aquí, pero se incluye por limpieza */
     pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
     return 0;
 }
 