/**
 * \file pantallaDigitales.c
 * \brief Control de juego ball crash con Raspberry Pi Pico y tira de LEDs WS2812.
 *
 * Este archivo implementa la lógica de juego de crash ball para dos jugadores,
 * incluyendo manejo de joystick, sensores de gol, poderes especiales,
 * animaciones LED WS2812, y pantalla LCD I2C dual.
 */


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "ws2812.pio.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include <string.h>

/** \def CENTER_P1
 *  Centro calibrado para joystick del jugador 1. */
#define CENTER_P1 1980  

/** \def CENTER_P2
 *  Centro calibrado para joystick del jugador 2. */

#define CENTER_P2 1730 

/** \def DEADZONE
 *  Zona muerta (tolerancia) en lecturas ADC para joystick. */

#define DEADZONE  500
/** \def PWM_VAL
 *  Valor de PWM aplicado a motores. */
/** \def PWM_VAL
 *  Valor de PWM aplicado a los motores (resolución de 8 bits). */
#define PWM_VAL   200

/** \def LED_P1_A
 *  GPIO del LED A del jugador 1. */
#define LED_P1_A 0
/** \def LED_P1_B
 *  GPIO del LED B del jugador 1. */
#define LED_P1_B 1
/** \def LED_P2_A
 *  GPIO del LED A del jugador 2. */
#define LED_P2_A 16
/** \def LED_P2_B
 *  GPIO del LED B del jugador 2. */
#define LED_P2_B 17

/** \def I2C_LCD1
 *  Instancia I2C para el primer display LCD. */
#define I2C_LCD1 i2c0
/** \def I2C_LCD2
 *  Instancia I2C para el segundo display LCD. */
#define I2C_LCD2 i2c1

/** \def SDA_LCD1
 *  Pin SDA para el primer display LCD. */
#define SDA_LCD1 4
/** \def SCL_LCD1
 *  Pin SCL para el primer display LCD. */
#define SCL_LCD1 5
/** \def SDA_LCD2
 *  Pin SDA para el segundo display LCD. */
#define SDA_LCD2 6
/** \def SCL_LCD2
 *  Pin SCL para el segundo display LCD. */
#define SCL_LCD2 7

/** \def LCD_ADDR
 *  Dirección I2C del módulo LCD. */
#define LCD_ADDR 0x27
/** \def LCD_BACKLIGHT
 *  Máscara para habilitar la luz de fondo del LCD. */
#define LCD_BACKLIGHT 0x08
/** \def ENABLE
 *  Máscara para la señal ENABLE del LCD. */
#define ENABLE 0b00000100

/** \def WIN_SCORE
 *  Puntuación necesaria para ganar el juego. */
#define WIN_SCORE 7

// === Jugador 1 ===
/** \def SENSOR_P1_PIN
 *  Pin del sensor de gol del jugador 1. */
#define SENSOR_P1_PIN 10
/** \def PULSADOR_P1_PIN
 *  Pin del pulsador (botón) del jugador 1. */
#define PULSADOR_P1_PIN 13
/** \def TRANSISTOR_P1_PIN
 *  Pin de control del transistor/solenoide del jugador 1. */
#define TRANSISTOR_P1_PIN 14
/** \def VRX_P1_PIN
 *  Canal ADC (VRX) para el joystick del jugador 1. */
#define VRX_P1_PIN 27       // ADC1
/** \def PWM_P1_PIN
 *  Pin de salida PWM para el motor del jugador 1. */
#define PWM_P1_PIN 22
/** \def IN1_P1
 *  Pin IN1 del puente H para el jugador 1. */
#define IN1_P1 2
/** \def IN2_P1
 *  Pin IN2 del puente H para el jugador 1. */
#define IN2_P1 3
/** \def SW_P1
 *  Pin del interruptor del joystick del jugador 1. */
#define SW_P1 11

// === Jugador 2 ===
/** \def SENSOR_P2_PIN
 *  Pin del sensor de gol del jugador 2. */
#define SENSOR_P2_PIN 21
/** \def PULSADOR_P2_PIN
 *  Pin del pulsador (botón) del jugador 2. */
#define PULSADOR_P2_PIN 20
/** \def TRANSISTOR_P2_PIN
 *  Pin de control del transistor/solenoide del jugador 2. */
#define TRANSISTOR_P2_PIN 19
/** \def VRX_P2_PIN
 *  Canal ADC (VRX) para el joystick del jugador 2. */
#define VRX_P2_PIN 28       // ADC2
/** \def PWM_P2_PIN
 *  Pin de salida PWM para el motor del jugador 2. */
#define PWM_P2_PIN 18
/** \def IN1_P2
 *  Pin IN1 del puente H para el jugador 2. */
#define IN1_P2 8
/** \def IN2_P2
 *  Pin IN2 del puente H para el jugador 2. */
#define IN2_P2 9
/** \def SW_P2
 *  Pin del interruptor del joystick del jugador 2. */
#define SW_P2 12





// === Variables ===

volatile bool animacion_gol_activa = false;

/**var animacion_gol_inicio
 *  Instante en milisegundos de inicio de la animación de gol. */
volatile uint32_t animacion_gol_inicio = 0;

/**var color_gol_r
 *  Componente roja del color de animación de gol. */
volatile uint8_t color_gol_r = 0;
/**var color_gol_g
 *  Componente verde del color de animación de gol. */
volatile uint8_t color_gol_g = 0;
/**var color_gol_b
 *  Componente azul del color de animación de gol. */
volatile uint8_t color_gol_b = 0;

/**var gol_de_p1
 *  True si el último gol fue marcado por el jugador 1. */
volatile bool gol_de_p1 = true;

/** var duracion_animacion_gol
 *  Duración de la animación de gol en milisegundos. */
const uint32_t duracion_animacion_gol = 5000;

/**
 * \brief Patrón animado inicial de futbolito en LEDs.
 * \param pio Instancia PIO donde corre el programa WS2812.
 * \param sm  State machine de PIO.
 * \param len Número total de LEDs.
 * \param t   Contador de frames.
 */
void pattern_futbolito(PIO pio, uint sm, uint len, uint t);

/**
 * \brief Patrón de animación de gol con color fijo.
 * \param pio Instancia PIO.
 * \param sm  State machine de PIO.
 * \param len Número de LEDs.
 * \param t   Contador de frames.
 * \param R   Componente roja.
 * \param G   Componente verde.
 * \param B   Componente azul.
 */
void pattern_gol_color(PIO pio, uint sm, uint len, uint t, uint8_t R, uint8_t G, uint8_t B);


// Contadores de puntos y estado del juego
int score_p1 = 0;
int score_p2 = 0;
int frame = 0;
bool last_sensor1 = 1;
bool last_sensor2 = 1;

// Flags de uso de poderes
bool power_invert_used_p1 = false;
bool power_freeze_used_p1 = false;
bool power_invert_used_p2 = false;
bool power_freeze_used_p2 = false;

// Tiempos de activación
uint32_t invert_start_p1 = 0;
uint32_t freeze_start_p1 = 0;
uint32_t invert_start_p2 = 0;
uint32_t freeze_start_p2 = 0;

// LCD - Mensajes temporales
char lcd_msg[17] = "";           // mensaje actual en segunda línea
uint32_t lcd_msg_timeout = 0;    // momento en que debe ocultarse

// Detección de doble toque (por jugador)
uint32_t last_joy_press_p1 = 0;
bool awaiting_double_p1 = false;
bool prev_joy_btn_p1 = true;

uint32_t last_joy_press_p2 = 0;
bool awaiting_double_p2 = false;
bool prev_joy_btn_p2 = true;

// WS2812 config
#define IS_RGBW false
#define NUM_PIXELS 59
uint32_t ws_pixels[NUM_PIXELS];
#define WS2812_PIN 26
PIO pio;
uint sm_ws;
uint offset_ws;

/**
 * \brief Envía un píxel al FIFO del PIO.
 * \param pio Instancia PIO.
 * \param sm  State machine.
 * \param pixel Valor RGB empaquetado.
 */
static inline void put_pixel(PIO pio, uint sm, uint32_t pixel) {
    while (pio_sm_is_tx_fifo_full(pio, sm));
    pio_sm_put(pio, sm, pixel << 8u);
}

/**
 * \brief Empaqueta componentes RGB en un valor de 32 bits.
 * \param r Rojo (0-255).
 * \param g Verde (0-255).
 * \param b Azul (0-255).
 * \return Valor empaquetado.
 */
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
}

/**
 * \brief Establece el color de un píxel en el buffer de LEDs.
 * \param index Índice del LED (0 a NUM_PIXELS-1).
 * \param r     Componente roja (0–255).
 * \param g     Componente verde (0–255).
 * \param b     Componente azul (0–255).
 */
static inline void set_pixel_color(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= 0 && index < NUM_PIXELS)
        ws_pixels[index] = urgb_u32(r, g, b);
}

/**
 * \brief Envía todo el buffer de píxeles al strip WS2812.
 * \param pio Instancia PIO donde corre el programa WS2812.
 * \param sm  State machine de PIO utilizada.
 */
void ws2812_show(PIO pio, uint sm) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        while (pio_sm_is_tx_fifo_full(pio, sm));  // Espera si el FIFO está lleno
        pio_sm_put(pio, sm, ws_pixels[i] << 8u);
    }
}

/**
 * \brief Patrón de animación de gol con dos colores alternos.
 * \details Desplaza un “pico” de color que alterna RGB entre los dos jugadores,
 *          disminuyendo gradualmente la intensidad.
 * \param pio   Instancia PIO donde corre el programa WS2812.
 * \param sm    State machine de PIO utilizada.
 * \param len   Número total de LEDs en la tira.
 * \param t     Contador de frames para controlar la posición del pico.
 * \param R1,G1,B1  Color (RGB) asignado al jugador 1.
 * \param R2,G2,B2  Color (RGB) asignado al jugador 2.
 */
void pattern_gol_dual_color(PIO pio, uint sm, uint len, uint t,
                            uint8_t R1, uint8_t G1, uint8_t B1,
                            uint8_t R2, uint8_t G2, uint8_t B2) {
    int head = t % len;
    for (uint i = 0; i < len; ++i) {
        int d = abs((int)i - head);
        uint8_t r = 0, g = 0, b = 0;

        if (d == 0) {
            if (i % 2 == 0) { r = R1; g = G1; b = B1; }
            else            { r = R2; g = G2; b = B2; }
        } else if (d == 1) {
            if (i % 2 == 0) { r = R1/2; g = G1/2; b = B1/2; }
            else            { r = R2/2; g = G2/2; b = B2/2; }
        } else if (d == 2) {
            if (i % 2 == 0) { r = R1/4; g = G1/4; b = B1/4; }
            else            { r = R2/4; g = G2/4; b = B2/4; }
        }

        set_pixel_color(i, r, g, b);
    }
    ws2812_show(pio, sm);
}

/**
 * \brief Patrón de “freeze” que modula azul y blanco.
 * \details Usa una función sinusoidal para variar la intensidad,
 *          oscilando el azul entre 200–255 y un toque de blanco.
 * \param pio Instancia PIO donde corre el programa WS2812.
 * \param sm  State machine de PIO utilizada.
 * \param len Número total de LEDs en la tira.
 * \param t   Contador de frames para animar la onda.
 */
void pattern_freeze(PIO pio, uint sm, uint len, uint t) {
    float intensity = (sin(t * 0.1f) + 1.0f) / 2.0f;  // 0–1 en onda
    uint8_t blue  = 200 + 55 * intensity;            // 200–255
    uint8_t white = 100 + 50 * intensity;            // 100–150

    for (int i = 0; i < len; i++) {
        set_pixel_color(i, white / 2, white / 2, blue);
    }
    ws2812_show(pio, sm);
}

/**
 * \brief Bucle principal que corre en el segundo core para animar la tira WS2812.
 * \details Comprueba el estado de “gol” o “freeze” y selecciona el patrón adecuado:
 *          - pattern_gol_dual_color mientras dure la animación de gol
 *          - pattern_freeze si algún jugador está “congelado”
 *          - pattern_futbolito en modo normal
 * \note Llama a sleep_ms(33) para aproximar ~30 FPS.
 */
void core1_ws2812_loop(void) {
    int t = 0;
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (animacion_gol_activa) {
            if (gol_de_p1) {
                pattern_gol_dual_color(pio, sm_ws, NUM_PIXELS, t,
                                       255, 0, 0,    // Rojo P1
                                       0, 0, 255);   // Azul P2
            } else {
                pattern_gol_dual_color(pio, sm_ws, NUM_PIXELS, t,
                                       0, 255, 0,    // Verde P2
                                       255, 255, 255);// Blanco P1
            }
            if (now - animacion_gol_inicio >= duracion_animacion_gol) {
                animacion_gol_activa = false;
            }
        }
        else if ((freeze_start_p1 > 0 && now - freeze_start_p1 < duracion_animacion_gol) ||
                 (freeze_start_p2 > 0 && now - freeze_start_p2 < duracion_animacion_gol)) {
            pattern_freeze(pio, sm_ws, NUM_PIXELS, t);
        }
        else {
            pattern_futbolito(pio, sm_ws, NUM_PIXELS, t);
        }

        t++;
        sleep_ms(33);
    }
}

/**
 * \brief Patrón de fondo “futbolito” que desplaza bloques de color.
 * \details Divide la tira en dos mitades:
 *          - LEDs [0..27]: verde/blanco
 *          - LEDs [28..59]: rojo/azul
 *          Mueve la transición con el frame counter \p t.
 * \param pio Instancia PIO donde corre el programa WS2812.
 * \param sm  State machine de PIO utilizada.
 * \param len Número total de LEDs (debe ser 60).
 * \param t   Contador de frames para desplazar el patrón.
 */
void pattern_futbolito(PIO pio, uint sm, uint len, uint t) {
    const uint lado_izq_len = 28;
    const uint lado_der_len = 32;
    const uint segmento_izq = lado_izq_len / 2;
    const uint segmento_der = lado_der_len / 2;

    for (uint i = 0; i < len; ++i) {
        uint8_t r, g, b;
        if (i < lado_izq_len) {
            uint local = (i + t) % lado_izq_len;
            if (local < segmento_izq) { r = 0;   g = 255; b = 0;   }
            else                      { r = 255; g = 255; b = 255; }
        } else {
            uint local = ((i - lado_izq_len) + t) % lado_der_len;
            if (local < segmento_der) { r = 255; g = 0;   b = 0;   }
            else                      { r = 0;   g = 0;   b = 255; }
        }
        set_pixel_color(i, r, g, b);
    }
    ws2812_show(pio, sm);
}

/**
 * \brief Patrón de animación de gol con un solo color fijo.
 * \details Muestra un “pico” de intensidad en color \p (R,G,B) que decae a
 *          la mitad y al cuarto en los LEDs adyacentes.
 * \param pio Instancia PIO donde corre el programa WS2812.
 * \param sm  State machine de PIO utilizada.
 * \param len Número total de LEDs en la tira.
 * \param t   Contador de frames para mover el pico.
 * \param R   Componente roja (0–255) del color del pico.
 * \param G   Componente verde (0–255) del color del pico.
 * \param B   Componente azul (0–255) del color del pico.
 */
void pattern_gol_color(PIO pio, uint sm, uint len, uint t,
                       uint8_t R, uint8_t G, uint8_t B) {
    int head = t % len;
    for (uint i = 0; i < len; ++i) {
        int d = abs((int)i - head);
        if      (d == 0) set_pixel_color(i, R,   G,   B  );
        else if (d == 1) set_pixel_color(i, R/2, G/2, B/2);
        else if (d == 2) set_pixel_color(i, R/4, G/4, B/4);
        else             set_pixel_color(i, 0,   0,   0  );
    }
    ws2812_show(pio, sm);
}

/**
 * \brief Marcos de trofeo para animaciones de puntuación.
 * \details Dos fotogramas que alternan una forma de trofeo de 8×8 bits.
 */
uint8_t trophy_frames[2][8] = {
    {0x04, 0x0E, 0x0E, 0x1F, 0x1B, 0x1F, 0x0E, 0x00},
    {0x04, 0x0E, 0x0A, 0x1F, 0x1F, 0x1F, 0x0E, 0x00}
};

/**
 * \brief Parpadea dos LEDs simultáneamente durante un tiempo determinado.
 * 
 * Alterna ambos GPIOs (\p pin_a y \p pin_b) encendidos y apagados
 * cada 125 ms hasta que transcurran \p duration_ms milisegundos.
 * 
 * \param pin_a       GPIO del primer LED.
 * \param pin_b       GPIO del segundo LED.
 * \param duration_ms Tiempo total de parpadeo en milisegundos.
 */
void flash_leds_dual(uint pin_a, uint pin_b, uint duration_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < duration_ms) {
        gpio_put(pin_a, 1);
        gpio_put(pin_b, 1);
        sleep_ms(125);
        gpio_put(pin_a, 0);
        gpio_put(pin_b, 0);
        sleep_ms(125);
    }
}

/**
 * \brief Envía un pulso ENABLE al LCD a través de I2C.
 * 
 * Latcha los datos o comandos en la pantalla al alternar el
 * bit ENABLE en \p val. Utiliza la instancia \p bus.
 * 
 * \param val Byte con datos/mandos (sin ENABLE).
 * \param bus Instancia I2C del LCD.
 */
void lcd_toggle_enable(uint8_t val, i2c_inst_t *bus) {
    sleep_us(500);
    i2c_write_blocking(bus, LCD_ADDR, &val, 1, true);
    i2c_write_blocking(bus, LCD_ADDR, &(uint8_t){val | ENABLE}, 1, true);
    sleep_us(500);
    i2c_write_blocking(bus, LCD_ADDR, &(uint8_t){val & ~ENABLE}, 1, true);
    sleep_us(500);
}

/**
 * \brief Envía un byte al LCD en modo de 4 bits por ambos busses.
 * 
 * Divide \p data en dos seminybbles (alto y bajo), agrega la máscara
 * de backlight y el indicador de dato/comando, y lo transmite por
 * I2C_LCD1 e I2C_LCD2.
 * 
 * \param data    Byte de comando o carácter.
 * \param is_data true si es carácter, false si es comando.
 */
void lcd_send_byte_dual(uint8_t data, bool is_data) {
    uint8_t mode = is_data ? 0x01 : 0x00;
    uint8_t high = (data & 0xF0) | LCD_BACKLIGHT | mode;
    uint8_t low  = ((data << 4) & 0xF0) | LCD_BACKLIGHT | mode;

    /* Primero al LCD1 */
    i2c_write_blocking(I2C_LCD1, LCD_ADDR, &high, 1, true);
    lcd_toggle_enable(high, I2C_LCD1);
    i2c_write_blocking(I2C_LCD1, LCD_ADDR, &low, 1, true);
    lcd_toggle_enable(low, I2C_LCD1);
    /* Luego al LCD2 */
    i2c_write_blocking(I2C_LCD2, LCD_ADDR, &high, 1, true);
    lcd_toggle_enable(high, I2C_LCD2);
    i2c_write_blocking(I2C_LCD2, LCD_ADDR, &low, 1, true);
    lcd_toggle_enable(low, I2C_LCD2);
}

/**
 * \brief Envía un comando al LCD.
 * \param cmd Código de comando (RS=0).
 */
void lcd_cmd(uint8_t cmd) {
    lcd_send_byte_dual(cmd, false);
}

/**
 * \brief Envía un carácter al LCD.
 * \param c Carácter ASCII (RS=1).
 */
void lcd_char(char c) {
    lcd_send_byte_dual(c, true);
}

/**
 * \brief Carga un carácter personalizado en la CGRAM del LCD.
 * \param location Posición (0–7) dentro de la CGRAM.
 * \param charmap  Arreglo de 8 bytes que define el patrón de píxeles.
 */
void lcd_custom_char(uint8_t location, uint8_t charmap[]) {
    location &= 0x7;
    lcd_cmd(0x40 | (location << 3));
    for (int i = 0; i < 8; i++) {
        lcd_char(charmap[i]);
    }
}

/**
 * \brief Inicializa el LCD en modo 4 bits.
 */
void lcd_init() {
    sleep_ms(50);
    lcd_cmd(0x33);
    lcd_cmd(0x32);
    lcd_cmd(0x28);  // 4 bits, 2 líneas, 5×8 dots
    lcd_cmd(0x0C);  // Display on, cursor off
    lcd_cmd(0x06);  // Entrada automática, desplazamiento a la derecha
    lcd_cmd(0x01);  // Limpia pantalla
    sleep_ms(5);
}

/**
 * \brief Posiciona el cursor en la columna \p col y fila \p row.
 * \param col Columna (0-index).
 * \param row Fila (0-index).
 */
void lcd_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t offsets[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + offsets[row]));
}

/**
 * \brief Imprime una cadena de texto en el LCD.
 * \param str Cadena null-terminated.
 */
void lcd_print(const char *str) {
    while (*str) {
        lcd_char(*str++);
    }
}

/**
 * \brief Limpia toda la pantalla del LCD.
 */
void lcd_clear() {
    lcd_cmd(0x01);
    sleep_ms(5);
}

/**
 * \brief Muestra el texto del ganador en ambas líneas.
 * \param winner Cadena con el nombre o mensaje del ganador.
 */
void show_winner(const char *winner) {
    lcd_clear();
    lcd_set_cursor(2, 0);
    lcd_print(winner);
    lcd_set_cursor(1, 1);
    lcd_print("GANADOR!");
}

/**
 * \brief Dibuja el marcador en la primera línea del LCD.
 */
void draw_frame() {
    lcd_set_cursor(0, 0);
    lcd_print("P1");
    lcd_set_cursor(3, 0);
    char buffer[4];
    sprintf(buffer, "%02d", score_p1);
    lcd_print(buffer);
    lcd_custom_char(0, trophy_frames[frame % 2]);
    lcd_set_cursor(7, 0);
    lcd_char(0);
    lcd_set_cursor(11, 0);
    sprintf(buffer, "%02d", score_p2);
    lcd_print(buffer);
    lcd_set_cursor(14, 0);
    lcd_print("P2");
}

/**
 * \brief Muestra un mensaje de estado en la segunda línea.
 * \param msg Cadena de hasta 16 caracteres.
 */
void lcd_show_status(const char *msg) {
    lcd_set_cursor(0, 1);
    lcd_print("                ");  // limpia línea
    lcd_set_cursor(0, 1);
    lcd_print(msg);
}

/**
 * \brief Parpadea un solo LED en \p gpio_pin.
 * \param gpio_pin Pin GPIO del LED.
 * \param duration_ms Duración del parpadeo en milisegundos.
 */
void flash_led(uint gpio_pin, uint duration_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < duration_ms) {
        gpio_put(gpio_pin, 1);
        sleep_ms(250);
        gpio_put(gpio_pin, 0);
        sleep_ms(250);
    }
}

/**
 * \brief Configura un pin como salida PWM para motor.
 * \param pin GPIO a configurar como PWM.
 */
void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

/**
 * \brief Controla un motor DC usando lecturas ADC del joystick.
 * 
 * Calcula el offset respecto a \p centro, determina dirección,
 * aplica PWM desde el canal de \p pwm_pin y controla \p in1/in2
 * para el puente H. Si \p inverted es true invierte el sentido.
 * 
 * \param adc_val   Lectura ADC (0–4095) del joystick.
 * \param inverted  true para invertir la dirección de giro.
 * \param in1       Pin IN1 del puente H.
 * \param in2       Pin IN2 del puente H.
 * \param pwm_pin   Pin PWM para velocidad.
 * \param centro    Valor central calibrado (ADC) para zona muerta.
 */
void motor_control_pwm(uint16_t adc_val, bool inverted,
                       uint in1, uint in2, uint pwm_pin, uint16_t centro) {
    int offset = adc_val - centro;
    int direction = 0;
    int pwm_value = 0;

    if (offset > DEADZONE) {
        direction = -1;
        pwm_value = PWM_VAL;
    } else if (offset < -DEADZONE) {
        direction = 1;
        pwm_value = PWM_VAL;
    }

    if (inverted) {
        direction *= -1;
    }

    gpio_put(in1, direction > 0);
    gpio_put(in2, direction < 0);

    uint slice = pwm_gpio_to_slice_num(pwm_pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pwm_pin), pwm_value);
}

/**
 * brief Punto de entrada de la aplicación de futbolito.
 *
 * Inicializa todos los periféricos:
 *   - Configura el PIO y lanza el programa WS2812 para la tira de LEDs.
 *   - Inicializa PWM para ambos motores.
 *   - Configura dos buses I2C y las pantallas LCD duales.
 *   - Prepara el ADC para la lectura de los joysticks.
 *   - Inicializa GPIOs de botones, sensores y transistores.
 *
 * Tras la configuración, muestra un mensaje de bienvenida y espera a que ambos
 * jugadores presionen sus botones para comenzar, y luego entra en el bucle
 * principal de juego donde gestiona:
 *   - Lectura continua de joysticks y sensores.
 *   - Control de motores por PWM.
 *   - Activación de poderes especiales (freeze, confuse).
 *   - Detección de gol y actualización de puntuaciones.
 *   - Actualización de la pantalla LCD con el estado del juego.
 *
 * 
return Nunca retorna.
 */
int main() {
    stdio_init_all();

    // Iniciar PIO WS2812
    bool ok = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm_ws, &offset_ws, WS2812_PIN, 1, true);
    hard_assert(ok);
    ws2812_program_init(pio, sm_ws, offset_ws, WS2812_PIN, 800000, IS_RGBW);

    init_pwm(PWM_P1_PIN);
    init_pwm(PWM_P2_PIN);

    // I2C LCD
    i2c_init(I2C_LCD1, 100000);
    gpio_set_function(SDA_LCD1, GPIO_FUNC_I2C);
    gpio_set_function(SCL_LCD1, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_LCD1);
    gpio_pull_up(SCL_LCD1);

    i2c_init(I2C_LCD2, 100000);
    gpio_set_function(SDA_LCD2, GPIO_FUNC_I2C);
    gpio_set_function(SCL_LCD2, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_LCD2);
    gpio_pull_up(SCL_LCD2);

    lcd_init();  // se envía a ambos
    lcd_clear(); // se borra ambos

    // ADC para VRX
    adc_init();
    adc_gpio_init(VRX_P1_PIN);
    adc_gpio_init(VRX_P2_PIN);

    // Joystick y motor P1
    gpio_init(IN1_P1); gpio_set_dir(IN1_P1, GPIO_OUT);
    gpio_init(IN2_P1); gpio_set_dir(IN2_P1, GPIO_OUT);
    gpio_init(SW_P1);  gpio_set_dir(SW_P1, GPIO_IN); gpio_pull_up(SW_P1);

    // Joystick y motor P2
    gpio_init(IN1_P2); gpio_set_dir(IN1_P2, GPIO_OUT);
    gpio_init(IN2_P2); gpio_set_dir(IN2_P2, GPIO_OUT);
    gpio_init(SW_P2);  gpio_set_dir(SW_P2, GPIO_IN); gpio_pull_up(SW_P2);

    // Sensores
    gpio_init(SENSOR_P1_PIN); gpio_set_dir(SENSOR_P1_PIN, GPIO_IN); gpio_pull_up(SENSOR_P1_PIN);
    gpio_init(SENSOR_P2_PIN); gpio_set_dir(SENSOR_P2_PIN, GPIO_IN); gpio_pull_up(SENSOR_P2_PIN);

    // Pulsadores y transistores
    gpio_init(PULSADOR_P1_PIN); gpio_set_dir(PULSADOR_P1_PIN, GPIO_IN); gpio_pull_up(PULSADOR_P1_PIN);
    gpio_init(PULSADOR_P2_PIN); gpio_set_dir(PULSADOR_P2_PIN, GPIO_IN); gpio_pull_up(PULSADOR_P2_PIN);
    gpio_init(TRANSISTOR_P1_PIN); gpio_set_dir(TRANSISTOR_P1_PIN, GPIO_OUT); gpio_put(TRANSISTOR_P1_PIN, 0);
    gpio_init(TRANSISTOR_P2_PIN); gpio_set_dir(TRANSISTOR_P2_PIN, GPIO_OUT); gpio_put(TRANSISTOR_P2_PIN, 0);

    // LED
    gpio_init(LED_P1_A); gpio_set_dir(LED_P1_A, GPIO_OUT); gpio_put(LED_P1_A, 0);
    gpio_init(LED_P1_B); gpio_set_dir(LED_P1_B, GPIO_OUT); gpio_put(LED_P1_B, 0);
    gpio_init(LED_P2_A); gpio_set_dir(LED_P2_A, GPIO_OUT); gpio_put(LED_P2_A, 0);
    gpio_init(LED_P2_B); gpio_set_dir(LED_P2_B, GPIO_OUT); gpio_put(LED_P2_B, 0);


    sleep_ms(1000);
    lcd_init(); lcd_clear(); draw_frame();

    uint32_t last_lcd_update = to_ms_since_boot(get_absolute_time());

    lcd_clear();
    lcd_set_cursor(2, 0);
    lcd_print("Welcome!");
    lcd_set_cursor(0, 1);
    lcd_print("Press button...");

    bool led_state = false;
    uint32_t last_toggle = to_ms_since_boot(get_absolute_time());
    int t_w= 0;
    multicore_launch_core1(core1_ws2812_loop);
    while (gpio_get(PULSADOR_P1_PIN) && gpio_get(PULSADOR_P2_PIN)) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Alternar LEDs cada 300 ms
        if (now - last_toggle > 300) {
            led_state = !led_state;

            gpio_put(LED_P1_A, led_state);
            gpio_put(LED_P1_B, !led_state);
            gpio_put(LED_P2_A, !led_state);
            gpio_put(LED_P2_B, led_state);

            last_toggle = now;
        }

        pattern_futbolito(pio, sm_ws, NUM_PIXELS, t_w);
        t_w++;

        sleep_ms(33);  // ~30 FPS
    }

    // Apagar LEDs al salir del bucle
    gpio_put(LED_P1_A, 0);
    gpio_put(LED_P1_B, 0);
    gpio_put(LED_P2_A, 0);
    gpio_put(LED_P2_B, 0);


    // Confirmar inicio
    lcd_clear();
    lcd_set_cursor(4, 0);
    lcd_print("Starting...");
    sleep_ms(1500);
    lcd_clear();
    draw_frame();

    int t = 0;
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // === Control de Solenoides ===
        gpio_put(TRANSISTOR_P1_PIN, gpio_get(PULSADOR_P1_PIN) == 0);
        gpio_put(TRANSISTOR_P2_PIN, gpio_get(PULSADOR_P2_PIN) == 0);

        // === Lectura de botones ===
        bool joy_btn_p1 = gpio_get(SW_P1) == 0;
        bool pulsador_p1 = gpio_get(PULSADOR_P1_PIN) == 0;
        bool joy_btn_p2 = gpio_get(SW_P2) == 0;
        bool pulsador_p2 = gpio_get(PULSADOR_P2_PIN) == 0;

        // === Lectura de Joysticks ===
        bool freeze_p1 = now - freeze_start_p1 < 5000;
        bool invert_p1 = now - invert_start_p1 < 5000;

        adc_select_input(1);
        sleep_us(10);
        uint16_t adc_val_p1 = adc_read();
        
        
        if (!freeze_p1)
            motor_control_pwm(adc_val_p1, invert_p1, IN1_P1, IN2_P1, PWM_P1_PIN, CENTER_P1);
        else
            motor_control_pwm(2048, false, IN1_P1, IN2_P1, PWM_P1_PIN, CENTER_P1);  // detener


        bool freeze_p2 = now - freeze_start_p2 < 5000;
        bool invert_p2 = now - invert_start_p2 < 5000;

        adc_select_input(2);
        sleep_us(10);
        uint16_t adc_val_p2 = adc_read();

        if (!freeze_p2)
            motor_control_pwm(adc_val_p2, invert_p2, IN1_P2, IN2_P2, PWM_P2_PIN, CENTER_P2);
        else
            motor_control_pwm(2048, false, IN1_P2, IN2_P2, PWM_P2_PIN, CENTER_P2);  // detener


        // === ACTIVACIÓN DE PODERES ===

        // Congelar rival (joystick presionado + pulsador)
        if (!power_freeze_used_p1 && joy_btn_p1 && pulsador_p1) {
            freeze_start_p2 = now;
            power_freeze_used_p1 = true;
            strcpy(lcd_msg, "P1 freeze P2");
            lcd_msg_timeout = now + 5000;
        }

        if (!power_freeze_used_p2 && joy_btn_p2 && pulsador_p2) {
            freeze_start_p1 = now;
            power_freeze_used_p2 = true;
            strcpy(lcd_msg, "P2 freeze P1");
            lcd_msg_timeout = now + 5000;
        }

        // === Invert Control Power (Double Tap)

        // --- Player 1 ---
        if (!joy_btn_p1 && prev_joy_btn_p1) {
            if (!power_invert_used_p1) {
                if (awaiting_double_p1 && (now - last_joy_press_p1 <= 500)) {
                    invert_start_p2 = now;
                    power_invert_used_p1 = true;
                    awaiting_double_p1 = false;
                    strcpy(lcd_msg, "P1 confuse P2");
                    lcd_msg_timeout = now + 5000;
                } else {
                    awaiting_double_p1 = true;
                    last_joy_press_p1 = now;
                }
            }
        }
        if (awaiting_double_p1 && (now - last_joy_press_p1 > 500)) {
            awaiting_double_p1 = false;
        }
        prev_joy_btn_p1 = joy_btn_p1;

        // --- Player 2 ---
        if (!joy_btn_p2 && prev_joy_btn_p2) {
            if (!power_invert_used_p2) {
                if (awaiting_double_p2 && (now - last_joy_press_p2 <= 500)) {
                    invert_start_p1 = now;
                    power_invert_used_p2 = true;
                    awaiting_double_p2 = false;
                    strcpy(lcd_msg, "P2 confuse P1");
                    lcd_msg_timeout = now + 5000;
                } else {
                    awaiting_double_p2 = true;
                    last_joy_press_p2 = now;
                }
            }
        }
        if (awaiting_double_p2 && (now - last_joy_press_p2 > 500)) {
            awaiting_double_p2 = false;
        }
        prev_joy_btn_p2 = joy_btn_p2;

        // === Detección de sensores (inmediata) ===
        bool s1 = gpio_get(SENSOR_P1_PIN);
        bool s2 = gpio_get(SENSOR_P2_PIN);

        if (last_sensor1 && !s1 && score_p1 < WIN_SCORE && score_p2 < WIN_SCORE) {
            score_p1++;
            flash_leds_dual(LED_P1_A, LED_P1_B, 2000);
            animacion_gol_activa = true;
            animacion_gol_inicio = to_ms_since_boot(get_absolute_time());
            gol_de_p1 = true;  // <-- importante
        }

        if (last_sensor2 && !s2 && score_p1 < WIN_SCORE && score_p2 < WIN_SCORE) {
            score_p2++;
            flash_leds_dual(LED_P2_A, LED_P2_B, 2000);
            animacion_gol_activa = true;
            animacion_gol_inicio = to_ms_since_boot(get_absolute_time());
            gol_de_p1 = false;  // <-- importante
        }

        last_sensor1 = s1;
        last_sensor2 = s2;


        t++;
sleep_ms(1);

        // === Actualización LCD cada 300 ms ===
        if (now - last_lcd_update >= 300) {
            if (score_p1 == WIN_SCORE || score_p2 == WIN_SCORE) {
            show_winner(score_p1 == WIN_SCORE ? "Win P1" : "Win P2");

            // Esperar botón para reiniciar
            lcd_set_cursor(0, 1);
            lcd_print("Press to reset");

            // Espera que se suelten
            while (!gpio_get(PULSADOR_P1_PIN) || !gpio_get(PULSADOR_P2_PIN)) sleep_ms(10);
            // Espera que se presione cualquiera
            while (gpio_get(PULSADOR_P1_PIN) && gpio_get(PULSADOR_P2_PIN)) sleep_ms(10);

            // Limpiar estado del juego
            score_p1 = 0;
            score_p2 = 0;
            power_freeze_used_p1 = false;
            power_freeze_used_p2 = false;
            power_invert_used_p1 = false;
            power_invert_used_p2 = false;
            invert_start_p1 = 0;
            invert_start_p2 = 0;
            freeze_start_p1 = 0;
            freeze_start_p2 = 0;
            frame = 0;
            strcpy(lcd_msg, "");
            lcd_msg_timeout = 0;

            lcd_clear();
            lcd_set_cursor(2, 0);
            lcd_print("Welcome!");
            lcd_set_cursor(0, 1);
            lcd_print("Press button...");

            bool led_state = false;
            uint32_t last_toggle = to_ms_since_boot(get_absolute_time());

            while (gpio_get(PULSADOR_P1_PIN) && gpio_get(PULSADOR_P2_PIN)) {
                uint32_t now = to_ms_since_boot(get_absolute_time());

                // Alternar LEDs cada 300 ms
                if (now - last_toggle > 300) {
                    led_state = !led_state;

                    gpio_put(LED_P1_A, led_state);
                    gpio_put(LED_P1_B, !led_state);
                    gpio_put(LED_P2_A, !led_state);
                    gpio_put(LED_P2_B, led_state);

                    last_toggle = now;
                }

                sleep_ms(1);
            }

            // Apagar LEDs al salir del bucle
            gpio_put(LED_P1_A, 0);
            gpio_put(LED_P1_B, 0);
            gpio_put(LED_P2_A, 0);
            gpio_put(LED_P2_B, 0);


            lcd_clear();
            lcd_set_cursor(4, 0);
            lcd_print("Starting...");
            sleep_ms(1500);
            lcd_clear();
            draw_frame();
            }
            else {
                draw_frame();
                if (now < lcd_msg_timeout) {
                    lcd_set_cursor(0, 1);
                    lcd_print("                ");
                    lcd_set_cursor(0, 1);
                    lcd_print(lcd_msg);
                } else {
                    lcd_set_cursor(0, 1);
                    lcd_print("                ");
                }
            }
            frame++;
            last_lcd_update = now;
        }

        sleep_ms(1);
    }
}
