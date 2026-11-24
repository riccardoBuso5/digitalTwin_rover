#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// Pin L298N
#define ENA_PIN 21
#define IN1_PIN 19 // LED SX AVANTI
#define IN2_PIN 18 // LED SX INDIETRO
#define IN3_PIN 17 // LED DX AVANTI
#define IN4_PIN 16 // LED DX INDIETRO
#define ENB_PIN 4

// LED indicatore rilevamento ostacolo (anche LED onboard ESP32)
#define LED_OSTACOLO_PIN 2

// Configurazione UART0
#define UART_PORT      UART_NUM_0
#define UART_BAUD_RATE 115200
#define BUF_SIZE       256

// Configurazione PWM
#define PWM_FREQ_HZ    1000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT

static float clamp_f(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static void init_gpio_and_pwm(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) |
                        (1ULL << IN3_PIN) | (1ULL << IN4_PIN) |
                        (1ULL << LED_OSTACOLO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_a = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENA_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ch_a);

    ledc_channel_config_t ch_b = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENB_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ch_b);
}

static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
}

static void muovi_motori(float linear_x, float angular_z, int ostacolo) {
    float left  = clamp_f((linear_x - angular_z * 0.5f) * 255.0f, -255.0f, 255.0f);
    float right = clamp_f((linear_x + angular_z * 0.5f) * 255.0f, -255.0f, 255.0f);

    // Motore sinistro: se fermo (|left| < 5), spegne entrambi i LED
    if (fabsf(left) < 5.0f) {
        gpio_set_level(IN1_PIN, 0);
        gpio_set_level(IN2_PIN, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    } else {
        gpio_set_level(IN1_PIN, left > 0.0f ? 1 : 0);
        gpio_set_level(IN2_PIN, left < 0.0f ? 1 : 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)fabsf(left));
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    // Motore destro: se fermo (|right| < 5), spegne entrambi i LED
    if (fabsf(right) < 5.0f) {
        gpio_set_level(IN3_PIN, 0);
        gpio_set_level(IN4_PIN, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    } else {
        gpio_set_level(IN3_PIN, right > 0.0f ? 1 : 0);
        gpio_set_level(IN4_PIN, right < 0.0f ? 1 : 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)fabsf(right));
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // LED ostacolo (pin 2)
    gpio_set_level(LED_OSTACOLO_PIN, ostacolo ? 1 : 0);
}

void app_main(void) {
    init_gpio_and_pwm();
    init_uart();

    char line[BUF_SIZE];
    int idx = 0;
    uint8_t byte;

    while (1) {
        int len = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            if (byte == '\n' || byte == '\r') {
                if (idx > 0) {
                    line[idx] = '\0';
                    float linear_x = 0.0f;
                    float angular_z = 0.0f;
                    int ostacolo = 0;

                    // Accetta sia "linear_x,angular_z,ostacolo" sia "linear_x,angular_z"
                    int parsed = sscanf(line, "%f,%f,%d", &linear_x, &angular_z, &ostacolo);
                    if (parsed >= 2) {
                        if (parsed == 2) {
                            // Fallback se non arriva il flag dal sensore: accende se sta sterzando/frenando
                            ostacolo = (fabsf(angular_z) > 0.05f || fabsf(linear_x) < 0.1f) ? 1 : 0;
                        }
                        muovi_motori(linear_x, angular_z, ostacolo);
                    }
                    idx = 0;
                }
            } else if (idx < BUF_SIZE - 1) {
                line[idx++] = (char)byte;
            }
        }
    }
}