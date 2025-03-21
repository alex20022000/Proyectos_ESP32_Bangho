#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/uart.h"

#define STEP_PIN_1 GPIO_NUM_2
#define DIR_PIN_1 GPIO_NUM_4
#define STEPS_PER_REV 200  // Pasos por revolución
#define DEFAULT_PERIOD_US 500  // 500 µs (2 kHz -> 2 revoluciones por segundo aprox.)

#define BUF_SIZE 1024
#define TX_PIN 1
#define RX_PIN 3
#define CMD_BUFF_SIZE 20

static QueueHandle_t uart_queue;
static esp_timer_handle_t timer_1;
static int T1_pulse_counter = 0;
static int T1_pulse_target = 0;
static bool state_gpio_1 = false;
static int step_period_us = DEFAULT_PERIOD_US;  // Período del timer (ajustable)

static char cmd_buffer[CMD_BUFF_SIZE];
static int cmd_index = 0;

// ISR del timer (cada período es un paso del motor)
void IRAM_ATTR timer_isr_motor_1(void *arg) {
    state_gpio_1 = !state_gpio_1;
    gpio_set_level(STEP_PIN_1, state_gpio_1);
    T1_pulse_counter++;

    if (T1_pulse_counter >= T1_pulse_target) {
        esp_timer_stop(timer_1);
    }
}

// Inicializar timer
void iniciar_timer_motor_1() {
    esp_timer_create_args_t timer_args = {
        .callback = &timer_isr_motor_1,
        .name = "motor_1"
    };
    esp_timer_create(&timer_args, &timer_1);
}

// Mover motor con velocidad ajustable
void mover_motor_1(int grados) {
    int pasos = (STEPS_PER_REV * abs(grados)) / 360;
    gpio_set_level(DIR_PIN_1, (grados >= 0) ? 1 : 0);
    T1_pulse_target = pasos;
    T1_pulse_counter = 0;
    esp_timer_start_periodic(timer_1, step_period_us);  // Velocidad ajustable
}

// Inicializar UART
void iniciar_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Tarea de recepción UART
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[1];

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    while (uart_read_bytes(UART_NUM_0, data, 1, pdMS_TO_TICKS(100)) > 0) {
                        printf("%c", data[0]);
                        fflush(stdout);

                        if (data[0] == '\n') {
                            if (cmd_index > 0 && cmd_buffer[cmd_index - 1] == '\r') {
                                cmd_buffer[cmd_index - 1] = '\0';
                            } else {
                                cmd_buffer[cmd_index] = '\0';
                            }

                            printf("\nComando recibido: %s\n", cmd_buffer);

                            int motor, grados, velocidad;
                            if (sscanf(cmd_buffer, "M%d %d", &motor, &grados) == 2 && motor == 1) {
                                mover_motor_1(grados);
                            } else if (sscanf(cmd_buffer, "SPEED %d", &velocidad) == 1) {
                                if (velocidad > 0) {
                                    step_period_us = velocidad;
                                    printf("🚀 Velocidad ajustada a %d µs por paso\n", step_period_us);
                                } else {
                                    printf("⚠️ Velocidad inválida\n");
                                }
                            } else {
                                printf("⚠️ Comando no reconocido\n");
                            }

                            memset(cmd_buffer, 0, CMD_BUFF_SIZE);
                            cmd_index = 0;
                        } else {
                            if (cmd_index < CMD_BUFF_SIZE - 1) {
                                cmd_buffer[cmd_index++] = data[0];
                            } else {
                                printf("⚠️ Buffer lleno, descartando datos\n");
                                cmd_index = 0;
                            }
                        }
                    }
                    break;

                case UART_BUFFER_FULL:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    printf("⚠️ Error en UART\n");
                    break;

                default:
                    break;
            }
        }
    }
}

void app_main() {
    iniciar_uart();
    gpio_set_direction(STEP_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN_1, GPIO_MODE_OUTPUT);
    iniciar_timer_motor_1();

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 5, NULL);
}
