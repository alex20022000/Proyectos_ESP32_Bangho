/*
Ejercicio 5.2: Comunicación con ESP32 y una PC (Interrupciones y FreeRTOS)
🔹 Objetivo: Crear un sistema donde el ESP32 reciba comandos de una PC vía UART y active tareas en FreeRTOS.
Requisitos:

Implementar una interrupción UART para recibir datos.
Usar una cola para enviar los datos recibidos a una tarea de procesamiento.
💡 Desafío extra: Implementar comandos como "START", "STOP", "SET SPEED" para controlar una acción específica en el ESP32.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <string.h>

#define BUF_SIZE 1024
#define RX_PIN 3
#define TX_PIN 1
#define UART_NUM_0 UART_NUM_0
#define CMD_BUFF_SIZE 128

char cmd_buffer[CMD_BUFF_SIZE];
int cmd_index = 0;

QueueHandle_t uart_queue;

void init_UART();

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[1]; // Leer de a un byte

    while (1)
    {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
                while (uart_read_bytes(UART_NUM_0, data, 1, pdMS_TO_TICKS(100)) > 0)
                {
                    printf("%c", data[0]); // ✅ Mostrar cada carácter ingresado
                    fflush(stdout);        // ✅ Forzar impresión inmediata

                    if (data[0] == '\n')
                    {
                        if (cmd_index > 0 && cmd_buffer[cmd_index - 1] == '\r')
                        {
                            cmd_buffer[cmd_index - 1] = '\0'; // Eliminar \r
                        }
                        else
                        {
                            cmd_buffer[cmd_index] = '\0'; // Terminar string
                        }

                        printf("\nDato recibido: %s\n", cmd_buffer);

                        // Procesar comando
                        if (strcmp(cmd_buffer, "START") == 0)
                        {
                            printf("✅ Comando START recibido\n");
                        }
                        else if (strcmp(cmd_buffer, "STOP") == 0)
                        {
                            printf("🛑 Comando STOP recibido\n");
                        }
                        else if (strncmp(cmd_buffer, "SET SPEED ", 10) == 0)
                        {
                            int speed = atoi(cmd_buffer + 10);
                            printf("🚀 Velocidad establecida a %d\n", speed);
                        }

                        {
                            printf("⚠️ Comando no reconocido\n");
                        }

                        // Reiniciar buffer correctamente
                        memset(cmd_buffer, 0, CMD_BUFF_SIZE);
                        cmd_index = 0;
                    }
                    else
                    {
                        // Agregar carácter al buffer solo si hay espacio
                        if (cmd_index < CMD_BUFF_SIZE - 1)
                        {
                            cmd_buffer[cmd_index++] = data[0];
                        }
                        else
                        {
                            printf("⚠️ Buffer lleno, descartando datos\n");
                            cmd_index = 0; // Evitar corrupción de memoria
                        }
                    }
                }
                break;

            case UART_BREAK:
            case UART_BUFFER_FULL:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
                printf("⚠️  Error en UART\n");
                break;

            default:
                break;
            }
        }
    }
}

void app_main()
{
    init_UART();
    xTaskCreate(uart_event_task, "uart_task", 4096, NULL, 5, NULL);
}

void init_UART()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}