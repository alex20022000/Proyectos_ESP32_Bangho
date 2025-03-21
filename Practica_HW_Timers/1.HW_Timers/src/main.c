/*
Utilizar temporizadores de hardware para mover 3 motores paso a paso de manera continua.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "string.h"

// Definir los pines de paso para los tres motores
#define STEP_PIN_1 GPIO_NUM_14
#define STEP_PIN_2 GPIO_NUM_16
#define STEP_PIN_3 GPIO_NUM_19
// Definir constantes de comunicacion
#define BUF_SIZE 1024
#define RX_PIN 3
#define TX_PIN 1
#define UART_NUM_0 UART_NUM_0
#define CMD_BUFF_SIZE 128

// Parametros iniciales
#define DEFAULT_SPEED 20000

// Prototipos
void calculate_rpm(esp_timer_handle_t timer, const char *nombre_motor);

// UART queue
char cmd_buffer[CMD_BUFF_SIZE];
int cmd_index = 0;

QueueHandle_t uart_queue;

// Variables estáticas volátiles para evitar optimización de compilador
static volatile bool state_1 = false;
static volatile bool state_2 = false;
static volatile bool state_3 = false;

// Manejadores de los timers
esp_timer_handle_t timer_1, timer_2, timer_3;

// Función de callback para el temporizador del motor 1
void IRAM_ATTR timer_isr_motor_1(void *arg)
{
    static bool state_gpio_1 = false;
    state_gpio_1 = !state_gpio_1;
    gpio_set_level(STEP_PIN_1, state_gpio_1);
}

// Función de callback para el temporizador del motor 2
void IRAM_ATTR timer_isr_motor_2(void *arg)
{
    static bool state_gpio_2 = false;
    state_gpio_2 = !state_gpio_2;
    gpio_set_level(STEP_PIN_2, state_gpio_2);
}

// Función de callback para el temporizador del motor 3
void IRAM_ATTR timer_isr_motor_3(void *arg)
{
    static bool state_gpio_3 = false;
    state_gpio_3 = !state_gpio_3;
    gpio_set_level(STEP_PIN_3, state_gpio_3);
}

// Configuración de los temporizadores para los tres motores
void init_timers()
{
    esp_timer_create_args_t timer_args_1 = {
        .callback = &timer_isr_motor_1,
        .arg = NULL,
        .name = "motor_1_timer"};
    esp_timer_create_args_t timer_args_2 = {
        .callback = &timer_isr_motor_2,
        .arg = NULL,
        .name = "motor_2_timer"};
    esp_timer_create_args_t timer_args_3 = {
        .callback = &timer_isr_motor_3,
        .arg = NULL,
        .name = "motor_3_timer"};

    // Verificar errores en la creación de temporizadores
    if (esp_timer_create(&timer_args_1, &timer_1) != ESP_OK)
    {
        printf("Error creando timer 1\n");
    }
    if (esp_timer_create(&timer_args_2, &timer_2) != ESP_OK)
    {
        printf("Error creando timer 2\n");
    }
    if (esp_timer_create(&timer_args_3, &timer_3) != ESP_OK)
    {
        printf("Error creando timer 3\n");
    }

    // Iniciar los temporizadores
    if (esp_timer_start_periodic(timer_1, 200) != ESP_OK)
    {
        printf("Error iniciando timer 1\n");
    }
    if (esp_timer_start_periodic(timer_2, 200) != ESP_OK)
    {
        printf("Error iniciando timer 2\n");
    }
    if (esp_timer_start_periodic(timer_3, 200) != ESP_OK)
    {
        printf("Error iniciando timer 3\n");
    }
}

// Configuracion de UART
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

// Tarea de recepcion UART
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
                            printf("✅ Iniciando todos los motores\n");
                            esp_timer_start_periodic(timer_1, DEFAULT_SPEED);
                            esp_timer_start_periodic(timer_2, DEFAULT_SPEED);
                            esp_timer_start_periodic(timer_3, DEFAULT_SPEED);
                        }
                        else if (strcmp(cmd_buffer, "STOP") == 0)
                        {
                            printf("🛑 Deteniendo todos los motores\n");
                            esp_timer_stop(timer_1);
                            esp_timer_stop(timer_2);
                            esp_timer_stop(timer_3);
                        }
                        /*
                        else if (strncmp(cmd_buffer, "SET SPEED ", 10) == 0)
                        {
                            int speed = atoi(cmd_buffer + 10);
                            printf("🚀 Ajustando velocidad a %d\n", speed);
                            esp_timer_stop(timer_1);
                            esp_timer_stop(timer_2);
                            esp_timer_stop(timer_3);
                            esp_timer_start_periodic(timer_1, speed);
                            esp_timer_start_periodic(timer_2, speed);
                            esp_timer_start_periodic(timer_3, speed);
                        }
                        
                        else if (strncmp(cmd_buffer, "INFO", 4) == 0)
                        {
                            calculate_rpm(timer_1, "Motor 1");
                            calculate_rpm(timer_2, "Motor 2");
                            calculate_rpm(timer_3, "Motor 3");
                        }*/
                        else
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

// Funcion para calcular las RPM
void calculate_rpm(esp_timer_handle_t timer, const char *nombre_motor)
{
    if (esp_timer_is_active(timer))
    {

        // Calcular RPM
        uint64_t periodo_us;
        esp_timer_get_period(timer, &periodo_us); // Obtener período en microsegundos

        if (periodo_us > 0)
        {
            double frecuencia_hz = 1.0e6 / periodo_us; // Convertir a Hz
            double rpm = (frecuencia_hz * 60.0) / 200; // 200 pasos por vuelta

            printf("🔹 %s: %.2f RPM\n", nombre_motor, rpm);
        }
        else
        {
            printf("⚠️ %s no está activo\n", nombre_motor);
        }
    }
    else
    {
        printf("⚠️ %s no está activo\n", nombre_motor);
    }
}

void app_main()
{
    // Configuración de los pines de los motores
    gpio_reset_pin(STEP_PIN_1);
    gpio_set_direction(STEP_PIN_1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(STEP_PIN_2);
    gpio_set_direction(STEP_PIN_2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(STEP_PIN_3);
    gpio_set_direction(STEP_PIN_3, GPIO_MODE_OUTPUT);

    // Inicializar los temporizadores
    init_timers();

    // Iniciamos UART
    init_UART();
    xTaskCreate(uart_event_task, "uart_task", 4096, NULL, 5, NULL);
}
