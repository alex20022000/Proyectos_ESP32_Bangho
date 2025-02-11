/*
Ejercicio 5.1: Control de un motor paso a paso con FreeRTOS
🔹 Objetivo: Controlar un motor paso a paso usando una tarea periódica en FreeRTOS.
Requisitos:

Crear una tarea que active un GPIO cada 200 µs (para mover el motor).
Usar vTaskDelay() o vTaskDelayUntil().
💡 Desafío extra: Implementar un control de velocidad ajustando dinámicamente el delay.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h>

#define MOTOR_PIN GPIO_NUM_2

void tskMotor(void *pvParameters);

void app_main()
{

    gpio_reset_pin(MOTOR_PIN);
    gpio_set_direction(MOTOR_PIN, GPIO_MODE_OUTPUT);

    xTaskCreate(tskMotor, "Motor", 2048, NULL, 1, NULL);
}

void tskMotor(void *pvParameters)
{
    while (1)
    {

        gpio_set_level(MOTOR_PIN, 1);
        esp_rom_delay_us(200); //delay bloqueante
        gpio_set_level(MOTOR_PIN, 0);
        esp_rom_delay_us(200);
    }
}