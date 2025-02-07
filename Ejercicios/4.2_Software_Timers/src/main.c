/*
Ejercicio 4.2: Timers de software
🔹 Objetivo: Usar un Timer para ejecutar una función periódicamente.
Requisitos:

Crear un Timer que imprima "Timer activado" cada 3 segundos.
Iniciar el Timer en el app_main().
💡 Desafío extra: Crear dos Timers con diferentes períodos y controlar cuándo se activan.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

void TreeSecTimerCallback(TimerHandle_t xTimer);

void app_main()
{
    // Crear timer
    TimerHandle_t xTimer = xTimerCreate("TreeSecTimer", pdMS_TO_TICKS(3000), pdTRUE, 0, TreeSecTimerCallback);
}

void TreeSecTimerCallback(TimerHandle_t xTimer)
{
    printf("Timer activado\n");
}