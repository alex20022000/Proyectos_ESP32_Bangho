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
void FiveSecTimerCallback(TimerHandle_t xTimer2);

void app_main()
{
    // Crear timer n°1
    TimerHandle_t xTimer = xTimerCreate("TreeSecTimer", pdMS_TO_TICKS(3000), pdTRUE, 0, TreeSecTimerCallback);
    // Crear timer n°2
    TimerHandle_t xTimer2 = xTimerCreate("FiveSecTimer", pdMS_TO_TICKS(6000), pdTRUE, 0, FiveSecTimerCallback);

    if (xTimer != NULL && xTimer2 != NULL)
    {
        // Iniciamos timer
        xTimerStart(xTimer, 0);
        xTimerStart(xTimer2, 0);
    }
    else
    {
        printf("Error al crear los timers\n");
    }
}

void TreeSecTimerCallback(TimerHandle_t xTimer)
{
    printf("3 sec Timer1 activado\n");
}

void FiveSecTimerCallback(TimerHandle_t xTimer2)
{
    printf("6 sec Timer2 activado\n");
}