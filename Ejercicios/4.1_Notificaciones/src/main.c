/*
Ejercicio 4.1: Notificaciones de tareas
🔹 Objetivo: Reemplazar el uso de un semáforo con notificaciones de tareas para optimizar el código.
Requisitos:

Crear una tarea que espera una notificación para imprimir "Tarea activada por notificación".
Otra tarea envía la notificación cada 2 segundos.
💡 Desafío extra: Contar cuántas veces ha sido notificada la tarea e imprimir el total.

*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void tskSetUP();
void tskSender(void *pvParameters);
void tskReceiver(void *pvParameters);

// Config de la notificacion
TaskHandle_t xTaskNotify = NULL;
int notify_counter = 0; // Contador de notificaciones recibidas

void app_main()
{
    tskSetUP();
}

void tskSetUP()
{
    xTaskCreate(tskSender, "Notificadora", 2048, NULL, 1, NULL);
    xTaskCreate(tskReceiver, "Notificada", 2048, NULL, 1, &xTaskNotify);
}

void tskSender(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Espera 2 segundos
        if (xTaskNotify != NULL)
        {
            printf("📢 Enviando notificación...\n");
            xTaskNotify(xTaskNotify, 0, eNoAction);
        }
    }
}

void tskReceiver(void *pvParameters)
{
    uint32_t ulNotifiedValue;
    while (1)
    {
        // Esperar indefinidamente una notificación
        if (xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE)
        {
            printf("🔔 Tarea activada por notificación\n");
            notify_counter++;
            printf("🔔 Notificaciones recibidas: %d\n", notify_counter);
        }
    }
}