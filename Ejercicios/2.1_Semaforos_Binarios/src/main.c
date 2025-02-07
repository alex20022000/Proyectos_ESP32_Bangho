/*
Ejercicio 2.1: Sincronización con semáforos binarios
🔹 Objetivo: Crear dos tareas sincronizadas con un semáforo binario.
Requisitos:

Una tarea genera un evento ("Presionando botón virtual") cada 2 segundos y libera un semáforo.
Otra tarea espera el semáforo y, cuando lo recibe, imprime "Evento recibido" en consola.
💡 Desafío extra: Modificar el código para que, si la segunda tarea no recibe el semáforo en 3 segundos, muestre un mensaje "Timeout alcanzado".

*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Declaración global del semáforo
SemaphoreHandle_t Semaphore;

// Prototipos de funciones
void tskVirtualButton(void *pvParameters);
void tskWaitForEvent(void *pvParameters);
void setUPTasks();

void app_main()
{
    // Crear semáforo binario
    Semaphore = xSemaphoreCreateBinary();

    // Inicializar tareas
    setUPTasks();
}

void setUPTasks()
{
    xTaskCreate(tskVirtualButton, "taskButton", 2048, NULL, 1, NULL);
    xTaskCreate(tskWaitForEvent, "taskReceiver", 2048, NULL, 1, NULL);
}

// Tarea que simula un botón virtual y libera el semáforo cada 2 segundos
void tskVirtualButton(void *pvParameters)
{
    for (int i=0; i<5; i++)
    {
        printf("Presionando botón virtual\n");
        xSemaphoreGive(Semaphore);       // Libera el semáforo
        vTaskDelay(pdMS_TO_TICKS(2000)); // Espera 2 segundos antes de repetir
    }
    vTaskDelete(NULL); // Destruyo la tarea para que no se reinicie el esp32
}

// Tarea que espera el evento (semáforo) y maneja timeout
void tskWaitForEvent(void *pvParameters)
{
    while (1)
    {
        // Espera hasta 3 segundos por el semáforo
        if (xSemaphoreTake(Semaphore, pdMS_TO_TICKS(3000)) == pdTRUE)
        {
            printf("Evento recibido\n");
        }
        else
        {
            printf("Timeout alcanzado\n");
        }
    }
}