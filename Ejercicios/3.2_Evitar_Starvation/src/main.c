/*
Ejercicio 3.2: Detectar y solucionar Starvation
🔹 Objetivo: Implementar una situación de starvation y corregirla.
Requisitos:

Crear dos tareas con diferentes prioridades que intentan acceder a un Mutex.
La tarea de alta prioridad debe ejecutarse constantemente, causando starvation en la de baja prioridad.
Solucionar el problema activando la prioridad de herencia.
💡 Desafío extra: Implementar la solución usando xSemaphoreTake() con timeout.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

void setUPMutex();
void setUPTasks();

void tskHighPriority(void *pvParameters);
void tskLowPriority(void *pvParameters);

SemaphoreHandle_t xMutex;

void app_main()
{
    // Creamos mutex y tareas
    setUPMutex();
    setUPTasks();
}

void setUPMutex()
{
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL)
    {
        printf("Error al crear el mutex\n");
        return;
    }
}

void setUPTasks()
{
    xTaskCreate(tskHighPriority, "HigherPriority", 2048, NULL, 2, NULL);
    xTaskCreate(tskLowPriority, "LowerPriority", 2048, NULL, 1, NULL);
}

void tskHighPriority(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            printf("Tarea de alta prioridad tomó el mutex y lo mantiene 1 segundo...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            xSemaphoreGive(xMutex);
            printf("Tarea de alta prioridad liberó el mutex\n");
        }
    }
}

void tskLowPriority(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            printf("Tarea de baja prioridad tomó el mutex\n");
            xSemaphoreGive(xMutex);
            printf("Tarea de baja prioridad liberó el mutex\n");
        }
    }
    
}