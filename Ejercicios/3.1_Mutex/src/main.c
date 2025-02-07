/*
Ejercicio 3.1: Protección con Mutex
🔹 Objetivo: Proteger una variable global con un Mutex para evitar condiciones de carrera.
Requisitos:

Dos tareas intentan modificar una variable global (contador_global).
Sin protección, los valores pueden corromperse.
Implementar un Mutex para asegurar que la variable se modifique correctamente.
💡 Desafío extra: Medir el tiempo que cada tarea tarda en obtener el Mutex e imprimirlo en consola.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Prototipos de funciones
void tskInc(void *pvParameters);
void tskDec(void *pvParameters);
void tskBloker(void *pvParameters);
void setUPTasks();
void setUPMutex();

// Variable global
int global_counter = 0;

// Confiugración de mutex
SemaphoreHandle_t xMutex;

void app_main()
{
    setUPMutex();
    setUPTasks();
}

void setUPTasks()
{
    xTaskCreate(tskInc, "Incremental", 2048, NULL, 1, NULL);
    xTaskCreate(tskDec, "Decremental", 2048, NULL, 1, NULL);
    xTaskCreate(tskBloker, "Bloker", 2048, NULL, 1, NULL); // Bloque el mutex un tiempo
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

void tskInc(void *pvParameters)
{
    while (1)
    {
        TickType_t start_time = xTaskGetTickCount();

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            TickType_t end_time = xTaskGetTickCount();
            printf("[Incremental] Tiempo de espera: %lu ms\n", end_time - start_time);
            printf("[Incremental] obtuvo el mutex\n");
            global_counter++;
            printf("[Incremental] Contador: %d\n", global_counter);
            xSemaphoreGive(xMutex);
            printf("[Incremental] liberó el mutex\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void tskDec(void *pvParameters)
{
    while (1)
    {
        TickType_t start_time = xTaskGetTickCount();
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            TickType_t end_time = xTaskGetTickCount();
            printf("[Decremental] Tiempo de espera: %lu ms\n", end_time - start_time);
            printf("[Decremental] obtuvo el mutex\n");
            global_counter--;
            printf("[Decremental] Contador: %d\n", global_counter);
            xSemaphoreGive(xMutex);
            printf("[Deceremental] liberó el mutex\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void tskBloker(void *pvParameters)
{
    while (1)
    {
        if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            printf("🔒 [Blocker] Tomó el mutex y lo bloqueará por 3 segundo...\n");   
            vTaskDelay(pdMS_TO_TICKS(3000));
            xSemaphoreGive(xMutex);
            printf("🔓 [Blocker] Liberó el mutex\n");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 seg para volver a bloquear
    }
    
}