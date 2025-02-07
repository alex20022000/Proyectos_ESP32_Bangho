/*
Ejercicio 2.2: Compartir datos con una cola (Queue)
🔹 Objetivo: Usar una cola para comunicar datos entre tareas.
Requisitos:

Una tarea genera un número aleatorio cada segundo y lo envía a una cola.
Otra tarea lee los datos de la cola y los imprime en consola.
💡 Desafío extra: Usar una estructura en vez de un número simple, enviando un mensaje con "ID de sensor" y "Valor".
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <freertos/queue.h>
#include "esp_random.h" // Para funcion random

// Prototipos de funciones
void tskRandGenerator(void *pvParameters);
void tskPrinter(void *pvParameters);
void setUPTasks();
void setUPQueue();

QueueHandle_t xQueue;

void app_main()
{
    setUPQueue(); // Primero siempre crear la cola
    setUPTasks(); // Luego las tareas
}

void setUPTasks()
{
    xTaskCreate(tskRandGenerator, "taskRand", 2048, NULL, 1, NULL);
    xTaskCreate(tskPrinter, "taskPrinter", 2048, NULL, 1, NULL);
}

void setUPQueue()
{
    xQueue = xQueueCreate(5, sizeof(int));
    if (xQueue == NULL)
    {
        printf("Error al crear la cola.\n");
        return;
    }
}

void tskRandGenerator(void *pvParameters)
{
    while (1)
    {
        int randValue = (esp_random() % 100) + 1; // Genera un número entre 0 y 100
        printf("Número generado: %d\n", randValue);

        // Enviar a la cola
        if (xQueueSend(xQueue, &randValue, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("Dato enviado a la cola: %d\n", randValue);
        }
        else
        {
            printf("Error al enviar el dato a la cola.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tskPrinter(void *pvParameters)
{
    int receivedValue;
    while (1)
    {

        if (xQueueReceive(xQueue, &receivedValue, pdMS_TO_TICKS(5000)) == pdTRUE)
        {
            printf("Dato recibido de la cola: %d\n", receivedValue);
        }
        else
        {
            printf("Error al recibir el dato de la cola.\n");
        }
    }
}