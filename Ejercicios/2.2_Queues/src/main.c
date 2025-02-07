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
#include <string.h>     // Para strcpy

// Struct para la cola
typedef struct
{
    int sensorID;
    char sensorName[20];
    float value;
} SensorData;

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

void setUPQueue()
{
    xQueue = xQueueCreate(5, sizeof(SensorData));
    if (xQueue == NULL)
    {
        printf("Error al crear la cola.\n");
        return;
    }
}

void setUPTasks()
{
    xTaskCreate(tskRandGenerator, "taskRand", 2048, NULL, 1, NULL);
    xTaskCreate(tskPrinter, "taskPrinter", 2048, NULL, 1, NULL);
}

void tskRandGenerator(void *pvParameters)
{
    SensorData sensorData;
    while (1)
    {
        sensorData.sensorID = (esp_random() % 5) + 1;
        strcpy(sensorData.sensorName, "DHT11");
        sensorData.value = ((float)(esp_random() % 1000)) / 10.0; // Valor entre 0.0 y 99.9
        printf("Generado -> ID: %d, Nombre: %s, Valor: %.1f\n",
               sensorData.sensorID, sensorData.sensorName, sensorData.value);

        // Enviar a la cola
        if (xQueueSend(xQueue, &sensorData, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("Dato enviado a la cola\n");
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
    SensorData receivedData;
    while (1)
    {

        if (xQueueReceive(xQueue, &receivedData, pdMS_TO_TICKS(5000)) == pdTRUE)
        {
            printf("Recibido\nID: %d\n Nombre: %s\n Valor: %.1f\n\n\n",
                   receivedData.sensorID, receivedData.sensorName, receivedData.value);
        }
        else
        {
            printf("Error al recibir el dato de la cola.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}