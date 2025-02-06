/*

🔹 Unidad 1: Conceptos básicos y tareas
Ejercicio 1.1: Creación de dos tareas simples
🔹 Objetivo: Crear dos tareas con FreeRTOS que impriman mensajes en consola con diferente frecuencia.
Requisitos:

Crear una tarea que imprima "Hola desde la Tarea 1" cada 500 ms.
Crear otra tarea que imprima "Hola desde la Tarea 2" cada 1000 ms.
Usar vTaskDelay() para la temporización.
💡 Desafío extra: Modificar la prioridad de las tareas y observar el comportamiento.

*/

#include <FreeRTOSConfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

void set_Up_Tasks();
void tarea1(void *pvParameters);
void tarea2(void *pvParameters);



void app_main() {
    set_Up_Tasks();
}

void set_Up_Tasks(){
    xTaskCreate(tarea1,"Task1",1024, NULL, 2, NULL);
    xTaskCreate(tarea2,"Task2",1024, NULL, 1, NULL);
}

void tarea1(void *pvParameters){
    while (1)
    {
        printf("Hola desde la Tarea 1\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void tarea2(void *pvParameters){
    while (1)
    {
        printf("Hola desde la Tarea 2\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}