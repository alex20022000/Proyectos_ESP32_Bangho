/*
Ejercicio 1.2: Crear y eliminar tareas dinámicamente
🔹 Objetivo: Crear una tarea que se elimine a sí misma después de ejecutarse un número determinado de veces.
Requisitos:

Crear una tarea que imprima "Ejecutando tarea temporal" hasta llegar a 10 repeticiones.
Después de la décima ejecución, la tarea debe eliminarse con vTaskDelete(NULL).
💡 Desafío extra: Crear otra tarea que monitoree cuántas tareas están activas en el sistema usando uxTaskGetNumberOfTasks().

*/

#include <FreeRTOSConfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

void set_Up_Tasks();
void taskTemp(void *pvParameters);

void app_main()
{

    set_Up_Tasks();
}

void set_Up_Tasks(){
    xTaskCreate(taskTemp,"TareaTEmporal",1024,NULL, 1, NULL);
}

void taskTemp(void *pvParameters){
    vTaskDelay(pdMS_TO_TICKS(10000));
    for(int i = 0; i<10; i++){
        printf("Ejecutando tarea temporal en el ciclo: %d\n", i+1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("Tarea temporal finalizada\n");
    fflush(stdout);  // Asegurar que el mensaje se imprime antes de eliminar la tarea
    vTaskDelay(pdMS_TO_TICKS(100));  // Pequeño delay para garantizar impresión
    vTaskDelete(NULL);  // Se elimina a sí misma
}