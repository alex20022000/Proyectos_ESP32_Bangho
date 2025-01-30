#include <FreeRTOSConfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <stdio.h>

SemaphoreHandle_t xCountingSemaphore;

void sensor_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore, portMAX_DELAY)) {
            printf("Sensor en uso por la tarea %s\n", pcTaskGetName(NULL));
            vTaskDelay(pdMS_TO_TICKS(5000));  // Simula lectura del sensor
            xSemaphoreGive(xCountingSemaphore);  // Libera el semáforo
        }
    }
}

void app_main() {
    xCountingSemaphore = xSemaphoreCreateCounting(3, 3);  // 3 sensores disponibles

    xTaskCreate(sensor_task, "Sensor Task 1", 2048, NULL, 1, NULL);
    xTaskCreate(sensor_task, "Sensor Task 2", 2048, NULL, 1, NULL);
    xTaskCreate(sensor_task, "Sensor Task 3", 2048, NULL, 1, NULL);
    xTaskCreate(sensor_task, "Sensor Task 4", 2048, NULL, 1, NULL);
}
