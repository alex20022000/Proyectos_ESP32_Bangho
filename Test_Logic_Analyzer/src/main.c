#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BLINK_GPIO GPIO_NUM_4  // GPIO donde parpadeará el LED
#define BLINK_DELAY_MS 500     // Tiempo de parpadeo en milisegundos

void blink_task(void *pvParameter) {
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));

        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
}

void app_main() {
    xTaskCreate(blink_task, "Blink Task", 2048, NULL, 5, NULL);
}
