#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_timer.h>

#define PIN_1 GPIO_NUM_15 // Pin de interrupción 1
#define PIN_2 GPIO_NUM_4  // Pin de interrupción 2
#define PIN_3 GPIO_NUM_5  // Pin de interrupción 3

#define DEBOUNCE_TIME_US 80000 // 80ms

static const char *TAG = "INTERRUPCION";
volatile int64_t last_interrupt_time = 0;

// ISR (Interrupción)
void IRAM_ATTR isr_handler(void *arg)
{
    int gpio_num = (int)arg; // Convertir el puntero a entero (GPIO activado)
    int64_t current_time = esp_timer_get_time();

    if (current_time - last_interrupt_time > DEBOUNCE_TIME_US)
    {
        last_interrupt_time = current_time;
        ESP_EARLY_LOGI(TAG, "Interrupción detectada en GPIO %d!", gpio_num);
    }
}

// Inicializar interrupciones en múltiples pines
void init_interrupt()
{
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Flanco de bajada

    // Configurar cada pin
    uint64_t pin_mask = (1ULL << PIN_1) | (1ULL << PIN_2) | (1ULL << PIN_3);
    io_conf.pin_bit_mask = pin_mask;
    gpio_config(&io_conf);

    // Instalar ISR
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    // Registrar la ISR para cada pin
    gpio_isr_handler_add(PIN_1, isr_handler, (void *)PIN_1);
    gpio_isr_handler_add(PIN_2, isr_handler, (void *)PIN_2);
    gpio_isr_handler_add(PIN_3, isr_handler, (void *)PIN_3);
}

void app_main()
{
    init_interrupt();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
