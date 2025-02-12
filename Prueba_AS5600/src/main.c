#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22       // Pin SCL
#define I2C_MASTER_SDA_IO 21       // Pin SDA
#define I2C_MASTER_NUM I2C_NUM_0   // Bus I2C
#define I2C_MASTER_FREQ_HZ 400000  // Frecuencia I2C (400kHz)
#define I2C_TIMEOUT_MS 1000

#define AS5600_ADDR 0x36   // Dirección I2C del AS5600

// Registros del AS5600
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D

static const char *TAG = "AS5600_TEST";

// Función para leer el ángulo del AS5600
float read_encoder_angle() {
    uint8_t data[2];
    uint8_t reg = AS5600_RAW_ANGLE_H;
    
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, AS5600_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    
    if (ret == ESP_OK) {
        uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];  // Combinar bytes
        raw_angle &= 0x0FFF; // El AS5600 usa solo 12 bits

        float angle_deg = (raw_angle / 4096.0) * 360.0;  // Convertir a grados
        return angle_deg;
    } else {
        ESP_LOGE(TAG, "Error leyendo AS5600");
        return -1;
    }
}

// Configuración inicial del I2C
void init_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Tarea para leer el encoder cada 100 ms
void encoder_task(void *pvParameter) {
    while (1) {
        float angle = read_encoder_angle();
        //ESP_LOGI(TAG, "Ángulo: %.2f°", angle);
        printf(">Ángulo:%.2f\n", angle);
        vTaskDelay(pdMS_TO_TICKS(100)); // Leer cada 100 ms
    }
}

void app_main() {
    init_i2c();
    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);
}
