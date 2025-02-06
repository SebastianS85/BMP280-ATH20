#include <stdio.h>
#include "BMP280.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"


#define TAG "AHT20"

static i2c_master_dev_handle_t bmp280_handle;

esp_err_t bmp280_init(void){


    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };


    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &i2c_bus));
    esp_err_t init_state =i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bmp280_handle);
    return init_state ;
}

void  bmp280_get_id(void){
    uint8_t data[1];
    uint8_t buf[1] = {0xD0};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmp280_handle,buf, sizeof(buf), data, 1, -1));
    ESP_LOGI(TAG, "BMP280 ID: %x", data[0]);
}

void bmp280_get_status(void){
    uint8_t data[2];
    uint8_t buf[1] = {0xF3};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmp280_handle,buf, sizeof(buf), data, 2, -1));
    ESP_LOGI(TAG, "BMP280 Status: %x %x", data[0],data[1]);
}
