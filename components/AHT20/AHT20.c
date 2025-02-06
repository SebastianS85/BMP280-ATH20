#include <stdio.h>
#include "AHT20.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"



#define TAG "AHT20"
#define AHT20_ADDR 0x38

static i2c_master_dev_handle_t aht20_handle = NULL;

esp_err_t aht20_init(void){


    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };
    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &i2c_bus_handle));
    return i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &aht20_handle);
}






void  aht20_read_data(void){

uint8_t data[7];
uint8_t buf[1] = {0x71};
ESP_ERROR_CHECK(i2c_master_transmit(aht20_handle,TEMP_HUM_CMD,sizeof(TEMP_HUM_CMD) ,-1));
vTaskDelay(80/ portTICK_PERIOD_MS);
ESP_ERROR_CHECK(i2c_master_transmit_receive(aht20_handle,buf, sizeof(buf), data, 7, -1));
vTaskDelay(80/ portTICK_PERIOD_MS);


uint32_t temp =((((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) * AHT21_TEMP_SCALE) / AHT21_RESOLUTION - AHT21_TEMP_OFFSET;
uint32_t hum = (((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4)) * AHT21_HUM_SCALE / AHT21_RESOLUTION;
    
    ESP_LOGI(TAG, "Temperature: %2lu C, Humidity: %2lu %%", temp, hum);
    
}