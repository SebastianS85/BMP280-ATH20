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
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &aht20_handle);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for power-up
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    // Send soft reset command
    uint8_t reset_cmd = 0xBA;
    ret = i2c_master_transmit(aht20_handle, &reset_cmd, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send soft reset");
        return ret;
    }
    
    // Wait for reset to complete
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    // Check if calibration is needed and calibrate if necessary
    uint8_t status_cmd = 0x71;
    uint8_t status;
    ret = i2c_master_transmit_receive(aht20_handle, &status_cmd, 1, &status, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status");
        return ret;
    }
    
    // Check if calibration bit is set (bit 3)
    if (!(status & 0x08)) {
        // Send calibration command
        uint8_t calib_cmd[3] = {0xE1, 0x08, 0x00};
        ret = i2c_master_transmit(aht20_handle, calib_cmd, 3, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send calibration command");
            return ret;
        }
        
        // Wait for calibration to complete
        vTaskDelay(75 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "AHT20 initialized successfully");
    return ret;
}






void  aht20_read_data(void){

uint8_t data[7];
uint8_t buf[1] = {0x71};
esp_err_t ret = i2c_master_transmit(aht20_handle,TEMP_HUM_CMD,sizeof(TEMP_HUM_CMD) ,-1);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send measurement command: %s", esp_err_to_name(ret));
    return;
}
vTaskDelay(80/ portTICK_PERIOD_MS);
ret = i2c_master_transmit_receive(aht20_handle,buf, sizeof(buf), data, 7, -1);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read measurement data: %s", esp_err_to_name(ret));
    return;
}
vTaskDelay(80/ portTICK_PERIOD_MS);


uint32_t temp =((((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) * AHT21_TEMP_SCALE) / AHT21_RESOLUTION - AHT21_TEMP_OFFSET;
uint32_t hum = (((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4)) * AHT21_HUM_SCALE / AHT21_RESOLUTION;
    
    ESP_LOGI(TAG, "Temperature: %2lu C, Humidity: %2lu %%", temp, hum);
    
}

esp_err_t aht20_read_temperature(float *temperature) {
    uint8_t data[7];
    uint8_t buf[1] = {0x71};
    
    esp_err_t ret = i2c_master_transmit(aht20_handle, TEMP_HUM_CMD, sizeof(TEMP_HUM_CMD), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command");
        return ret;
    }
    
    vTaskDelay(80 / portTICK_PERIOD_MS);
    
    ret = i2c_master_transmit_receive(aht20_handle, buf, sizeof(buf), data, 7, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }
    
    // Extract temperature data
    uint32_t temp_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    *temperature = ((float)temp_raw * AHT21_TEMP_SCALE) / AHT21_RESOLUTION - AHT21_TEMP_OFFSET;
    
    ESP_LOGI(TAG, "AHT20 Temperature: %.2f °C", *temperature);
    return ESP_OK;
}

esp_err_t aht20_read_humidity(float *humidity) {
    uint8_t data[7];
    uint8_t buf[1] = {0x71};
    
    esp_err_t ret = i2c_master_transmit(aht20_handle, TEMP_HUM_CMD, sizeof(TEMP_HUM_CMD), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command");
        return ret;
    }
    
    vTaskDelay(80 / portTICK_PERIOD_MS);
    
    ret = i2c_master_transmit_receive(aht20_handle, buf, sizeof(buf), data, 7, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }
    
    // Extract humidity data
    uint32_t hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    *humidity = ((float)hum_raw * AHT21_HUM_SCALE) / AHT21_RESOLUTION;
    
    ESP_LOGI(TAG, "AHT20 Humidity: %.2f %%RH", *humidity);
    return ESP_OK;
}

esp_err_t aht20_read_temp_hum(float *temperature, float *humidity) {
    uint8_t data[7];
    uint8_t buf[1] = {0x71};
    
    esp_err_t ret = i2c_master_transmit(aht20_handle, TEMP_HUM_CMD, sizeof(TEMP_HUM_CMD), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command");
        return ret;
    }
    
    vTaskDelay(80 / portTICK_PERIOD_MS);
    
    ret = i2c_master_transmit_receive(aht20_handle, buf, sizeof(buf), data, 7, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }
    
    // Extract temperature data
    uint32_t temp_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    *temperature = ((float)temp_raw * AHT21_TEMP_SCALE) / AHT21_RESOLUTION - AHT21_TEMP_OFFSET;
    
    // Extract humidity data
    uint32_t hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    *humidity = ((float)hum_raw * AHT21_HUM_SCALE) / AHT21_RESOLUTION;
    
    ESP_LOGI(TAG, "AHT20 - Temperature: %.2f °C, Humidity: %.2f %%RH", *temperature, *humidity);
    return ESP_OK;
}

esp_err_t aht20_get_status(uint8_t *status) {
    uint8_t status_cmd = 0x71;
    esp_err_t ret = i2c_master_transmit_receive(aht20_handle, &status_cmd, 1, status, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status");
        return ret;
    }
    
    ESP_LOGI(TAG, "AHT20 Status: 0x%02X (Busy: %s, Calibrated: %s)", 
             *status, 
             (*status & 0x80) ? "Yes" : "No",
             (*status & 0x08) ? "Yes" : "No");
    return ESP_OK;
}