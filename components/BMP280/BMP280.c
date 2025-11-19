#include <stdio.h>
#include <string.h>
#include <math.h>
#include "BMP280.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

#define TAG "BMP280"

static i2c_master_dev_handle_t bmp280_handle;

// BMP280 calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

static bmp280_calib_data_t calib_data;
static int32_t t_fine; // Global variable for temperature compensation

esp_err_t bmp280_init(void){

    // Try both possible BMP280 addresses
    uint8_t bmp280_addresses[] = {0x77, 0x76};
    esp_err_t init_state = ESP_FAIL;
    
    for (int i = 0; i < 2; i++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = bmp280_addresses[i],
            .scl_speed_hz = I2C_MASTER_FREQ_HZ
        };

        i2c_master_bus_handle_t i2c_bus;
        ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &i2c_bus));
        init_state = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bmp280_handle);
        
        if (init_state == ESP_OK) {
            ESP_LOGI(TAG, "BMP280 found at address 0x%02X", bmp280_addresses[i]);
            
            // Test communication by reading ID
            uint8_t id_reg = 0xD0;
            uint8_t chip_id;
            esp_err_t id_ret = i2c_master_transmit_receive(bmp280_handle, &id_reg, 1, &chip_id, 1, -1);
            if (id_ret == ESP_OK) {
                ESP_LOGI(TAG, "BMP280 Chip ID: 0x%02X", chip_id);
                if (chip_id == 0x58) {
                    // Valid BMP280 chip ID, wait a bit and then configure
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    
                    // Soft reset the sensor first
                    uint8_t reset_cmd[2] = {0xE0, 0xB6};
                    esp_err_t reset_ret = i2c_master_transmit(bmp280_handle, reset_cmd, 2, -1);
                    if (reset_ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to reset BMP280: %s", esp_err_to_name(reset_ret));
                    } else {
                        ESP_LOGI(TAG, "BMP280 reset successfully");
                        vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reset to complete
                    }
                    
                    // Read calibration data first (before configuration)
                    if (bmp280_read_calibration() == ESP_OK) {
                        if (bmp280_configure() == ESP_OK) {
                            ESP_LOGI(TAG, "BMP280 initialization complete");
                            return ESP_OK;
                        } else {
                            ESP_LOGE(TAG, "BMP280 configuration failed");
                        }
                    } else {
                        ESP_LOGE(TAG, "BMP280 calibration failed");
                    }
                } else {
                    ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x58)", chip_id);
                }
            } else {
                ESP_LOGE(TAG, "Failed to read chip ID from address 0x%02X", bmp280_addresses[i]);
            }
            
            // Clean up this attempt
            i2c_master_bus_rm_device(bmp280_handle);
            bmp280_handle = NULL;
        }
    }
    
    ESP_LOGE(TAG, "BMP280 not found at any address (0x76, 0x77)");
    return ESP_FAIL;
}

esp_err_t bmp280_configure(void) {
    uint8_t config_data[2];
    
    // Set control register (0xF4) - normal mode, pressure and temperature enabled
    config_data[0] = 0xF4;
    config_data[1] = 0x27; // osrs_t=001, osrs_p=001, mode=11 (normal mode)
    esp_err_t ret = i2c_master_transmit(bmp280_handle, config_data, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure control register");
        return ret;
    }
    
    // Set config register (0xF5) - standby time and filter
    config_data[0] = 0xF5;
    config_data[1] = 0xA0; // t_sb=101 (1000ms), filter=000, spi3w_en=0
    ret = i2c_master_transmit(bmp280_handle, config_data, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure config register");
        return ret;
    }
    
    ESP_LOGI(TAG, "BMP280 configured successfully");
    return ESP_OK;
}

esp_err_t bmp280_read_calibration(void) {
    uint8_t calib_reg = 0x88; // Calibration data starts at 0x88
    uint8_t calib_raw[24]; // 24 bytes of calibration data
    
    // Clear the buffer first
    memset(calib_raw, 0, sizeof(calib_raw));
    
    ESP_LOGI(TAG, "Reading calibration data from register 0x88...");
    esp_err_t ret = i2c_master_transmit_receive(bmp280_handle, &calib_reg, 1, calib_raw, 24, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Log raw calibration data for debugging
    ESP_LOGI(TAG, "Raw calibration data:");
    for (int i = 0; i < 24; i += 8) {
        ESP_LOGI(TAG, "  [%02d-%02d]: %02x %02x %02x %02x %02x %02x %02x %02x", 
                 i, i+7,
                 calib_raw[i], calib_raw[i+1], calib_raw[i+2], calib_raw[i+3],
                 calib_raw[i+4], calib_raw[i+5], calib_raw[i+6], calib_raw[i+7]);
    }
    
    // Parse calibration data (little endian)
    calib_data.dig_T1 = (calib_raw[1] << 8) | calib_raw[0];
    calib_data.dig_T2 = (calib_raw[3] << 8) | calib_raw[2];
    calib_data.dig_T3 = (calib_raw[5] << 8) | calib_raw[4];
    calib_data.dig_P1 = (calib_raw[7] << 8) | calib_raw[6];
    calib_data.dig_P2 = (calib_raw[9] << 8) | calib_raw[8];
    calib_data.dig_P3 = (calib_raw[11] << 8) | calib_raw[10];
    calib_data.dig_P4 = (calib_raw[13] << 8) | calib_raw[12];
    calib_data.dig_P5 = (calib_raw[15] << 8) | calib_raw[14];
    calib_data.dig_P6 = (calib_raw[17] << 8) | calib_raw[16];
    calib_data.dig_P7 = (calib_raw[19] << 8) | calib_raw[18];
    calib_data.dig_P8 = (calib_raw[21] << 8) | calib_raw[20];
    calib_data.dig_P9 = (calib_raw[23] << 8) | calib_raw[22];
    
    // Validate calibration data
    if (calib_data.dig_T1 == 0 || calib_data.dig_P1 == 0) {
        ESP_LOGE(TAG, "Invalid calibration data (T1=%u, P1=%u)", calib_data.dig_T1, calib_data.dig_P1);
        
        // Check if all bytes are 0xFF (unprogrammed EEPROM)
        bool all_ff = true;
        bool all_zero = true;
        for (int i = 0; i < 24; i++) {
            if (calib_raw[i] != 0xFF) all_ff = false;
            if (calib_raw[i] != 0x00) all_zero = false;
        }
        
        if (all_ff) {
            ESP_LOGE(TAG, "Calibration EEPROM appears unprogrammed (all 0xFF)");
        } else if (all_zero) {
            ESP_LOGE(TAG, "Calibration EEPROM appears blank (all 0x00) - possible read error");
        }
        
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Calibration data read successfully");
    ESP_LOGI(TAG, "  T1=%u, T2=%d, T3=%d", calib_data.dig_T1, calib_data.dig_T2, calib_data.dig_T3);
    ESP_LOGI(TAG, "  P1=%u, P2=%d, P3=%d", calib_data.dig_P1, calib_data.dig_P2, calib_data.dig_P3);
    return ESP_OK;
}

esp_err_t bmp280_read_temperature(float *temperature) {
    if (bmp280_handle == NULL) {
        ESP_LOGE(TAG, "BMP280 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[3] = {0};
    uint8_t buf[1] = {0xFA}; // Temperature register starts at 0xFA
    esp_err_t ret = i2c_master_transmit_receive(bmp280_handle, buf, sizeof(buf), data, 3, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Combine bytes into a 20-bit value
    int32_t adc_T = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    
    // Check for valid reading
    if (adc_T == 0x80000) {
        ESP_LOGE(TAG, "Temperature sensor disabled or reading invalid");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Temperature compensation (from BMP280 datasheet)
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    *temperature = (float)T / 100.0f;
    
    ESP_LOGI(TAG, "Temperature: %.2f °C (raw: %ld, t_fine: %ld)", *temperature, adc_T, t_fine);
    return ESP_OK;
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

esp_err_t read_pressure_raw() {
    uint8_t data[3] = {0};
    uint8_t buf[1] = {0xF7};
    esp_err_t ret = i2c_master_transmit_receive(bmp280_handle,buf, sizeof(buf), data, 3, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure data");
        return ret;
    }

    // Combine bytes into a 20-bit value
    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    
    // Pressure compensation (from BMP280 datasheet)
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1*(int64_t)calib_data.dig_P5)<<17);
    var2 = var2 + (((int64_t)calib_data.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3)>>8) + ((var1 * (int64_t)calib_data.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib_data.dig_P1)>>33;
    
    if (var1 == 0) {
        ESP_LOGE(TAG, "Pressure calculation error - division by zero");
        return ESP_FAIL;
    }
    
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7)<<4);
    
    float pressure_pa = (float)p / 256.0f;
    float pressure_hpa = pressure_pa / 100.0f;

    ESP_LOGI(TAG, "Pressure: %.2f hPa (%.2f Pa)", pressure_hpa, pressure_pa);
    return ESP_OK;
}

esp_err_t bmp280_read_pressure(float *pressure_hpa) {
    uint8_t data[3] = {0};
    uint8_t buf[1] = {0xF7};
    esp_err_t ret = i2c_master_transmit_receive(bmp280_handle, buf, sizeof(buf), data, 3, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure data");
        return ret;
    }

    // Combine bytes into a 20-bit value
    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    
    // Pressure compensation (from BMP280 datasheet)
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1*(int64_t)calib_data.dig_P5)<<17);
    var2 = var2 + (((int64_t)calib_data.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3)>>8) + ((var1 * (int64_t)calib_data.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib_data.dig_P1)>>33;
    
    if (var1 == 0) {
        ESP_LOGE(TAG, "Pressure calculation error - division by zero");
        return ESP_FAIL;
    }
    
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7)<<4);
    
    float pressure_pa = (float)p / 256.0f;
    *pressure_hpa = pressure_pa / 100.0f;

    ESP_LOGI(TAG, "BMP280 Pressure: %.2f hPa", *pressure_hpa);
    return ESP_OK;
}

esp_err_t bmp280_calculate_altitude(float pressure_hpa, float *altitude_m) {
    // Standard atmospheric pressure at sea level (1013.25 hPa)
    float sea_level_pressure = 1013.25f;
    
    // Barometric formula for altitude calculation
    *altitude_m = 44330.0f * (1.0f - powf(pressure_hpa / sea_level_pressure, 0.1903f));
    
    ESP_LOGI(TAG, "Calculated Altitude: %.2f meters", *altitude_m);
    return ESP_OK;
}

esp_err_t bmp280_read_all(float *temperature, float *pressure_hpa, float *altitude_m) {
    esp_err_t ret;
    
    // Read temperature first (needed for pressure compensation)
    ret = bmp280_read_temperature(temperature);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read pressure
    ret = bmp280_read_pressure(pressure_hpa);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Calculate altitude
    ret = bmp280_calculate_altitude(*pressure_hpa, altitude_m);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "BMP280 All Values - Temp: %.2f°C, Pressure: %.2f hPa, Altitude: %.2f m", 
             *temperature, *pressure_hpa, *altitude_m);
    return ESP_OK;
}

