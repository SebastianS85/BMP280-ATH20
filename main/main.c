#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include  "AHT20.h"
#include "BMP280.h"
#include "i2c_bus.h"

#define TAG "i2c"

void app_main(void) {
    
    ESP_LOGI(TAG, "Starting I2C sensor demo...");
    
    // Initialize I2C bus
    esp_err_t ret = i2c_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    
    // Initialize AHT20
    ret = aht20_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AHT20: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "AHT20 initialized successfully");
    }
    
    // Initialize BMP280  
    ret = bmp280_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "BMP280 initialized successfully");
    }
    
    // Wait a bit before starting readings
    vTaskDelay(1000 / portTICK_PERIOD_MS);
   
    while (true) {
        ESP_LOGI(TAG, "\n=== SENSOR READING CYCLE ===\n");
        
        // AHT20 readings - test each function individually
        float aht_temp, aht_hum;
        uint8_t aht_status;
        
        ESP_LOGI(TAG, "Testing AHT20 functions...");
        if (aht20_read_temperature(&aht_temp) == ESP_OK) {
            ESP_LOGI(TAG, "✓ AHT20 Temperature: %.2f °C", aht_temp);
        } else {
            ESP_LOGI(TAG, "✗ AHT20 Temperature read failed");
        }
        
        if (aht20_read_humidity(&aht_hum) == ESP_OK) {
            ESP_LOGI(TAG, "✓ AHT20 Humidity: %.2f %%RH", aht_hum);
        } else {
            ESP_LOGI(TAG, "✗ AHT20 Humidity read failed");
        }
        
        if (aht20_get_status(&aht_status) == ESP_OK) {
            ESP_LOGI(TAG, "✓ AHT20 Status: 0x%02X", aht_status);
        } else {
            ESP_LOGI(TAG, "✗ AHT20 Status read failed");
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // BMP280 readings
        float bmp_temp, bmp_pressure, bmp_altitude;
        
        ESP_LOGI(TAG, "Testing BMP280 functions...");
        if (bmp280_read_temperature(&bmp_temp) == ESP_OK) {
            ESP_LOGI(TAG, "✓ BMP280 Temperature: %.2f °C", bmp_temp);
        } else {
            ESP_LOGI(TAG, "✗ BMP280 Temperature read failed");
        }
        
        if (bmp280_read_pressure(&bmp_pressure) == ESP_OK) {
            ESP_LOGI(TAG, "✓ BMP280 Pressure: %.2f hPa", bmp_pressure);
            
            if (bmp280_calculate_altitude(bmp_pressure, &bmp_altitude) == ESP_OK) {
                ESP_LOGI(TAG, "✓ BMP280 Altitude: %.2f m", bmp_altitude);
            }
        } else {
            ESP_LOGI(TAG, "✗ BMP280 Pressure read failed");
        }
        
        // Test legacy functions (these handle their own errors now)
        ESP_LOGI(TAG, "Testing legacy functions...");
        aht20_read_data();
        bmp280_get_id();
        bmp280_get_status();
        read_pressure_raw();
        
        ESP_LOGI(TAG, "\n=== End Cycle - Waiting 10 seconds ===\n");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}