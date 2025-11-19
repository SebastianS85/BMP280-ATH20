#include <math.h>
#include "sensors_combined.h"
#include "AHT20.h"
#include "BMP280.h"
#include "esp_log.h"

#define TAG "SENSORS_COMBINED"

esp_err_t read_all_sensors(sensor_data_t *data) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Reading all sensor values...");
    
    // Read AHT20 data
    ret = aht20_read_temp_hum(&data->aht20_temperature, &data->aht20_humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read AHT20 temperature and humidity");
        return ret;
    }
    
    ret = aht20_get_status(&data->aht20_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read AHT20 status");
        return ret;
    }
    
    // Read BMP280 data
    ret = bmp280_read_all(&data->bmp280_temperature, &data->bmp280_pressure_hpa, &data->bmp280_altitude_m);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 data");
        return ret;
    }
    
    // Calculate derived values
    calculate_derived_values(data);
    
    ESP_LOGI(TAG, "All sensor data read successfully");
    return ESP_OK;
}

esp_err_t calculate_derived_values(sensor_data_t *data) {
    // Calculate temperature difference between sensors
    data->temperature_diff = data->aht20_temperature - data->bmp280_temperature;
    
    // Calculate heat index (only valid for temperatures >= 26.7°C and humidity >= 40%)
    if (data->aht20_temperature >= 26.7f && data->aht20_humidity >= 40.0f) {
        data->heat_index = calculate_heat_index(data->aht20_temperature, data->aht20_humidity);
    } else {
        data->heat_index = data->aht20_temperature; // Heat index = air temperature when conditions not met
    }
    
    // Calculate dew point
    data->dew_point = calculate_dew_point(data->aht20_temperature, data->aht20_humidity);
    
    return ESP_OK;
}

float calculate_heat_index(float temperature, float humidity) {
    // Heat index formula (in Celsius)
    // Simplified approximation for temperatures in Celsius
    float T = temperature;
    float RH = humidity;
    
    float HI = -8.78469475556f +
               1.61139411f * T +
               2.33854883889f * RH +
               -0.14611605f * T * RH +
               -0.012308094f * T * T +
               -0.0164248277778f * RH * RH +
               0.002211732f * T * T * RH +
               0.00072546f * T * RH * RH +
               -0.000003582f * T * T * RH * RH;
    
    return HI;
}

float calculate_dew_point(float temperature, float humidity) {
    // Magnus formula for dew point calculation
    float a = 17.27f;
    float b = 237.7f;
    
    float alpha = ((a * temperature) / (b + temperature)) + logf(humidity / 100.0f);
    float dew_point = (b * alpha) / (a - alpha);
    
    return dew_point;
}

void print_all_sensor_data(const sensor_data_t *data) {
    ESP_LOGI(TAG, "=== COMPLETE SENSOR READOUT ===");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "AHT20 Sensor:");
    ESP_LOGI(TAG, "  Temperature: %.2f °C", data->aht20_temperature);
    ESP_LOGI(TAG, "  Humidity: %.2f %%RH", data->aht20_humidity);
    ESP_LOGI(TAG, "  Status: 0x%02X (Busy: %s, Cal: %s)", 
             data->aht20_status,
             (data->aht20_status & 0x80) ? "Yes" : "No",
             (data->aht20_status & 0x08) ? "Yes" : "No");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "BMP280 Sensor:");
    ESP_LOGI(TAG, "  Temperature: %.2f °C", data->bmp280_temperature);
    ESP_LOGI(TAG, "  Pressure: %.2f hPa", data->bmp280_pressure_hpa);
    ESP_LOGI(TAG, "  Altitude: %.2f m", data->bmp280_altitude_m);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Calculated Values:");
    ESP_LOGI(TAG, "  Temperature Diff: %.2f °C (AHT20 - BMP280)", data->temperature_diff);
    ESP_LOGI(TAG, "  Heat Index: %.2f °C", data->heat_index);
    ESP_LOGI(TAG, "  Dew Point: %.2f °C", data->dew_point);
    ESP_LOGI(TAG, "===============================");
}