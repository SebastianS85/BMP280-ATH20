#ifndef SENSORS_COMBINED_H
#define SENSORS_COMBINED_H

#include "esp_err.h"

// Combined sensor data structure
typedef struct {
    // AHT20 data
    float aht20_temperature;
    float aht20_humidity;
    uint8_t aht20_status;
    
    // BMP280 data
    float bmp280_temperature;
    float bmp280_pressure_hpa;
    float bmp280_altitude_m;
    
    // Calculated values
    float temperature_diff;     // Difference between AHT20 and BMP280 temperature
    float heat_index;          // Heat index calculation
    float dew_point;           // Dew point calculation
} sensor_data_t;

// Function declarations
esp_err_t read_all_sensors(sensor_data_t *data);
esp_err_t calculate_derived_values(sensor_data_t *data);
void print_all_sensor_data(const sensor_data_t *data);
float calculate_heat_index(float temperature, float humidity);
float calculate_dew_point(float temperature, float humidity);

#endif