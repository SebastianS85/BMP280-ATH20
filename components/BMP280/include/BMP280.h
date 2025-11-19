#ifndef BMP280_H
#define BMP280_H
#define BMP280_ADDR 0x77
#define BMP280_ID_REG 0xD0
#define PRESSURE_MSB_REG     0xF7  



#include "driver/i2c_master.h"

esp_err_t bmp280_init(void);
esp_err_t bmp280_configure(void);
esp_err_t bmp280_read_calibration(void);
esp_err_t bmp280_read_temperature(float *temperature);
esp_err_t bmp280_read_pressure(float *pressure_hpa);
esp_err_t bmp280_calculate_altitude(float pressure_hpa, float *altitude_m);
esp_err_t bmp280_read_all(float *temperature, float *pressure_hpa, float *altitude_m);
void  bmp280_get_id(void);
void  bmp280_get_status(void);
esp_err_t read_pressure_raw();

#endif
