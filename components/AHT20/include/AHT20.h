#ifndef AHT20_H
#define AHT20_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#define AHT20_ADDRESS 0x38
#define AHT20_STATUS_REG 0x71

#define AHT21_TEMP_SCALE 200
#define AHT21_HUM_SCALE 100
#define AHT21_TEMP_OFFSET 50
#define AHT21_RESOLUTION 1048576

static const uint8_t TEMP_HUM_CMD[] = {0xAC,0x33, 0x00};

esp_err_t aht20_init(void);
void aht20_read_data();
esp_err_t aht20_read_temperature(float *temperature);
esp_err_t aht20_read_humidity(float *humidity);
esp_err_t aht20_read_temp_hum(float *temperature, float *humidity);
esp_err_t aht20_get_status(uint8_t *status);

#endif
