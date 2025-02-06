#ifndef BMP280_H
#define BMP280_H
#define BMP280_ADDR 0x77
#define BMP280_ID_REG 0xD0


static
#include "driver/i2c_master.h"

esp_err_t bmp280_init(void);
void  bmp280_get_id(void);
void  bmp280_get_status(void);




#endif
