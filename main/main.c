#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include  "AHT20.h"
#include "bmp280.h"
#include "i2c_bus.h"

#define TAG "i2c"

void app_main(void) {
    
  
   i2c_bus_init();
   aht20_init();
   bmp280_init();
  
 
   
    while (true) {

        aht20_read_data();
        vTaskDelay(2000/ portTICK_PERIOD_MS);  
        bmp280_get_id();
        bmp280_get_status();
    }
}