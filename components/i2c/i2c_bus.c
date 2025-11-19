#include <stdio.h>
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "i2c_bus.h"
#include "esp_log.h"

#define I2C_MASTER_SCL      20
#define I2C_MASTER_SDA      21


i2c_master_bus_handle_t i2c_bus ;

esp_err_t i2c_bus_init(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_io_num = I2C_MASTER_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus);
    if (ret == ESP_OK) {
        ESP_LOGI("I2C_BUS", "I2C bus initialized - SCL: GPIO%d, SDA: GPIO%d, Freq: %dkHz", 
                 I2C_MASTER_SCL, I2C_MASTER_SDA, I2C_MASTER_FREQ_HZ/1000);
    }
    return ret;
}

void i2c_scanner(void) {
    ESP_LOGI("I2C_SCANNER", "Scanning I2C bus for devices...");
    ESP_LOGI("I2C_SCANNER", "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    
    for (int i = 0; i < 128; i += 16) {
        printf("I2C_SCANNER: %02x:", i);
        for (int j = 0; j < 16; j++) {
            uint8_t address = i + j;
            if (address < 0x08 || address > 0x77) {
                printf("   ");  // Invalid address range
                continue;
            }
            
            // Try to probe the device
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = address,
                .scl_speed_hz = I2C_MASTER_FREQ_HZ
            };
            
            i2c_master_dev_handle_t dev_handle;
            esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
            if (ret == ESP_OK) {
                // Try to read from the device
                uint8_t data;
                ret = i2c_master_receive(dev_handle, &data, 1, 100);
                i2c_master_bus_rm_device(dev_handle);
                
                if (ret == ESP_OK) {
                    printf(" %02x", address);  // Device found
                } else {
                    printf(" --");  // Device responded to address but read failed
                }
            } else {
                printf(" --");  // No device at this address
            }
        }
        printf("\n");
    }
    
    ESP_LOGI("I2C_SCANNER", "Expected devices:");
    ESP_LOGI("I2C_SCANNER", "  AHT20: 0x38");
    ESP_LOGI("I2C_SCANNER", "  BMP280: 0x77 (or 0x76)");
}
