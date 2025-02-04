#include <stdio.h>
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "i2c_bus.h"
#include "esp_log.h"

#define I2C_MASTER_SCL      20
#define I2C_MASTER_SDA      21


i2c_master_bus_handle_t i2c_bus = NULL;

esp_err_t i2c_bus_init(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_io_num = I2C_MASTER_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    return i2c_new_master_bus(&i2c_bus_config, &i2c_bus);
}
