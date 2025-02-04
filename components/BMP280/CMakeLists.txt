idf_component_register(SRCS "BMP280.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver log AHT20 i2c) 
