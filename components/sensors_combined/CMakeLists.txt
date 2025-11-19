idf_component_register(SRCS "sensors_combined.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "AHT20" "BMP280" "driver" "log")