idf_component_register(SRCS "pca9685_i2c.c" "main.c"
                            "i2c_bus.c"
                            "bn008x.c"
                            "bn008x_i2c.c"
                            "bn008x_spi.c"
                            INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_spi esp_driver_gpio)
