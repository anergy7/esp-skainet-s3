idf_component_register(SRCS "ws2812_control.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_rmt espressif__led_strip)