idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_lcd esp_driver_i2c esp_driver_gpio esp_timer log freertos bme280
        espressif__esp_lvgl_port lvgl__lvgl
)
