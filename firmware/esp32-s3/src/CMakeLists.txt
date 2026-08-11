set(laksa_sources
    "steering_control.c"
    "vesc_uart.cpp"
    "pca9685_i2c.c"
    "main.c"
    "i2c_bus.c"
    "bno08x_adapter.cpp"
    "micro_ros_bridge.c"
)

set(laksa_requirements
    esp_driver_i2c
    esp_driver_spi
    esp_driver_gpio
    esp_driver_uart
    esp32_BNO08x
    arduino-esp32
    VescUart
    micro_ros_espidf_component
    esp_usbcdc_transport
    esp_wifi
    esp_http_server
    esp_netif
    esp_event
    nvs_flash
)

if(CONFIG_LAKSA_WEB_DASHBOARD)
    list(APPEND laksa_sources "web_dashboard.c")
endif()

idf_component_register(
    SRCS ${laksa_sources}
    INCLUDE_DIRS "."
    REQUIRES ${laksa_requirements}
)
