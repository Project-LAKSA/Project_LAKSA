idf_component_register(SRCS "esp_usbcdc_transport.c"
                       INCLUDE_DIRS "."
                       REQUIRES micro_ros_espidf_component
                                espressif__esp_tinyusb)
