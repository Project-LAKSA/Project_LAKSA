# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "src")
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sample_project)

# Arduino-ESP32 3.3.10 supports IDF < 6.1, but IDF 6.0 headers trigger
# missing-field-initializer diagnostics when included as C++. Keep the
# workaround local to the Arduino component instead of weakening the project.
idf_component_get_property(arduino_component espressif__arduino-esp32 COMPONENT_LIB)
target_compile_options(${arduino_component} PRIVATE
    -Wno-error=missing-field-initializers
)

# The firmware uses Arduino only for HardwareSerial/VescUart. Arduino's USB
# core embeds a second TinyUSB stack which conflicts with esp_tinyusb, the
# owner of the native USB peripheral used by the micro-ROS CDC transport.
get_target_property(arduino_sources ${arduino_component} SOURCES)
list(FILTER arduino_sources EXCLUDE REGEX
    "/(esp32-hal-tinyusb\\.c|USB\\.cpp|USBCDC\\.cpp|USBMSC\\.cpp|FirmwareMSC\\.cpp|firmware_msc_fat\\.c)$"
)
set_property(TARGET ${arduino_component} PROPERTY SOURCES ${arduino_sources})
