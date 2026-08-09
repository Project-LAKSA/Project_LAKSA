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
