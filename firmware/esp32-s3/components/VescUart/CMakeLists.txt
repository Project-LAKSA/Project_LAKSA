idf_component_register(
    SRCS "src/VescUart.cpp" "src/buffer.cpp" "src/crc.cpp"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32
)

