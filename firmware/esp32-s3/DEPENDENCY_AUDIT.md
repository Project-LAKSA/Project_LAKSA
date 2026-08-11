# Auditoría de dependencias del firmware

## Decisiones

- **USB micro-ROS:** conservar `esp_usbcdc_transport`. Sus cuatro callbacks no
  implementan un protocolo propio: son la adaptación requerida por el API de
  transporte custom de micro-ROS y delegan la E/S a `esp_tinyusb` de Espressif.
- **BNO08x:** conservar `esp32_BNO08x` y el adaptador C. El driver de comunidad
  ya resuelve SHTP/SH-2; el adaptador sólo expone al código C los reportes que
  utiliza el firmware.
- **VESC UART:** conservar `VescUart`. No se encontró un componente ESP-IDF
  UART equivalente y maduro. Cambiar al componente VESC CAN requeriría cambiar
  la interfaz eléctrica a TWAI/CAN; reimplementar el paquete UART localmente
  sería precisamente duplicar una biblioteca probada.
- **PCA9685:** conservar el driver local. Es pequeño y usa directamente el API
  I2C master actual de ESP-IDF. Sustituirlo por `esp-idf-lib/pca9685` añadiría
  otra capa I2C y más dependencias sin eliminar lógica del producto.
- **Dashboard:** ahora es seleccionable con `CONFIG_LAKSA_WEB_DASHBOARD`. Se
  mantiene activo por defecto, pero el perfil micro-ROS-only ahorra cerca de
  638 KiB de flash.

## Limpieza aplicada

- Eliminados `laksa_ecu.c` y `laksa_ecu.h`: 190 líneas huérfanas que no estaban
  en CMake ni tenían consumidores.
- El workspace embebido se limita al cierre de dependencias de `rclc`,
  `sensor_msgs` y `laksa_interfaces`: 37 paquetes en vez de 66.
- Desactivados los transportes WLAN/Ethernet internos del componente micro-ROS;
  LAKSA usa exclusivamente TinyUSB CDC.
- El build host usa `g++` para C++ y ejecución secuencial, lo que permite una
  reconstrucción limpia en macOS con ESP-IDF 6.0/Python 3.14.
- Los archivos `.msg` y `.srv`, así como el Makefile de micro-ROS, invalidan la
  librería generada cuando cambian; no se conservan headers obsoletos.

## Costos que permanecen

`VescUart` depende de Arduino como componente ESP-IDF. Aunque el linker sólo
incluye la parte usada (alrededor de 17 KiB en la imagen medida), el gestor de
componentes descarga y puede compilar muchas dependencias transitivas. Quitarlo
sin cambiar hardware implicaría mantener una implementación propia del
protocolo VESC UART, por lo que no se recomienda todavía.

Los directorios `build/`, `managed_components/`, `micro_ros_src/` y
`micro_ros_dev/` son generados e ignorados por Git. Explican gran parte del
tamaño en disco, pero no son código fuente del proyecto. Tras el recorte, el
árbol generado de micro-ROS bajó aproximadamente de 663 MiB a 406 MiB.

## Pendientes del repositorio padre

`components/esp32_BNO08x` aparece como gitlink, pero el repositorio padre no
contiene una entrada correspondiente en `.gitmodules`. Hay que registrar el
submódulo en la raíz de Project_LAKSA para que un checkout limpio sea
reproducible. Esa raíz queda fuera del alcance de este firmware.
