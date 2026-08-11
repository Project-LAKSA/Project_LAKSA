# LAKSA ESP32-S3 — micro-ROS por USB

El firmware usa el USB nativo del ESP32-S3 como dispositivo CDC para micro-ROS.
El puerto de programación puede permanecer conectado al monitor serial; el de
la Jetson debe ser el que la tarjeta enruta directamente a D-/D+ del S3 (GPIO
19/20). En esta tarjeta se espera que sea el USB-C inferior con la antena a la
izquierda, pero hay que confirmarlo en el esquemático de la placa.

## Interfaz ROS 2

| Nombre | Tipo | Frecuencia/uso |
|---|---|---|
| `/laksa/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz, best effort |
| `/laksa/imu/mag` | `sensor_msgs/msg/MagneticField` | 20 Hz, best effort |
| `/laksa/vesc/state` | `laksa_interfaces/msg/VescState` | 10 Hz, best effort |
| `/laksa/state` | `laksa_interfaces/msg/VehicleState` | 10 Hz, best effort |
| `/laksa/command` | `laksa_interfaces/msg/DriveCommand` | velocidad y dirección |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | entrada compatible con Nav2 |
| `/laksa/set_drive_command` | `laksa_interfaces/srv/SetDriveCommand` | escritura puntual |
| `/laksa/get_state` | `laksa_interfaces/srv/GetVehicleState` | lectura puntual |

`DriveCommand.speed_mps` es velocidad lineal y `steering_angle_rad` es el
ángulo de las ruedas delanteras; positivo gira a la izquierda. `/cmd_vel` se
convierte a dirección Ackermann con
`atan(wheelbase * angular.z / linear.x)`.

El VESC reporta eRPM (RPM eléctricas). El firmware aplica:

```text
wheel_rpm = erpm / (pole_pairs * gear_reduction)
speed_mps = wheel_rpm * (pi * wheel_diameter_m) / 60
```

No hace falta otro wrapper en la Jetson: publica eRPM crudas, rad/s mecánicos
del motor, rad/s de rueda y m/s del vehículo.

## Calibrar antes de mover el vehículo

Los valores iniciales son marcadores, no mediciones del vehículo. Configurar en
`idf.py menuconfig` > **LAKSA vehicle configuration**:

- pares de polos del motor (polos magnéticos / 2);
- reducción motor:a-rueda;
- diámetro efectivo de la llanta;
- distancia entre ejes;
- ángulo máximo físico de las ruedas;
- signo de avance del VESC;
- `ROS_DOMAIN_ID`, igual al de la Jetson.

La IMU usa `frame_id: imu_link`. Su montaje debe respetar REP-103; si no,
publicar la transformación fija correcta en la Jetson. Las covarianzas quedan
en cero cuando existe medición y deben sustituirse por valores obtenidos de una
calibración experimental para fusión sensorial de producción.

## Compilar y grabar

La integración está fijada a micro-ROS Humble y ESP-IDF 6.0.1:

```bash
source ~/.espressif/v6.0.1/esp-idf/export.sh
python -m pip install catkin_pkg colcon-common-extensions lark 'empy<4'
idf.py menuconfig
idf.py build
idf.py -p /dev/cu.PUERTO_DE_PROGRAMACION flash monitor
```

El primer build necesita red para descargar fuentes y componentes. El USB
nativo de datos no se usa para `flash` en esta configuración.

### Perfil sin dashboard Wi-Fi

Para una instalación dedicada a la Jetson, desactivar en `idf.py menuconfig`:

```text
LAKSA vehicle configuration
  [ ] Enable Wi-Fi web dashboard
```

Esto conserva toda la interfaz micro-ROS y omite el código del dashboard. En
la compilación de referencia reduce la imagen de `0x108800` a `0x68ec0` bytes
(aproximadamente 638 KiB) y deja 72% libre en la partición de aplicación. La
opción permanece activada por defecto para conservar el comportamiento previo.

## Jetson con ROS 2 Humble

Copiar las interfaces a un workspace de la Jetson y compilarlas:

```bash
mkdir -p ~/laksa_ws/src
cp -r extra_ros_packages/laksa_interfaces ~/laksa_ws/src/
cd ~/laksa_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select laksa_interfaces
source install/setup.bash
```

Conectar el USB-C nativo, localizarlo y dar acceso al usuario (cerrar sesión una
vez después de agregar el grupo):

```bash
ls -l /dev/ttyACM*
sudo usermod -aG dialout "$USER"
```

Iniciar el Agent Humble instalado localmente:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/laksa_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v4
```

Si no está instalado, usar el contenedor oficial:

```bash
docker run --rm -it --privileged --net=host \
  -v /dev:/dev -v /dev/shm:/dev/shm \
  microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v4
```

## Prueba segura

Primero levantar las ruedas motrices. Confirmar datos antes de habilitar
movimiento:

```bash
ros2 topic list
ros2 topic echo /laksa/imu/data
ros2 topic echo /laksa/state
ros2 service call /laksa/get_state laksa_interfaces/srv/GetVehicleState '{}'
```

Mandar comandos continuamente; el watchdog existente detiene el VESC si no se
renuevan en 500 ms:

```bash
ros2 topic pub -r 10 /laksa/command laksa_interfaces/msg/DriveCommand \
  '{speed_mps: 0.20, steering_angle_rad: 0.0}'

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.20}, angular: {z: 0.0}}'
```

Detener explícitamente:

```bash
ros2 topic pub --once /laksa/command laksa_interfaces/msg/DriveCommand \
  '{speed_mps: 0.0, steering_angle_rad: 0.0}'
```

El servicio de escritura devuelve `error_code`: 0 correcto, 1 velocidad/eRPM
fuera de rango, 2 dirección fuera de rango, 3 fallo VESC y 4 fallo de dirección.
Para conducir se debe usar un topic a 10 Hz o más. Si el Agent se desconecta,
el firmware manda velocidad cero y centra la dirección.

La prueba Xbox Series desde la Jetson, su configuración segura y el inventario
necesario para adaptar ROSOrin están en `JETSON_ROSORIN_INTEGRATION.md`.
