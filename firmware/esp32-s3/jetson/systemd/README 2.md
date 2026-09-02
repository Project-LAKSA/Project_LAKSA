# Servicios base de LAKSA en la Jetson

Estos servicios se ejecutan desde el arranque, independientemente del modo de
conducción:

- `laksa-xbox-reconnect.service` mantiene conectado el control Xbox previamente
  emparejado y confiable. Si el control está apagado, sigue reintentando sin
  bloquear otros servicios.
- `laksa-micro-ros-agent.service` espera el enlace estable
  `/dev/laksa_microros` y ejecuta el Agent oficial Humble. Se recupera al
  desconectar y volver a conectar el ESP32.
- `laksa-lidar-mapping.service` espera `/dev/laksa_lidar`, inicia el RPLIDAR
  A2M12 a 256000 baud, mantiene activo su flujo `/scan` y levanta SLAM Toolbox
  en modo mapping.
- `laksa-control-navigation.service` inicia una sola autoridad de manejo. Arranca
  en manual, publica odometría Ackermann medida por VESC y mantiene Nav2 y el
  explorador de fronteras inactivos hasta sostener A durante tres segundos. X
  cancela navegación y vuelve a manual; B impone velocidad cero.

No se inicia `xbox_drive_node` automáticamente. Publicarlo junto con navegación
autónoma sin arbitraje produciría dos fuentes de comandos para los mismos
actuadores.

## Instalación

```bash
sudo install -D -m 0755 jetson/scripts/laksa-xbox-reconnect /usr/local/lib/laksa/laksa-xbox-reconnect
sudo install -D -m 0755 jetson/scripts/laksa-micro-ros-agent /usr/local/lib/laksa/laksa-micro-ros-agent
sudo install -D -m 0755 jetson/scripts/laksa-lidar-mapping /usr/local/lib/laksa/laksa-lidar-mapping
sudo install -D -m 0755 jetson/scripts/laksa-control-navigation /usr/local/lib/laksa/laksa-control-navigation
sudo install -m 0644 jetson/systemd/laksa-xbox-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-micro-ros-agent.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-lidar-mapping.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-control-navigation.service /etc/systemd/system/
sudo install -m 0644 jetson/udev/99-laksa-devices.rules /etc/udev/rules.d/
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl enable --now laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-lidar-mapping.service laksa-control-navigation.service
```

## Diagnóstico

```bash
systemctl status laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-lidar-mapping.service laksa-control-navigation.service
journalctl -u laksa-xbox-reconnect.service -u laksa-micro-ros-agent.service -u laksa-lidar-mapping.service -u laksa-control-navigation.service -f
```
