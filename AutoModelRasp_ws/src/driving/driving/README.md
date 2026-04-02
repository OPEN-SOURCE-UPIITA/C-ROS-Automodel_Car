```markdown
# Guía de Configuración: Cámara Angstrong HP60C + Detector de Señal STOP

Este documento resume la configuración del driver de la cámara Angstrong HP60C y el detector de señales de STOP en ROS 2 Jazzy.

---

## 1. Estructura del Proyecto

```
pruebas_ws/
├── src/
│   ├── ascam_ros2/                    # Driver de la cámara
│   │   └── ascamera/
│   │       ├── launch/hp60c.launch.py
│   │       ├── configurationfiles/
│   │       └── scripts/
│   └── driving/                        # Detector de STOP
│       ├── scripts/
│       │   ├── proc_senial.py          # Nodo detector
│       │   └── params_gui.py           # GUI para parámetros
│       ├── launch/stop_sign.launch.py
│       ├── package.xml
│       └── setup.py
```

---

## 2. Driver de la Cámara (ascam_ros2)

### Clonar y compilar

```bash
cd ~/pruebas_ws/src
git clone https://github.com/j0z3ph/ascam_ros2.git
cd ~/pruebas_ws
colcon build --packages-select ascamera
source install/setup.bash
```

### Reglas udev (permisos USB)

```bash
cd ~/pruebas_ws/src/ascam_ros2/ascamera/scripts
sudo bash create_udev_rules.sh
# Desconectar y conectar la cámara
```

### Configurar ruta del JSON

Editar `launch/hp60c.launch.py` y actualizar:

```python
config_path = "/home/usuario/pruebas_ws/src/ascam_ros2/ascamera/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json"
```

### Variables de entorno (para comunicación en red)

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=30
```

---

## 3. Detector de Señal STOP (driving)

### Estructura del nodo detector (`proc_senial.py`)

- **Suscripciones:**
  - `/ascamera_hp60c/camera_publisher/rgb0/image` → imagen RGB
  - `/ascamera_hp60c/camera_publisher/depth0/image_raw` → profundidad
  - `/stop_sign/params` → parámetros HSV

- **Publicación:**
  - `/vision/stop_debug` → imagen con detección dibujada

- **Algoritmo:**
  1. Filtro de color rojo (HSV con dos rangos)
  2. Detección de contornos
  3. Verificación de forma octagonal (7-9 vértices)
  4. Cálculo de distancia usando mapa de profundidad

### GUI de parámetros (`params_gui.py`)

- Interfaz PySide6 con sliders para ajustar rangos HSV en tiempo real
- Publica en `/stop_sign/params`

### Configuración del paquete `driving`

**setup.py** (método scripts):

```python
scripts=glob('scripts/*.py'),
```

**stop_sign.launch.py**:

```python
Node(package='driving', executable='proc_senial.py', output='screen'),
Node(package='driving', executable='params_gui.py', output='screen')
```

### Compilar y ejecutar

```bash
cd ~/pruebas_ws
colcon build --packages-select driving
source install/setup.bash
ros2 launch driving stop_sign.launch.py
```

---

## 4. Solución de Problemas Comunes

### Error: `Exec format error` en scripts
- Agregar `#!/usr/bin/env python3` al inicio del archivo
- Dar permisos: `chmod +x scripts/*.py`
- Recompilar

### Error: `executable not found` en launch
- Usar `scripts=` en setup.py, no `entry_points`
- Incluir extensión `.py` en el launch
- Verificar estructura: los scripts deben estar en `scripts/`

### Error: `get mjpeg from xu command failed`
- Verificar que el archivo JSON existe y la ruta es correcta
- Reducir FPS o resolución en el launch

### No se ve la imagen en RViz desde otra PC
- Configurar `ROS_LOCALHOST_ONLY=0`
- Mismo `ROS_DOMAIN_ID` en ambas máquinas
- Reducir FPS a 5-10 para mejorar streaming

---

## 5. Comandos Útiles

```bash
# Ver tópicos activos
ros2 topic list

# Ver imagen en tiempo real
ros2 run rqt_image_view rqt_image_view

# Ver imagen procesada por el detector
ros2 run rqt_image_view rqt_image_view /vision/stop_debug

# Publicar parámetros manualmente
ros2 topic pub /stop_sign/params std_msgs/msg/Int32MultiArray "data: [0,80,80,15,255,255,165,80,80,180,255,255]"

# Ver profundidad
ros2 topic echo /ascamera_hp60c/depth0/image_raw --once | head
```

---

## 6. Parámetros HSV Recomendados

| Parámetro | Valor por defecto | Rango |
|-----------|-------------------|-------|
| H min 1 | 0 | 0-180 |
| H max 1 | 10 | 0-180 |
| H min 2 | 170 | 0-180 |
| H max 2 | 180 | 0-180 |
| S min | 100 | 0-255 |
| V min | 100 | 0-255 |

---

## 7. Referencias

- Repositorio driver: [j0z3ph/ascam_ros2](https://github.com/j0z3ph/ascam_ros2)
- SDK Angstrong: v1.2.28.20241021
- Cámara modelo: HP60C
- ROS 2: Jazzy (Ubuntu 24.04)
```
