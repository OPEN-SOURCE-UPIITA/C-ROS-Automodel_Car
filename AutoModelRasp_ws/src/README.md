# 📂 Código Fuente ROS

Este directorio contiene todos los paquetes de ROS 2 para el **AutoModelCar**. Para mantener el sistema estable, el código se divide en "Nodos Core" (Hardware) y "Paquetes de Comportamiento" (Lógica).

## 🛡️ Nodos Core (Hardware - INTOCABLES)
Estos paquetes son la base del vehículo. Se comunican directamente con los sensores y el STM32. **No deben ser modificados sin autorización:**
* `comSTM`: Comunicación serial y control de motores.
* `camera`: Driver y publicación de imágenes RGB.
* `lidar`: Driver y publicación de la nube de puntos.
* `motor_msg`: Estructura de los mensajes para los motores.

## 🧠 Paquetes de Comportamiento (Desarrollo Activo)
Aquí es donde el equipo implementará la autonomía del auto, dividida por funcionalidades:
* `manual/`: Control por teclado, control y rutinas de prueba básicas.
* `driving/`: Seguimiento de carriles, detección de señales, evasión de obstáculos estáticos y móviles.
* `parking/`: Mapeo espacial y rutinas de estacionamiento.


## 💬 Mensajes y Comunicaciones (Interfaces) - En proceso
Para mantener el workspace limpio, aplicamos estas dos reglas estrictas sobre los mensajes:
1. **Priorizar Mensajes Estándar:** Para sensores genéricos, usa siempre los mensajes de ROS 2 (ej. `sensor_msgs/Image` para cámara, `sensor_msgs/LaserScan` para Lidar, `geometry_msgs/Twist` para movimiento).
2. **Mensajes Personalizados:** Cualquier mensaje (`.msg`), servicio (`.srv`) o acción (`.action`) específico del equipo debe crearse EXCLUSIVAMENTE dentro del paquete aglutinador **`automodel_interfaces`**. (Evita crear paquetes como `motor_msgs` o `camara_msgs`).

> **💡 Regla de Nomenclatura:** Como estándar del equipo, el código (variables, nombres de nodos, tópicos y mensajes) debe escribirse en **inglés** para mantener compatibilidad universal, mientras que la documentación y comentarios pueden ir en **español**.

