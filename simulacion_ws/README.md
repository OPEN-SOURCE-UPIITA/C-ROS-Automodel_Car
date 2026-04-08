# 🚗 Simulación de Carro en ROS 2 y Gazebo

Este paquete contiene la simulación de un vehículo diferencial/Ackermann en Gazebo, controlado a través de ROS 2. Incluye un puente de comunicación automático y un adaptador (`sim_adapter`) que traduce los comandos personalizados de hardware a mensajes de velocidad estándar (`Twist`).

## 📋 Requisitos Previos

Para ejecutar este proyecto, necesitas tener instalado lo siguiente en tu sistema (probado en ROS 2 Jazzy):

* **ROS 2** (jazzy)
* **Gazebo** (Harmonic)
* **Paquetes de integración ROS-Gazebo:**
  sudo apt install ros-jazzy-ros-gz

* **Paquete de dependencias de control y estado:**
  sudo apt install ros-jazzy-robot-state-publisher

* **El paquete personalizado de mensajes:** `motor_msgs` (Asegúrate de tener este paquete compilado en tu workspace, ya que contiene la definición de `MotorCommand`).

## Instalación y Compilación

1. Crea un workspace de ROS 2 (si no tienes uno):
   mkdir -p ~/simulacion_ws/src
   cd ~/simulacion_ws/src

2. Clona o pega la carpeta de este paquete (`carro_simulacion`) y el paquete de mensajes (`motor_msgs`) dentro de la carpeta `src`.

3. Compila el workspace:
   cd ~/simulacion_ws
   colcon build

4. Actualiza las variables de entorno:
   source install/setup.bash

## Cómo Ejecutar la Simulación

Gracias al archivo de *launch* integrado, toda la simulación (Gazebo, el robot, los puentes de comunicación y el traductor de comandos) se inicia con un solo comando.

Abre una terminal y ejecuta:

ros2 launch carro_simulacion car_sim.launch.py

Al ejecutarlo verás que se abre Gazebo con la pista (`pista.world`) y tu modelo del carro cargado en el origen.

## Cómo Controlar el Carro

El sistema espera recibir comandos a través del topic `/motor_command` (usando el mensaje personalizado `motor_msgs/MotorCommand`). El nodo `sim_adapter` se encarga de convertir esos datos y enviarlos a Gazebo de forma transparente.

Para probar el movimiento manual sin un control físico, abre una **nueva terminal**, haz el source (`source install/setup.bash`) y publica un mensaje de prueba para avanzar y girar:

ros2 topic pub /motor_command motor_msgs/msg/MotorCommand "{dir_dc: 1, speed_dc: 50, dir_servo: 1740}"

**Parámetros de control:**
* `dir_dc`: Dirección del motor (1 = Adelante, 2 = Atrás, 0 = Stop)
* `speed_dc`: Velocidad lineal en porcentaje (0 a 100)
* `dir_servo`: Dirección del servo en PWM (Centro = 1500, Rango = 1110 a 1740)

## 📁 Estructura del Paquete

* `launch/`: Archivos de lanzamiento que orquestan Gazebo y los nodos de ROS 2.
* `urdf/`: El modelo 3D del carro (`carro.urdf`).
* `meshes/`: Archivos STL para la visualización del carro.
* `worlds/`: Entornos de simulación (ej. `pista.world`).
* `carro_simulacion/sim_adapter.py`: Nodo de Python que mapea las señales del control a velocidades en el simulador.