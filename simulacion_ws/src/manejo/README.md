# manejo

Paquete ROS 2 Jazzy para controlar el carro de Gazebo desde una terminal.
Publica `motor_msgs/msg/MotorCommand` en `/motor_command`, que recibe el nodo
`sim_adapter` de `carro_simulacion`.

## Compilar

Desde `simulacion_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motor_msgs carro_simulacion manejo
source install/setup.bash
```

## Uso

Inicia la simulacion en una terminal:

```bash
ros2 launch carro_simulacion car_sim.launch.py
```

En otra terminal, inicia el control con velocidad inicial del 50%:

```bash
ros2 run manejo teleop_wasd
```

Controles: `W` avanza, `S` retrocede, `A` gira a la izquierda, `D` gira a la
derecha, `+`/`-` ajusta la velocidad en pasos de 5% y `espacio` detiene y centra
la direccion. `Ctrl+C` cierra el nodo. Si no se reciben teclas durante 0.5 s,
el carro se detiene y la direccion vuelve al centro.

La velocidad inicial se puede cambiar al ejecutar el nodo, por ejemplo:

```bash
ros2 run manejo teleop_wasd --ros-args -p speed:=30
```