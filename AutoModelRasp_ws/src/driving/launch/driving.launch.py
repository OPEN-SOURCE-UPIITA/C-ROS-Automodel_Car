from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Detector de Señales (STOP, etc.)
        Node(
            package='driving',
            executable='detector_senales',
            name='signals_node',
            output='screen'
        ),

        # 2. Detector de Carril (Segmentación y cálculo de error)
        Node(
            package='driving',
            executable='detector_carril',
            name='lane_node',
            output='screen'
        ),

        # 3. Lane Detection 2 (El que tienes mapeado como 'lane_detection' en setup.py)
        Node(
            package='driving',
            executable='lane_detection',
            name='lane_vision_node',
            output='screen'
        ),

        # 4. Detector de Objetos (Radar de profundidad para rebase)
        Node(
            package='driving',
            executable='car_detection',
            name='object_radar_node',
            output='screen'
        ),

        # 5. Nodo Maestro de Control (Motores y Lógica de Rebase)
        Node(
            package='driving',
            executable='drive_carril',
            name='drive_node',
            output='screen',
            parameters=[{
                'habilitar_conduccion': False,
                'carril_derecho_activo': True,
                'fuerza_rebase_pct': 60,
                'velocidad_dc': 80
            }]
        ),
    ])
