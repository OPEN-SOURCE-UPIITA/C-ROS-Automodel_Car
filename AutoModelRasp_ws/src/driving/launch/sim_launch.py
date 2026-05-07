from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    paquete = 'driving'

    return LaunchDescription([
        
        # 1. Detector de Carril (Cámara)
        Node(
            package=paquete,
            executable='detector_carril',
            name='detector_carril_node',
            output='screen'
        ),

        # 2. Detector de Cruce Peatonal (Cámara + FFT)
        Node(
            package=paquete,
            executable='detector_cruces',
            name='detector_cruce_node',
            output='screen'
        ),

        # 3. Cerebro de Conducción (Control del chasis)
        Node(
            package=paquete,
            executable='drive_carril',
            name='drive_carril_node',
            output='screen'
        )
    ])