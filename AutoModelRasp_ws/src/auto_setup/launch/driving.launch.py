#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Procesador de imágenes (genera máscaras)
        ExecuteProcess(
            cmd=['ros2', 'run', 'driving', 'proc_image'],
            output='screen'
        ),
        
         # 2. Detector de carriles multicarril
        ExecuteProcess(
            cmd=['ros2', 'run', 'driving', 'detector_carril_multi'],
            output='screen'
        ),
        # 2. seguidor de carril.
        ExecuteProcess(
            cmd=['ros2', 'run', 'driving', 'detector_carril'],
            output='screen'
        ),
        
        # 3. Detector de stops
        ExecuteProcess(
            cmd=['ros2', 'run', 'driving', 'detector_senales'],
            output='screen'
        ),
        
        # 4. Detector de cruces peatonales
        ExecuteProcess(
            cmd=['ros2', 'run', 'driving', 'detector_cruces'],
            output='screen'
        ),
        
        # 5. Radar de franjas (LiDAR)
        ExecuteProcess(
            cmd=['ros2', 'run', 'driving', 'radar_franjas'],
            output='screen'
        ),
        
    ])
