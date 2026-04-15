#!/usr/bin/env python3
# Archivo: auto_setup/launch/hardware_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        # 1. Cámara Astra HP60C
        Node(
            package='ascamera_hp60c',
            executable='camera_publisher',
            name='ascamera_hp60c',
            output='screen',
            parameters=[{
                'device_id': 0,
                'fps': 30,
                'resolution': '640x480'
            }]
        ),
        
        # 2. Comunicación con STM32 (UART)
        Node(
            package='com_stm',
            executable='com_stm',
            name='com_stm',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200
            }]
        ),
        
        # 3. Driver del motor MS200
        Node(
            package='ms200_driver',
            executable='ms200_node',
            name='ms200_driver',
            output='screen',
            parameters=[{
                'servo_center': 1500,
                'max_speed': 100
            }]
        ),
    ])
