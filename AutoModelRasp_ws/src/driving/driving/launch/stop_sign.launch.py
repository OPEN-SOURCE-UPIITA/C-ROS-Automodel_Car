import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='driving',
            executable='proc_senial.py',
            name='stop_sign_detector',
            output='screen'
        ),
        Node(
            package='driving',
            executable='params_gui.py',
            name='stop_sign_gui',
            output='screen'
        )
    ])
