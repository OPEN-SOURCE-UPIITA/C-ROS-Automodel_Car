#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Incluir hardware launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('auto_setup'),
                    'launch',
                    'hardware.launch.py'
                ])
            ])
        ),
        
        # Incluir driving launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('auto_setup'),
                    'launch',
                    'driving.launch.py'
                ])
            ])
        ),
    ])
