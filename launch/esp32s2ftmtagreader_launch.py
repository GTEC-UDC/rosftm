#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    serial_arg = DeclareLaunchArgument(
        'serial',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32S2 FTM Tag Reader'
    )
    
    # Create node
    esp32s2_ftm_tag_reader_node = Node(
        package='gtec_ftm',
        executable='ESP32S2FTMTagReader',
        name='ESP32S2FTMTagReader',
        output='screen',
        parameters=[{
            'serial': LaunchConfiguration('serial')
        }]
    )
    
    return LaunchDescription([
        serial_arg,
        esp32s2_ftm_tag_reader_node
    ]) 