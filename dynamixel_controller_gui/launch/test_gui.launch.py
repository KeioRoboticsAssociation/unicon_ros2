#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch file to start both mock controller and GUI for testing"""
    
    return LaunchDescription([
        # Start mock dynamixel controller
        ExecuteProcess(
            cmd=['python3', 'src/dynamixel_controller_gui/test_gui.py'],
            cwd='/home/imanoob/ros2_jazzy',
            name='mock_controller',
            output='screen'
        ),
        
        # Start GUI after a short delay
        Node(
            package='dynamixel_controller_gui',
            executable='dynamixel_gui',
            name='dynamixel_gui',
            output='screen'
        )
    ])