#!/usr/bin/env python3
"""
Launch file for YOLO traffic detector.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for YOLO detector."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('yolo_traffic_detector')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'yolo_config.yaml')
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    use_cuda_arg = DeclareLaunchArgument(
        'use_cuda',
        default_value='true',
        description='Whether to use CUDA for inference'
    )
    
    # YOLO detector node
    yolo_detector_node = Node(
        package='yolo_traffic_detector',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Add any topic remappings if needed
        ]
    )
    
    return LaunchDescription([
        log_level_arg,
        use_cuda_arg,
        yolo_detector_node
    ])