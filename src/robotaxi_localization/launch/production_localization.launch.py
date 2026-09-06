# ~/robotaxi_ws/src/robotaxi_localization/launch/production_localization.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Package directory
    pkg_dir = get_package_share_directory('robotaxi_localization')
    config_file = os.path.join(pkg_dir, 'config', 'sensor_config.yaml')
    
    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )

    # GPS-IMU Localizer with config file
    gps_imu_localizer = Node(
        package='robotaxi_localization',
        executable='gps_imu_localizer', 
        name='gps_imu_localizer',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # Sensor Quality Monitor
    sensor_monitor = Node(
        package='robotaxi_localization',
        executable='sensor_monitor',
        name='sensor_monitor', 
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # Static transforms based on vehicle specifications
    sensor_transforms = [
        # GPS/IMU: +1440mm X, 0 Y, +1390mm Z from front axle
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_transform', 
            arguments=['1.44', '0', '1.39', '0', '0', '0', 'base_link', 'gps_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_transform',
            arguments=['1.44', '0', '1.39', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # LIDAR: -177mm X, 0 Y, +620mm Z
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='lidar_static_transform',
            arguments=['-0.177', '0', '0.62', '0', '0', '0', 'base_link', 'velodyne']
        ),
        
        # ZED2 Camera: -205mm X, 0 Y, +685mm Z  
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', 
            name='zed2_static_transform',
            arguments=['-0.205', '0', '0.685', '0', '0', '0', 'base_link', 'zed2_left_camera_frame']
        )
    ]
    
    return LaunchDescription([
        config_arg,
        gps_imu_localizer,
        sensor_monitor
    ] + sensor_transforms)
