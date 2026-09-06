# launch/bag_test_localization.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robotaxi_localization')
    config_file = os.path.join(pkg_dir, 'config', 'bag_test_config.yaml')
    
    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/akaln/bags',
        description='Path to bag files directory'
    )
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate', 
        default_value='1.0',
        description='Bag playback rate'
    )
    
    # GNSS bag player
    gnss_bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 
             [LaunchConfiguration('bag_path'), '/ukf_gnss_0.db3'],
             '--rate', LaunchConfiguration('playback_rate'),
             '--topics', '/gnss'],
        name='gnss_bag_player',
        output='screen'
    )
    
    # IMU bag player  
    imu_bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             [LaunchConfiguration('bag_path'), '/ukf_imu_0.db3'], 
             '--rate', LaunchConfiguration('playback_rate'),
             '--topics', '/imu/data'],
        name='imu_bag_player',
        output='screen'
    )
    
    # GPS-IMU Localizer
    localizer = Node(
        package='robotaxi_localization',
        executable='gps_imu_localizer',
        name='gps_imu_localizer',
        output='screen',
        parameters=[config_file]
    )
    
    # Sensor Monitor
    sensor_monitor = Node(
        package='robotaxi_localization', 
        executable='sensor_monitor',
        name='sensor_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    # Static transforms
    static_transforms = [
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
        )
    ]
    
    return LaunchDescription([
        bag_path_arg,
        playback_rate_arg,
        gnss_bag_player,
        imu_bag_player, 
        localizer,
        sensor_monitor
    ] + static_transforms)