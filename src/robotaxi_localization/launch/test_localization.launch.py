# ~/robotaxi_ws/src/robotaxi_localization/launch/test_localization.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Launch arguments
    utm_zone_arg = DeclareLaunchArgument(
        'utm_zone',
        default_value='35',
        description='UTM zone for coordinate conversion'
    )
    
    # Test data publisher
    test_publisher = Node(
        package='robotaxi_localization',
        executable='test_data_publisher',
        name='test_data_publisher',
        output='screen'
    )
    
    # GPS-IMU Localizer
    gps_imu_localizer = Node(
        package='robotaxi_localization', 
        executable='gps_imu_localizer',
        name='gps_imu_localizer',
        output='screen',
        parameters=[{
            'utm_zone': LaunchConfiguration('utm_zone'),
            'publish_rate': 50.0,
            'tf_broadcast': True,
            'base_frame': 'base_link',
            'map_frame': 'map'
        }]
    )
    
    # Static transforms
    gps_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_static_transform',
        arguments=['1.44', '0', '1.39', '0', '0', '0', 'base_link', 'gps_link']
    )
    
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='imu_static_transform',
        arguments=['1.44', '0', '1.39', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    return LaunchDescription([
        utm_zone_arg,
        test_publisher,
        gps_imu_localizer,
        gps_static_tf,
        imu_static_tf
    ])