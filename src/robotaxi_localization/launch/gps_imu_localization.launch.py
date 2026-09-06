# ~/robotaxi_ws/src/robotaxi_localization/launch/gps_imu_localization.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Launch setup function to handle dynamic configuration loading"""
    
    # Get package directory
    pkg_robotaxi_localization = get_package_share_directory('robotaxi_localization')
    
    # Config file path
    config_file = os.path.join(
        pkg_robotaxi_localization,
        'config',
        'sensor_config.yaml'
    )
    
    # Verify config file exists
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    # GPS-IMU Localizer node with config file
    gps_imu_localizer = Node(
        package='robotaxi_localization',
        executable='gps_imu_localizer',
        name='gps_imu_localizer',
        output='screen',
        parameters=[
            config_file,  # Load from YAML config
            {
                # Override with launch arguments if provided
                'utm_zone': LaunchConfiguration('utm_zone'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'tf_broadcast': LaunchConfiguration('tf_broadcast'),
                'base_frame': LaunchConfiguration('base_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
            }
        ],
        remappings=[
            # Dynamic remapping based on config
            ('/gnss', LaunchConfiguration('gps_topic')),
            ('/imu/data', LaunchConfiguration('imu_topic'))
        ]
    )
    
    # Sensor Quality Monitor node
    sensor_monitor = Node(
        package='robotaxi_localization',
        executable='sensor_monitor',
        name='sensor_quality_monitor',
        output='screen',
        parameters=[config_file],  # Use same config file
        condition=IfCondition(LaunchConfiguration('enable_monitor'))
    )
    
    # Static transforms for sensor positions
    # GPS transform (from robotaxi spec: +1440mm X, 0 Y, +1390mm Z)
    gps_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_static_transform',
        arguments=[
            '1.44', '0', '1.39',      # Translation (m)
            '0', '0', '0',            # Rotation (rad)
            LaunchConfiguration('base_frame'), 
            'gps_link'
        ]
    )
    
    # IMU transform (same position as GPS for XSENS MTI-680)
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_transform',
        arguments=[
            '1.44', '0', '1.39',      # Translation (m) 
            '0', '0', '0',            # Rotation (rad)
            LaunchConfiguration('base_frame'),
            'imu_link'
        ]
    )
    
    # Camera static transform (from robotaxi spec: -205mm X, 0 Y, +685mm Z)
    # ZED2 center position
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform',
        arguments=[
            '-0.205', '0', '0.685',   # Translation (m)
            '0', '0', '0',            # Rotation (rad)
            LaunchConfiguration('base_frame'),
            'camera_link'
        ]
    )
    
    # LIDAR static transform (from robotaxi spec: -177mm X, 0 Y, +620mm Z)
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform',
        arguments=[
            '-0.177', '0', '0.62',    # Translation (m)
            '0', '0', '0',            # Rotation (rad)
            LaunchConfiguration('base_frame'),
            'lidar_link'
        ]
    )
    
    return [
        gps_imu_localizer,
        sensor_monitor,
        gps_static_tf,
        imu_static_tf,
        camera_static_tf,
        lidar_static_tf,
    ]


def generate_launch_description():
    """Generate the launch description with all arguments and nodes"""
    
    # Declare launch arguments
    declared_arguments = [
        # Core parameters
        DeclareLaunchArgument(
            'utm_zone',
            default_value='35',
            description='UTM zone for coordinate conversion (Turkey: 35-37)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Publishing rate in Hz for pose and odometry'
        ),
        DeclareLaunchArgument(
            'tf_broadcast',
            default_value='true',
            description='Enable TF broadcasting'
        ),
        
        # Frame names
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame of the vehicle'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global map frame'
        ),
        
        # Topic remapping
        DeclareLaunchArgument(
            'gps_topic',
            default_value='/gnss',
            description='GPS topic name'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU topic name'
        ),
        
        # Feature toggles
        DeclareLaunchArgument(
            'enable_monitor',
            default_value='true',
            description='Enable sensor quality monitoring'
        ),
        
        # Environment selection
        DeclareLaunchArgument(
            'environment',
            default_value='production',
            choices=['production', 'test', 'simulation'],
            description='Runtime environment selection'
        ),
    ]
    
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])