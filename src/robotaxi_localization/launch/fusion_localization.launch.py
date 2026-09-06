#!/usr/bin/env python3
"""
robot_localization çift-UKF füzyon lokalizasyonu.

  Teker odometrisi (FB_VehicleSpeed + FeedbackSteeringAngle)
  + ZED2 IMU  + GPS (/gnss)
        │
        ├─ ukf_local        (odom->base_link)  ->  /odometry/filtered   (sürekli)
        ├─ navsat_transform  (GPS -> map)       ->  /odometry/gps
        └─ ukf_global        (map->odom)        ->  /odometry/filtered_map (mutlak)

/odometry/filtered kontrol node'una /odom olarak remap edilir (lane_following).
Statik TF'ler Mimari doküman montaj offsetlerinden (production_localization ile aynı).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('robotaxi_localization')
    ukf_cfg = os.path.join(pkg, 'config', 'ukf_fusion.yaml')
    sensor_cfg = os.path.join(pkg, 'config', 'sensor_config.yaml')

    gps_topic = LaunchConfiguration('gps_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    src = LaunchConfiguration('localization_source')

    use_xsens = IfCondition(PythonExpression(["'", src, "' == 'xsens'"]))
    use_ukf = IfCondition(PythonExpression(["'", src, "' == 'ukf'"]))

    return LaunchDescription([
        DeclareLaunchArgument('gps_topic', default_value='/gnss'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),

        # ===================================================================
        #  localization_source: 'xsens' (VARSAYILAN) | 'ukf'
        # ===================================================================
        # 2026-07-28 saha verisi karari. 'xsens' = tek dugum, Xsens MTi'nin
        # kendi GNSS/INS cozumunu dogrudan /odometry/* olarak yayinlar.
        # 'ukf' = eski cift-UKF zinciri (wheel_odometry + ZED IMU +
        # navsat_transform). Gerekce ve olcumler: xsens_odometry_node.py
        # ust yorumu. Ozetle Xsens dururken 0.01-0.03 m sacilma ve 0.03-0.15
        # derece yaw kararliligi veriyor; UKF katmani bunun uzerine fayda
        # eklemiyor ama uc dogrulanmamis bagimlilik getiriyor.
        #
        # UKF'e donmek icin:
        #   ros2 launch parking_safety park_bringup.launch.py \
        #       start_localization:=true localization_source:=ukf
        DeclareLaunchArgument('localization_source', default_value='xsens'),

        # ---------------- SECENEK A: Xsens dogrudan (varsayilan) ------------
        Node(
            package='robotaxi_localization',
            executable='xsens_odometry_node',
            name='xsens_odometry_node',
            output='screen',
            condition=use_xsens,
        ),

        # ---------------- SECENEK B: cift-UKF zinciri -----------------------
        # -------- Teker + direksiyon odometrisi --------
        Node(
            package='robotaxi_localization',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            condition=use_ukf,
        ),

        # -------- ukf_local: odom -> base_link (sürekli) --------
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_local',
            output='screen',
            parameters=[ukf_cfg],
            condition=use_ukf,
            # çıktı: /odometry/filtered (varsayılan)
        ),

        # -------- navsat_transform: GPS -> map odometry --------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ukf_cfg],
            condition=use_ukf,
            remappings=[
                ('gps/fix', gps_topic),
                ('imu', imu_topic),
                ('odometry/filtered', '/odometry/filtered'),   # ukf_local çıktısı
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
        ),

        # -------- ukf_global: map -> odom (mutlak) --------
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_global',
            output='screen',
            parameters=[ukf_cfg],
            condition=use_ukf,
            remappings=[
                ('odometry/filtered', '/odometry/filtered_map'),  # çıktıyı ayır
            ],
        ),

        # -------- Sensör sağlık izleme (mevcut node) --------
        Node(
            package='robotaxi_localization',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen',
            parameters=[sensor_cfg],
        ),

        # -------- Statik TF'ler (Mimari doküman offsetleri) --------
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='gps_static_tf',
             arguments=['1.44', '0', '1.39', '0', '0', '0', 'base_link', 'gps_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='velodyne_static_tf',
             # z=1.366: 2026-07-28 LiDAR bag'lerinden zemin duzlemi fit ile OLCULDU
             # (iki bagimsiz kayit 1 mm icinde ayni). ESKI 0.62 -> 0.75 m hatali.
             arguments=['-0.177', '0', '1.366', '0', '0', '0', 'base_link', 'velodyne']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='zed2_static_tf',
             arguments=['-0.205', '0', '0.685', '0', '0', '0', 'base_link', 'zed2_left_camera_frame']),
        # ZED IMU frame — ZED imu mesajının frame_id'sine göre AD DOĞRULA (varsayılan zed_imu_link)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='zed_imu_static_tf',
             arguments=['-0.205', '0', '0.685', '0', '0', '0', 'base_link', 'zed_imu_link']),
        # Sahada IMU mesajlarının frame_id'si "imu_link" geldi; navsat_transform
        # base_link -> imu_link arar ve bulamayınca sürekli hata basıyordu.
        # (Doğrula: ros2 topic echo <imu_topic> --once | grep frame_id)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='imu_link_static_tf',
             arguments=['-0.205', '0', '0.685', '0', '0', '0', 'base_link', 'imu_link']),
    ])
