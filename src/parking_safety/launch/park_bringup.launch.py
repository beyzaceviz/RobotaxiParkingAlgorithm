#!/usr/bin/env python3
"""Park Algoritmasi - Arac Uzerinde Tek Komutla Baslatma (Bringup).

Calistirir (bu SIRAYLA gerekli DEGIL, hepsi paralel baslar; her node kendi
girdisini bekler):
  * robotaxi_localization/fusion_localization.launch.py
        wheel_odometry_node + robot_localization cift-UKF
        -> /odometry/filtered (surekli/puruzsuz, odom->base_link)
        -> /odometry/filtered_map (GPS-mutlak, map->odom)
        Bu depodaki beemobs_odometry_node'un (basit tekerlek dead-reckoning)
        YERINE gecer (2026-07-24 karari) - ikisi birden CALISTIRILMAZ, cunku
        ikisi de /odom uretmeye calisirsa cakisir.
  * yolo_traffic_detector      : ZED2 RGB -> /yolo_detections (tabela tespiti)
        (2026-07-25: ayri YOLO imaji birlestirildi - planner'in tabela<->cep
         eslemesi bu topic olmadan calismaz. Kamera yoksa/bench testinde
         start_yolo:=false ile kapatilir; planner o zaman tabelasiz, salt
         LiDAR doluluguyle calisir.)
  * parking_gps_zone_trigger   : /gnss -> /parking_zone_reached (GPS geofence)
  * parking_mission_planner    : /velodyne_points + /yolo_detections -> /parking/slot_status
        (/odom remap -> /odometry/filtered)
  * parking_decision_and_control_node : GPS gate + LiDAR + odom + slot_status
        -> DOGRUDAN /beemobs/* (BeemobsActuator; ara /cmd_vel + kopru YOK)
        (/odom remap -> /odometry/filtered; require_gps_zone=True varsayilan:
         /parking_zone_reached=True gelene kadar arac STATE_BEKLE_GPS'te
         tamamen hareketsiz kalir. GPS'siz masabasi/bench testi icin:
         --ros-args -p parking_decision_and_control_node.require_gps_zone:=false)

NOT: Park kontrol dugumu araca DOGRUDAN yazar (BeemobsActuator, parking_safety/
beemobs_actuator.py). Eski cmd_vel_to_beemobs_bridge dugumu KALDIRILDI - ara
/cmd_vel + ayri kopru yerine dugum CAN'i kendisi surer. Ayni aktuator sinifini
durak paketi de kullanir (import). Gorev disinda aktuator susar; /beemobs'un
sahibi lane_following kalir.

SAHADA HIZLI KALIBRASYON (rebuild GEREKMEZ - launch argumani olarak verilir):
    ros2 launch parking_safety park_bringup.launch.py \
        steer_sign:=1.0 max_steer_deg:=30.0 \
        cam_focal_px:=235.3 cam_image_width:=672.0

  * steer_sign (varsayilan -1.0): ROS konvansiyonu angular.z POZITIF=SOLA,
    arac konvansiyonu ise POZITIF=SAGA (bkz. lane_following_controller.cpp
    satir 318/331). Bu yuzden isaret cevrilir. SAHADA ILK TEST: arac ters
    yone kiriyorsa bunu 1.0 yapin - EN KRITIK KALIBRASYON.
  * max_steer_deg (varsayilan 37.0): mekanik sinir 50° ama simulasyon
    37°'de dogrulandi; kademeli acin. Bkz. beemobs_actuator.py.
  * thr_max (varsayilan 120, lane_following ile ayni): gaz pedali TAVANI.
    Park hizlari cok dusuk (~1-2 km/h) ve FB_VehicleSpeed cozunurlugu
    1 km/h; arac hizi 0 raporlarken entegrator tavana tirmanip ANI
    FIRLAMA yapabilir. Ilk saha testlerinde 80-90 ile baslayin.
  * cam_focal_px / cam_image_width / cam_dx: gercek ZED2 coz./HFOV'una
    gore YOLO tabela-cep acisal eslestirmesi icin - SORUN #5, bkz.
    parking_mission_planner.py.

DIREKSIYON/GAZ artik aractaki steering_pid_node & speed_pid_node'a BAGLI
DEGIL (pid_launch calistirmaya GEREK YOK). BeemobsActuator geri beslemeyi
(/beemobs/FeedbackSteeringAngle, /beemobs/FB_VehicleSpeed) kendisi okuyup
dogrudan PWM/gaz basiyor - gerekcesi beemobs_actuator.py ust yorumunda.

Kullanim (SSH ile araca baglandiktan, docker container icinden):
    ros2 launch parking_safety park_bringup.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    localization_launch = os.path.join(
        get_package_share_directory('robotaxi_localization'),
        'launch', 'fusion_localization.launch.py')
    yolo_launch = os.path.join(
        get_package_share_directory('yolo_traffic_detector'),
        'launch', 'yolo_detector.launch.py')

    start_localization = LaunchConfiguration('start_localization')
    start_yolo = LaunchConfiguration('start_yolo')
    steer_sign = LaunchConfiguration('steer_sign')
    max_steer_deg = LaunchConfiguration('max_steer_deg')
    thr_max = LaunchConfiguration('thr_max')
    cam_focal_px = LaunchConfiguration('cam_focal_px')
    cam_image_width = LaunchConfiguration('cam_image_width')
    cam_dx = LaunchConfiguration('cam_dx')

    return LaunchDescription([
        # Lokalizasyon: full_autonomy'den dahil edilince false (cift-UKF onleme;
        # full_autonomy lokalizasyonu kendisi baslatir). Tek basina kullanimda true.
        DeclareLaunchArgument('start_localization', default_value='false'),
        # YOLO: full_autonomy'den dahil edilince false (orada zaten baslatilir;
        # ikinci bir yolo_detector_node GPU'yu bosuna ikiye boler ve ayni
        # /yolo_detections'a iki kaynak yayin yapar). Park tek basina
        # kullanilirken true olmali - planner tabelayi bu topic'ten okur.
        DeclareLaunchArgument('start_yolo', default_value='true'),
        # Direksiyon kalibrasyonu (dogrudan PWM kapali cevrim)
        # 2026-07-28: her uc deger de saha bag'lerinden OLCULDU, artik tahmin
        # degil (gerekce + yontem: beemobs_actuator.py ust yorumu).
        #   steer_sign  -1.0 : PWM 70 -> fb azalir -> yaw_rate + -> arac SOLA.
        #                      ROS'ta angular.z + = SOL oldugundan isaret ters.
        #   max_steer   20.0 : mekanik kilitler fb -37/+33 = +21.26/-20.20 deg
        #                      (2026-07-28 REVIZE kalibrasyon, 26502 ornek);
        #                      20.0 iki yonde de ULASILABILIR simetrik sinir.
        #                      ESKI 22.0 yeni kalibrasyonla ULASILAMIYOR.
        #   thr_max       90 : sahada en fazla 84 kullanildi (1.25 m/s).
        DeclareLaunchArgument('steer_sign', default_value='-1.0'),
        DeclareLaunchArgument('max_steer_deg', default_value='20.0'),
        DeclareLaunchArgument('thr_max', default_value='90'),
        # SORUN #5 kalibrasyonu (YOLO kamera modeli - gercek ZED2'ye gore)
        # 2026-07-25 DUZELTME (birlestirmenin ortaya cikardigi tutarsizlik):
        #   Eski varsayilanlar 672.2 px / 1920 px idi = GAZEBO degeri.
        #   Ayni imajdaki yolo_traffic_detector/config/yolo_config.yaml ise
        #   gercek kamerayi 1280x720, focal 528.28 px olarak tanimliyor ve
        #   bbox'lari O cerceve icinde yayinliyor. Planner 1920 px genislik
        #   varsayarsa ayni bbox'un acisini ~%50 hatali hesaplar -> tabela
        #   yanlis cebe eslesir. Iki taraf ayni degerlere cekildi.
        #   NOT: yolo_detector_node fx'i CameraInfo'dan okuyabiliyor; aractaki
        #   gercek grab_resolution (can_launch.xml) VGA ise ikisi de yeniden
        #   hesaplanmali. SAHA TESTINDE `ros2 topic echo /zed/.../camera_info`
        #   ile DOGRULAYIN - rebuild gerekmez, launch argumani olarak verilir.
        DeclareLaunchArgument('cam_focal_px', default_value='528.2849731445312'),
        DeclareLaunchArgument('cam_image_width', default_value='1280.0'),
        DeclareLaunchArgument('cam_dx', default_value='-0.205'),
        # YOLO tabela izni sarti. Kamera/YOLO yokken TRUE kalirsa hicbir cep
        # 'kullanilabilir' olmaz ve arac park etmez (bkz. slot_available).
        # Tabelasiz deneme icin: require_sign_permit:=false
        DeclareLaunchArgument('require_sign_permit', default_value='true'),
        # REFERANS KAYDIRMA: cep haritasini KATI tutup toptan oteler.
        # GPS mutlak konumu ~0.4 m (1 sigma) belirsiz ama haritanin SEKLI
        # dogru. Saha basinda araci cep1'e park edip olculen farki buraya
        # verin - 9 cep birden kayar, rebuild GEREKMEZ.
        DeclareLaunchArgument('slot_offset_x', default_value='0.0'),
        DeclareLaunchArgument('slot_offset_y', default_value='0.0'),
        # NOT (2026-08-02): 'target_slot_id' (Sabit Cep Secimi) KALDIRILDI.
        # Cep secimi artik tamamen YOLO tabela izni + LiDAR doluluk kararina
        # birakildi; o mod 'available' kapisini butunuyle atladigi icin dolu
        # veya 'park_yasak' tabelali cebe de girebiliyordu. Izni okunamayan
        # cepler icin kontrollu son care yolu var (parking_mission_planner.
        # slot_available_fallback). target_slot_id:=N verilirse SESSIZCE yok
        # sayilir - ros2 launch bilinmeyen argumanlari gormezden gelir.

        # GPS + IMU + teker cift-UKF lokalizasyonu -> /odometry/filtered
        # SADECE start_localization:=true iken (tek basina kullanim).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(start_localization)),

        # YOLO tabela tespiti -> /yolo_detections (planner'in girdisi)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch),
            condition=IfCondition(start_yolo)),

        Node(
            package='parking_safety',
            executable='parking_gps_zone_trigger',
            name='parking_gps_zone_trigger',
            output='screen',
        ),
        Node(
            package='parking_safety',
            executable='parking_mission_planner',
            name='parking_mission_planner',
            output='screen',
            remappings=[('/odom', '/odometry/filtered')],
            parameters=[{
                'cam_focal_px': cam_focal_px,
                'cam_image_width': cam_image_width,
                'cam_dx': cam_dx,
                'require_sign_permit': LaunchConfiguration('require_sign_permit'),
                'slot_offset_x': LaunchConfiguration('slot_offset_x'),
                'slot_offset_y': LaunchConfiguration('slot_offset_y'),
            }],
        ),
        # Park kontrol dugumu ARTIK araca DOGRUDAN yazar (BeemobsActuator).
        # Eski cmd_vel_to_beemobs_bridge KALDIRILDI; onun kalibrasyon paramlari
        # (steer_sign, max_steer_deg, thr_max) buraya tasindi.
        Node(
            package='parking_safety',
            executable='parking_decision_and_control_node',
            name='parking_decision_and_control_node',
            output='screen',
            remappings=[('/odom', '/odometry/filtered')],
            parameters=[{
                'steer_sign': steer_sign,
                'max_steer_deg': max_steer_deg,
                'thr_max': thr_max,
            }],
        ),
    ])
