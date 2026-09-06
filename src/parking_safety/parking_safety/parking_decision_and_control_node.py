#!/usr/bin/env python3
"""Park Karar ve Kontrol Node'u.

Mimari:
  - /velodyne_points (PointCloud2)  -> ROI icinde engel kontrolu (guvenlik)
  - /odom (nav_msgs/Odometry)       -> anlik pozisyon (X, Y) ve yonelim (psi)
  - /parking/slot_status (std_msgs/String, JSON)
        parking_mission_planner'dan gelen cep durum matrisi; hedef cep
        buradan DINAMIK secilir (available == True olan en yakin cep)
  - /cmd_vel (geometry_msgs/Twist)  -> hiz + direksiyon komutu
  - /parking_state (std_msgs/String) -> anlik durum yayini (diger node'lar icin)
  - /perception_active (std_msgs/Bool) -> Algi Kilidi sinyali (Stop-and-Stare):
        SADECE dur-ve-tara fazlarinda (ARAYIS_BASLANGIC / ARAYIS_DURAK)
        True; arac hareket halindeyken False. parking_mission_planner bu
        sinyal False iken YOLO/LiDAR guncellemelerini islemez (hareket
        bulanikligi + odometri gecikmesi + perspektif kaymasi kararsiz
        eslesme uretir), sadece son stabil matrisi yayinlamaya devam eder.

Ayarlanabilir parametreler (ornek: --ros-args -p kp:=0.5):
  kp, approach_speed, goal_tolerance,
  lead_x_offset, lead_y_offset, mouth_x_offset
Oncelik zinciri korunur: komut satiri (-p) > cep profili > varsayilan.
Profil artik calisma aninda (cep secilince) yuklendigi icin, CLI'da
acikca degistirilen parametreler kaydedilir ve profil yuklendikten
sonra yeniden uygulanir.

Dur-ve-Tara Kesif Sistemi (Stop-and-Scan) ve Fail-Safe:
  ARAYIS_BASLANGIC: Arac hareketsiz bekler, /parking/slot_status'tan
    uygun cep taranir. Hedef bulunursa -> YONELME. scan_timeout (10 s)
    icinde bulunamazsa ikinci gorus noktasina kesif surusu baslar.
  KESIF_SURUS: Pure pursuit ile EXPLORATION_STOPS listesindeki siradaki
    duraga surulur (bir cebin park rotasi DEGIL, occlusion golgesindeki
    tabelalari gorebilecek konumlar). Iki bacakli: once duragin yaw
    yonunun gerisindeki hizalama noktasi, sonra durak - arac duraga
    istenen YAW ile hizalanmis varir (Ackermann yerinde donemez).
  ARAYIS_DURAK (jenerik, indeks sayacli): Durakta hareketsiz tarama.
    Hedef bulunursa rota ANLIK konumdan kurulur -> YONELME. Ara durakta
    corridor_scan_timeout (10 s) icinde bulunamazsa listedeki sonraki
    duraga -> KESIF_SURUS. Durak eklemek icin sadece listeye eleman ekle.
    Her yeni durusta planner oy havuzunu sifirlar, yani YOLO o noktadan
    TAZE oy toplar: "tek cep goruldu ve izinli degil" durumunda arac
    kendiliginden sonraki duraga ilerleyip yeniden bakar.
  SON CARE (fallback): SON durakta scan_timeout (10 s) dolunca hemen
    PARK_IPTAL'e gidilmez; secim olcutu BIR KEZ gevsetilir - izni
    okunamamis (permit='unknown') cepler de aday olur, ama acikca
    'park_yasak' gorulmus cep YINE reddedilir (planner.
    slot_available_fallback). Kamera/YOLO olu oldugunda ya da tabelalar
    hicbir duraktan okunamadiginda gorevin komple bosa gitmesini onler.
    Aday yine yoksa -> PARK_IPTAL.
  PARK_IPTAL: Fail-safe kilit; arac tamamen durur, durum surekli yayinlanir.
  Veri cakismasini onlemek icin taramalar SADECE arac dururken yapilir:
  status_callback surus fazlarinda gelen guncellemeleri yoksayar.

Giris Kavisli 3 Asamali Ileri Manevra (Offset Lead-in Path):
  1. YONELME: Arac donuse hazirlanmak icin cebin disina acilir.
     Hedef: (target_x + 5.5, target_y + 2.5 * sign_y). sign_y araca
     baslangic pozisyonunda BIR KEZ atanir (yukaridan geliyorsa +1).
  2. GIRIS: Kavis tamamlanir, cebin agzina dik gelinir.
     Hedef: (target_x + 1.5, target_y).
  3. YANASMA: Dumduz iceri girilir. Hedef: cep merkezi.
  -> PARK_TAMAMLANDI: merkeze goal_tolerance kala kilitlenir.

Direksiyon: Pure Pursuit. Hedef acisi dogrudan waypoint'e degil, aktif yol
segmenti (onceki waypoint -> aktif waypoint) uzerinde aracin izdusumunden
lookahead_dist kadar ilerideki sanal noktaya gore hesaplanir. Boylece arac
cizgiden sapinca sanal nokta onu cizgiye geri ceker (cizgi cignemeyi onler).
YANASMA fazinda lookahead otomatik kisalir (dar alan hassasiyeti).

NOT: Aractaki gazebo_ros_ackermann_drive eklentisi cmd_vel'deki angular.z
degerini acisal hiz olarak DEGIL, hedef direksiyon acisi (rad) olarak
yorumlar ve max_steer=0.6458 rad ile sinirlar. P kontrolcu bu yuzden
dogrudan direksiyon acisi uretir.
"""

import json
import math
from dataclasses import dataclass, fields

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from parking_safety.beemobs_actuator import BeemobsActuator


@dataclass
class ParkingConfig:
    """Park algoritmasinin tum ayarlanabilir parametreleri tek yerde."""

    # Kontrol parametreleri
    kp: float = 0.4                 # P kontrolcu katsayisi (direksiyon)
    approach_speed: float = 0.5     # Ileri hiz [m/s]
    final_stop_tolerance: float = 0.03  # El freni cekme esigi [m]
                                        # Arac merkeze bu kadar kala safe_stop()
                                        # cagrilir. NOT: gercek fren/kayma mesafesi
                                        # HENUZ OLCULMEDI - bu sayi tek basina kesin
                                        # bir "N cm'de durur" garantisi VERMEZ.
                                        # Asil guvence yanasma_ramp_start/min_speed
                                        # ile yaklasirken hizin onceden dusurulmesi
                                        # (bkz. asagi) - fren o an zaten cok yavas
                                        # hizda tetiklenir, kayma kucuk kalir.
    yanasma_ramp_start: float = 1.0  # Bu mesafeden itibaren hiz mesafeyle
                                     # orantili dusurulmeye baslar [m]
    yanasma_min_speed: float = 0.08  # Yavaslama tabani [m/s] - sifira
                                     # inmez (motor/vites gecisi icin;
                                     # beemobs_actuator 0.05 m/s altini N sayar)
    yanasma_lookahead: float = 0.35  # YANASMA fazinda pure pursuit bakis [m]
    yonelme_tolerance: float = 1.0  # Faz 1 -> Faz 2 gecis esigi [m]
    giris_tolerance: float = 0.5    # Faz 2 -> Faz 3 gecis esigi [m]

    # Waypoint ofsetleri (Offset Lead-in Path)
    lead_x_offset: float = 4.5      # Faz 1 hedefinin cep onune x mesafesi [m]
    lead_y_offset: float = 2.5      # Faz 1 yanal acilma [m] (7/8. ceplerde 0'lanir)
    mouth_x_offset: float = 3.5     # Cep agzinin merkeze x mesafesi [m]
    mouth_y_offset: float = 0.0     # Cep agzi yanal kaydirma [m] (kose kesme telafisi,
                                    # sign_y ile carpilir; ters yon icin negatif ver)

    # Pure Pursuit
    lookahead_dist: float = 1.8     # Bakis mesafesi [m] (YANASMA'da otomatik kisalir)

    # Direksiyon Slew Rate Limiter (sadece YONELME fazinda aktif)
    max_steer_rate: float = 0.6     # Direksiyon komutunun maks. degisim hizi [rad/s]

    # Dur-ve-Tara (Stop-and-Scan) kesif sistemi
    scan_timeout: float = 10.0        # Baslangic ve SON durak beklemesi [s]
    corridor_scan_timeout: float = 10.0  # Ara duraklarda bekleme [s]: havuz
                                        # ~0.5 s'de karar uretir, 3 s bol yeter;
                                        # hedef yoksa sonraki duraga gecilir
    # Gozlem noktasina VARIS toleransi. Eski deger (3.5) hedef bir park
    # rotasinin wp1'i iken "varmadan dur" kapisiydi; gozlem noktasi artik
    # hedefin kendisi oldugu icin kucuk tutulmali - yeni haritada baslangic
    # ile ilk gozlem noktasi arasi ~2 m oldugundan buyuk bir deger kalsaydi
    # arac HIC hareket etmezdi.
    kesif_stop_distance: float = 1.0

    # LiDAR ROI (Region of Interest) sinirlari
    x_min: float = 0.5              # Dar alan manevrasi icin kalkan menzili daraltildi
    x_max: float = 0.9
    y_min: float = -0.6
    y_max: float = 0.6
    # Dar kalkan (YANASMA fazi): cep icindeki yan duvar/dubalarin hayalet
    # fren tetiklememesi icin ROI arac genisligine kadar daraltilir.
    # x_min, z_min, z_max her iki kalkan icin ortaktir.
    narrow_x_max: float = 0.6       # Ileri gorus menzili kisalir
    narrow_y_min: float = -0.35     # Aracin sol genisligine yaklasir
    narrow_y_max: float = 0.35      # Aracin sag genisligine yaklasir
    # ---------------------------------------------------------------------
    #  z sinirlari 2026-07-28'de ZEMINE GORE yeniden tanimlandi.
    #
    #  ESKI hal: z_min=-0.1, z_max=1.0 dogrudan HAM velodyne z'sine
    #  uygulaniyordu. lidar_gozlem_0728_1022 + lidar_cepte_gozlem_0728_1029
    #  bag'lerinden (2 x 20 tarama, ~215.000 nokta, en kucuk kareler zemin
    #  duzlemi, artik sd 0.037 m) LiDAR'in zeminden 1.366 m yukarida oldugu
    #  olculdu. Yani eski bant gercekte YERDEN 1.27-2.37 m arasini tariyordu:
    #  duba, bordur, alcak duvar, hatta bir binek aracin govdesi bu bandin
    #  TAMAMEN ALTINDA kalir. Guvenlik kalkani pratikte KORDU.
    #
    #  Yeni degerler zemine gore yukseklik: 0.15 m (duba tabani, asfalt
    #  yansimasi elenir) - 2.00 m (arac tavani). Ham z'ye donusum
    #  lidar_callback icinde ground_z_in_lidar ile yapilir.
    ground_z_in_lidar: float = -1.366  # zeminin HAM velodyne z'si [m] (OLCULDU)
    z_min: float = 0.15             # zeminden yukseklik [m]
    z_max: float = 2.00


# Her cep icin optimize edilmis otonom park profilleri.
# Oncelik zinciri: komut satiri (-p) > cep profili > ParkingConfig varsayilani
#
# lead_x_offset NOTU (U-donus duzeltmesi): YONELME hedefi wp1'in x'i
# target_x + lead_x_offset olarak konur. Eski 8 cepli haritada arac
# spawn'i x=-17.8 idi; lead_x=5.5 ile ust ceplerin wp1'i (~ -18.5) aracin
# ONUNDE (daha negatif x) kaliyordu. YENI haritada spawn x=-19.2 ve kesif
# duraklari x=-20.2..-20.8 oldugu icin lead_x=5.5'lik wp1 (~ -18.5) artik
# aracin ARKASINDA kaliyor; arac -X'e (ceplere) bakarken arkasindaki
# noktaya ulasmak icin U donusu yapiyordu. Cozum: lead_x_offset tum cepler
# icin 3.5'e cekildi (Cep 1-2 zaten 3.5 idi); boylece wp1 (~ -20.5) her
# yaklasma pozunun onune duser ve arac dumduz kavisle YONELME'ye girer.
# Diger tum degerler (lead_y, lookahead, kp, max_steer_rate, mouth_y,
# yonelme_tolerance) eski calisan profillerden BIREBIR korundu.
SLOT_PROFILES = {
    9: {"lookahead_dist": 1.6, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 0.0, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.0, "max_steer_rate": 0.6},
    8: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 1.5, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.3, "max_steer_rate": 0.6},
    7: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.0, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.4, "max_steer_rate": 0.6},
    6: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.4, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.45, "max_steer_rate": 0.6},
    5: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.4, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.45, "max_steer_rate": 0.6},
    4: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.4, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.45, "max_steer_rate": 0.6},
    3: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.4, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.45, "max_steer_rate": 0.6},
    2: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.4, "yonelme_tolerance": 0.5, "mouth_y_offset": 0.45, "max_steer_rate": 0.6},
    1: {"lookahead_dist": 1.3, "kp": 0.5, "lead_x_offset": 3.5, "lead_y_offset": 2.6, "yonelme_tolerance": 0.3, "mouth_y_offset": 0.70, "max_steer_rate": 0.3},
}

# Gecersiz slot id icin kullanilacak varsayilan profil (standart kavis)
DEFAULT_PROFILE_ID = 6

# --- Dinamik Kesif Durak Listesi (Dynamic Stops Structure) ------------------
# Her eleman (X, Y, YAW): durus koordinati + duraktaki istenen bakis yonu
# [rad]. Yeni durak eklemek/cikarmak icin SADECE bu listeyi duzenle;
# durum makinesi durak sayisindan bagimsizdir (indeks sayaci kullanir).
#   - Ara duraklarda corridor_scan_timeout (10 s), SON durakta scan_timeout
#     (10 s) beklenir; son durakta da hedef yoksa once SON CARE olcutu
#     denenir, o da bos donerse PARK_IPTAL.
#   - YAW hizalamasi: Ackermann arac yerinde DONEMEZ; hizalama, duragin
#     yaw yonunun EXPLORATION_ALIGN_DIST gerisine konan sanal noktadan
#     duraga uzanan yaklasim bacagiyla saglanir (pure pursuit bu bacagi
#     izlerken burun yaw'a oturur, arac dogru yone bakarak durur).
#   - DIKKAT (manevra payi): duraklari y < -6.5'e tasima - Cep 1-2'nin
#     lead-in noktalari aracin ONUNDE kalmali ki en zor cep olan Cep 1
#     dahil tum alt cepler park edilebilir kalsin. (Bu kisit ESKI Gazebo
#     duzeni icindi; gercek saha koordinatlarinda gecerliligini sahada
#     dogrulayin.)
# 2026-07-24: GERCEK SAHA GPS olcumunden ENU donusumuyle hesaplandi (bkz.
# parking_mission_planner.py'deki ayni tarihli not - Origin = Start noktasi,
# x=Dogu[m], y=Kuzey[m], use_imu=True sarti gecerli). YAW degerleri, o
# duraktan cep sirasinin ortalamasina bakacak sekilde HESAPLANDI (varsayim);
# duraktaki GERCEK istenen bakis acisini sahada test edip ayarlayin. Eski
# Gazebo simulasyon koordinatlari (-20.2.., -20.8..) artik GECERSIZ, KALDIRILDI.
EXPLORATION_STOPS = [
    (0.025256, 6.415947, 1.400324),    # Ara Kesif Duragi 1
    (-1.979225, 11.185098, 0.964797),  # Ara Kesif Duragi 2
]

# Hizalama bacagi uzunlugu [m]: sanal on-nokta duragin yaw geri yonunde
# bu kadar uzaga konur. Cok kisa = hizalanmaya yol kalmaz; cok uzun =
# arac gereksiz dolanir.
EXPLORATION_ALIGN_DIST = 2.5

# Durum makinesi durumlari
STATE_BEKLE_GPS = 'BEKLE_GPS_BOLGESI'
STATE_ARAYIS_BASLANGIC = 'ARAYIS_BASLANGIC'
STATE_KESIF_SURUS = 'KESIF_SURUS'
# Jenerik durak taramasi: hangi durakta olundugu current_stop_index'te.
# /parking_state yayininda gorunurluk icin 'ARAYIS_DURAK_<n>' yayinlanir.
STATE_ARAYIS_DURAK = 'ARAYIS_DURAK'
STATE_PARK_IPTAL = 'PARK_IPTAL'
STATE_ENGEL = 'ENGEL'
STATE_YONELME = 'YONELME'
STATE_GIRIS = 'GIRIS'
STATE_YANASMA = 'YANASMA'
STATE_PARK_TAMAMLANDI = 'PARK_TAMAMLANDI'


class ParkingDecisionAndControlNode(Node):
    def __init__(self):
        super().__init__('parking_decision_and_control_node')

        # --- Yapilandirma ve ROS 2 Parametreleri -----------------------------
        # Hedef cep artik sabit parametre DEGIL; parking_mission_planner'in
        # /parking/slot_status yayinindan dinamik secilir (koordinatlar dahil).
        # Varsayilanlar ParkingConfig'ten gelir; komut satirindan override:
        #   ros2 run parking_safety parking_decision_and_control_node \
        #       --ros-args -p kp:=0.5
        self.cfg = ParkingConfig()
        self.target_slot_id = None
        self.target_x = 0.0
        self.target_y = 0.0
        # Planner'dan gelen SON durum matrisi (son care secimi icin saklanir)
        self.last_slots = None
        # Son care (fallback) secimi kullanildi mi - /parking_state'e yansir
        self.fallback_used = False

        # 2026-08-02: 'target_slot_id' (SABIT CEP MODU) KALDIRILDI. Cep secimi
        # artik TAMAMEN YOLO (tabela izni) + LiDAR (doluluk) kararina birakildi.
        # O mod 'available' kapisini butunuyle atliyordu; yani dolu ya da
        # 'park_yasak' tabelali bir cebe de girerdi - yaris kosusunda kabul
        # edilemez. Izni okunamayan cepler icin artik kontrollu bir son care
        # yolu var (bkz. select_target / slot_available_fallback).
        # DIKKAT: launch'ta target_slot_id:=N verilirse SESSIZCE yok sayilir
        # (ros2 launch bilinmeyen argumanlari gormezden gelir).

        # GPS Bolge Kapisi: parking_gps_zone_trigger, arac kayitli park
        # bolgesine (GPS geofence) girene kadar /parking_zone_reached=False
        # yayinlar; bu sure boyunca arac TAMAMEN hareketsiz kalir (bkz.
        # STATE_BEKLE_GPS). require_gps_zone=False verilirse (masabasi
        # test / GPS'siz saha denemesi) bu kapı atlanir.
        self.declare_parameter('require_gps_zone', True)
        self.require_gps_zone = bool(self.get_parameter('require_gps_zone').value)
        self.gps_in_zone = False

        # ENU -> yerel /odom donusumu icin durum (bkz. maybe_capture_enu_
        # transform ve parking_mission_planner.py'deki ayni tarihli DUZELTME
        # notu). EXPLORATION_STOPS ENU-mutlak tanimli; /odometry/filtered'in
        # yaw'i acilis anina gore keyfi oldugundan GPS bolgesine girildigi an
        # BIR KEZ yakalanan yaw_offset ile yerel cerceveye cevrilir.
        self.map_x = self.map_y = self.map_yaw = 0.0
        self.map_pose_received = False
        self.enu_transform_captured = False
        self.yaw_offset = 0.0
        self._local_x0 = self._local_y0 = 0.0
        self._map_x0 = self._map_y0 = 0.0
        # Donusum yakalanamazsa HAM ENU'ya duser (eski/hatali ama calisir)
        self.exploration_stops_local = list(EXPLORATION_STOPS)

        # ROS parametreleri ParkingConfig varsayilanlariyla tanimlanir.
        # CLI'da acikca degistirilenler cli_overrides'a kaydedilir; cep profili
        # calisma aninda yuklendiginde yeniden uygulanarak oncelik zinciri
        # (CLI > profil > varsayilan) korunur.
        defaults = ParkingConfig()
        param_names = [f.name for f in fields(ParkingConfig)
                       if not f.name.startswith(('x_', 'y_', 'z_', 'narrow_'))]
        self.cli_overrides = {}
        for name in param_names:
            self.declare_parameter(name, getattr(defaults, name))
            value = self.get_parameter(name).value
            setattr(self.cfg, name, value)
            if value != getattr(defaults, name):
                self.cli_overrides[name] = value
        if self.cli_overrides:
            self.get_logger().info(f'CLI override: {self.cli_overrides}')

        # --- Ic durum ---------------------------------------------------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0        # psi_current [rad]
        self.odom_received = False
        self.obstacle_detected = False
        self.parking_completed = False   # PARK_TAMAMLANDI kilidi
        self.last_state = None
        self.last_steer = 0.0            # Slew limiter icin onceki direksiyon komutu

        # Faz takibi ve waypoint'ler. Arac ARAYIS_BASLANGIC'ta hareketsiz
        # baslar; uygun cep bulununca rota kurulur ve YONELME'ye gecilir.
        # sign_y secim anindaki pozisyona gore BIR KEZ hesaplanir (her
        # donguda hesaplanirsa arac target_y cizgisini gectigi an ziplar).
        self._initial_phase = (STATE_BEKLE_GPS if self.require_gps_zone
                               else STATE_ARAYIS_BASLANGIC)
        self.phase = self._initial_phase
        # Gorev Kapisi: master_kararci /park_gorev='start' demeden dugum
        # tamamen pasiftir (hicbir /cmd_vel/perception_active/parking_state
        # yayinlamaz), boylece PARK bolgesi disinda lane_following ile
        # /beemobs cakismasi olmaz. require_gps_zone/STATE_BEKLE_GPS kapisi
        # KORUNUR; gorev komutu onun ONUNE eklenir.
        self.mission_active = False
        self.target_found = False        # status_callback hedef kilitledi mi?
        self.arayis_start_time = self.get_clock().now()
        self.koridor_start_time = None   # duraga varinca atanir
        self.current_stop_index = 0      # EXPLORATION_STOPS icindeki aktif durak
        self.kesif_leg = 1               # 1: hizalama noktasina, 2: duraga surus
        self.waypoints_ready = False
        self.start_x = self.start_y = 0.0  # Baslangic pozisyonu (YONELME segmentinin basi)
        self.wp1_x = self.wp1_y = 0.0    # YONELME hedefi
        self.wp2_x = self.wp2_y = 0.0    # GIRIS hedefi (cep agzi)

        # --- Abonelikler ve yayinci ------------------------------------------
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # parking_mission_planner'in cep durum matrisi (dinamik hedef secimi)
        self.status_sub = self.create_subscription(
            String, '/parking/slot_status', self.status_callback, 10)
        # parking_gps_zone_trigger'in GPS geofence sinyali (STATE_BEKLE_GPS kapisi)
        self.gps_zone_sub = self.create_subscription(
            Bool, '/parking_zone_reached', self.gps_zone_callback, 10)
        # master_kararci Gorev Kapisi (start/stop)
        self.park_gorev_sub = self.create_subscription(
            String, '/park_gorev', self.park_gorev_callback, 10)
        # GPS-duzeltmeli MUTLAK poz - SADECE yaw_offset yakalamak icin
        # (bkz. maybe_capture_enu_transform); surekli kontrolde HALA /odom
        # (/odometry/filtered, puruzsuz) kullanilir.
        self.map_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered_map', self.map_odom_callback, 10)
        # Arac CAN aktuatoru: kontrol dugumu (ileri hiz, direksiyon acisi) uretir,
        # aktuator DOGRUDAN /beemobs/*'a yazar. Eski cmd_vel_to_beemobs_bridge
        # KALDIRILDI (ara /cmd_vel + kopru yerine dugum araca dogrudan komut verir).
        self.actuator = BeemobsActuator(self)
        # Anlik durum yayini: goruntu isleme gibi diger node'lar buradan okur
        self.state_pub = self.create_publisher(String, '/parking_state', 10)
        # Algi Kilidi: planner'a "guncelleme isle / isleme" sinyali
        self.perception_pub = self.create_publisher(Bool, '/perception_active', 10)
        self.last_perception_active = None  # sadece degisim logu icin
        # RViz gorsellestirilmesi: waypoint'ler, hedef, takip yolu
        self.path_markers_pub = self.create_publisher(
            MarkerArray, '/parking/path_markers', 10)

        # Kontrol dongusu sensor callback'lerinden bagimsiz, sabit 20 Hz calisir
        self.control_dt = 0.05  # [s] - slew limiter da ayni dt'yi kullanir
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            'Parking Decision & Control Node baslatildi. '
            'ARAYIS modunda: /parking/slot_status uzerinden uygun cep bekleniyor.')

    # ------------------------------------------------------------------ LiDAR
    def lidar_callback(self, msg):
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)

        # Dinamik kalkan: YANASMA'da (cep ici) yan duvar/dubalarin hayalet
        # fren tetiklememesi icin dar ROI; diger tum fazlarda standart ROI.
        if self.phase == STATE_YANASMA:
            eff_x_max = self.cfg.narrow_x_max
            eff_y_min = self.cfg.narrow_y_min
            eff_y_max = self.cfg.narrow_y_max
        else:
            eff_x_max = self.cfg.x_max
            eff_y_min = self.cfg.y_min
            eff_y_max = self.cfg.y_max

        # z sinirlari ZEMINE GORE verilir; ham velodyne z'sine cevrilir.
        # (bkz. ParkingConfig.ground_z_in_lidar - sahada olculdu.)
        z_lo = self.cfg.ground_z_in_lidar + self.cfg.z_min
        z_hi = self.cfg.ground_z_in_lidar + self.cfg.z_max

        detected = False
        for point in points:
            x, y, z = point[0], point[1], point[2]
            if (self.cfg.x_min <= x <= eff_x_max) and \
               (eff_y_min <= y <= eff_y_max) and \
               (z_lo <= z <= z_hi):
                detected = True
                break
        self.obstacle_detected = detected

    # ------------------------------------------------------------------- Odom
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Quaternion -> Euler (yaw). tf_transformations sistemde kurulu
        # olmadigi icin dogrudan formul kullaniliyor.
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True
        self.maybe_capture_enu_transform()

    def map_odom_callback(self, msg):
        """GPS-duzeltmeli MUTLAK poz (/odometry/filtered_map, ukf_global)."""
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.map_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.map_pose_received = True
        self.maybe_capture_enu_transform()

    def maybe_capture_enu_transform(self):
        """ENU(Dogu/Kuzey) <-> yerel /odom donusumunu park bolgesine
        girildigi an BIR KEZ yakala (bkz. __init__ yorumu ve
        parking_mission_planner.py'deki ayni mekanizma - iki node
        BAGIMSIZ ama AYNI /parking_zone_reached anina gore hesaplar)."""
        if self.enu_transform_captured:
            return
        if not (self.gps_in_zone and self.odom_received and
                self.map_pose_received):
            return

        self._local_x0 = self.current_x
        self._local_y0 = self.current_y
        self._map_x0 = self.map_x
        self._map_y0 = self.map_y
        self.yaw_offset = math.atan2(
            math.sin(self.map_yaw - self.current_yaw),
            math.cos(self.map_yaw - self.current_yaw))
        self.enu_transform_captured = True

        self.exploration_stops_local = [
            (*self.enu_to_local(px, py),
             math.atan2(math.sin(pyaw - self.yaw_offset),
                        math.cos(pyaw - self.yaw_offset)))
            for (px, py, pyaw) in EXPLORATION_STOPS]

        self.get_logger().info(
            'ENU->yerel /odom donusumu YAKALANDI: '
            f'yaw_offset={math.degrees(self.yaw_offset):.1f} deg. '
            'Kesif duraklari yerel cerceveye donusturuldu.')

    def enu_to_local(self, px_enu, py_enu):
        """Bir ENU (Dogu,Kuzey) noktasini yerel /odom cercevesine cevirir.
        Donusum yakalanmadiysa HAM ENU degerini dondurur (eski/hatali ama
        sessizce cokmez)."""
        if not self.enu_transform_captured:
            return px_enu, py_enu
        dx = px_enu - self._map_x0
        dy = py_enu - self._map_y0
        c = math.cos(self.yaw_offset)
        s = math.sin(self.yaw_offset)
        lx = self._local_x0 + dx * c + dy * s
        ly = self._local_y0 - dx * s + dy * c
        return lx, ly

    # ---------------------------------------------------- Dinamik Hedef Secimi
    def status_callback(self, msg):
        """Guvenli Dinleyici: durum matrisinden hedef cep sec.

        SADECE arac hareketsizken (ARAYIS_BASLANGIC / ARAYIS_DURAK)
        calisir; surus sirasinda gelen guncellemeler veri cakismasi
        yaratmamasi icin yoksayilir. Burada hedef yalnizca KILITLENIR
        (target_found); profil yukleme, rota kurulumu ve faz gecisi arac
        dururken control_loop tarafindan yapilir.
        """
        if self.phase not in (STATE_ARAYIS_BASLANGIC, STATE_ARAYIS_DURAK) \
                or self.parking_completed or self.target_found:
            return
        if not self.odom_received:
            # En yakin cep secimi anlik pozisyona ihtiyac duyar
            self.get_logger().warn('Cep durumu geldi ama odom bekleniyor...',
                                   throttle_duration_sec=2.0)
            return

        try:
            slots = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Gecersiz slot_status JSON atlandi: {exc}')
            return

        # Son matris saklanir: son care secimi (fallback) bir sonraki mesaji
        # beklemeden, durak zaman asiminin TAM o aninda calisabilsin.
        self.last_slots = slots
        self.select_target(slots, use_fallback=False)

    def select_target(self, slots, use_fallback):
        """Durum matrisinden en yakin uygun cebi sec ve hedefi KILITLE.

        use_fallback=False (normal): planner'in strict 'available' alani
            kullanilir - cep bos VE izni 'park_edilebilir' olarak DOGRULANMIS
            olmalidir. Karari tamamen YOLO+LiDAR verir.
        use_fallback=True (SON CARE): 'available_fallback' alani kullanilir -
            izni okunamamis (unknown) cepler de aday olur, ama acikca
            'park_yasak' gorulmus cep YINE reddedilir. Sadece tum kesif
            duraklari tukendiginde cagirilir (bkz. control_loop son durak).

        Donus: hedef kilitlendiyse True.
        """
        key = 'available_fallback' if use_fallback else 'available'
        best_id = None
        best_dist = float('inf')
        best_info = None
        for sid_str, info in slots.items():
            try:
                if not info.get(key, False):
                    continue
                sid = int(sid_str)
                dist = math.hypot(info['x'] - self.current_x,
                                  info['y'] - self.current_y)
            except (KeyError, TypeError, ValueError):
                continue  # bozuk kayit: digerlerini degerlendirmeye devam et
            if dist < best_dist:
                best_id, best_dist, best_info = sid, dist, info

        if best_id is None:
            if not use_fallback:
                self.get_logger().info(
                    'ARAYIS: uygun (available) cep yok, bekleniyor.',
                    throttle_duration_sec=2.0)
            return False

        # Hedef kilitlenir: koordinatlar durum matrisinden alinir.
        # Faz gecisini control_loop yapacak (arac zaten hareketsiz).
        self.target_slot_id = best_id
        self.target_x = float(best_info['x'])
        self.target_y = float(best_info['y'])
        self.target_found = True
        if use_fallback:
            self.get_logger().error(
                f'SON CARE HEDEFI: Cep {best_id} '
                f'({self.target_x:.2f}, {self.target_y:.2f}), '
                f'uzaklik={best_dist:.2f} m | izin='
                f'{best_info.get("permit", "?")} - TABELA IZNI DOGRULANAMADI, '
                'gorev iptali yerine bu cep deneniyor.')
        else:
            self.get_logger().info(
                f'HEDEF SECILDI: Cep {best_id} '
                f'({self.target_x:.2f}, {self.target_y:.2f}), '
                f'uzaklik={best_dist:.2f} m | izin='
                f'{best_info.get("permit", "?")}')
        return True

    def gps_zone_callback(self, msg):
        """parking_gps_zone_trigger'dan gelen GPS geofence durumu."""
        self.gps_in_zone = bool(msg.data)
        self.maybe_capture_enu_transform()

    def park_gorev_callback(self, msg):
        """master_kararci Gorev Kapisi: 'start' gorevi baslatir, 'stop' pasife
        alir ve fazi baslangica dondurur (arac /cmd_vel yayinlamayi keser)."""
        cmd = str(msg.data).strip().lower()
        if cmd == 'start':
            if not self.mission_active:
                self.mission_active = True
                self.actuator.set_active(True)   # aktuator devrede (deadman acik)
                # Yeni gorev: durumu sifirla (tekrar calisabilsin)
                self.phase = self._initial_phase
                self.parking_completed = False
                self.target_found = False
                self.arayis_start_time = self.get_clock().now()
                self.get_logger().info('PARK GOREVI BASLADI (/park_gorev start)')
        elif cmd == 'stop':
            if self.mission_active:
                self.mission_active = False
                self.actuator.set_active(False)  # aktuator susar (/beemobs birak)
                self.phase = self._initial_phase
                self.get_logger().info('PARK GOREVI DURDURULDU (/park_gorev stop)')

    def start_park_sequence(self):
        """Kilitlenen hedef icin profili yukle, rotayi anlik konumdan kur
        ve YONELME fazini baslat. CLI override'lari profili ezer."""
        self.load_slot_profile(self.target_slot_id)
        for name, value in self.cli_overrides.items():
            setattr(self.cfg, name, value)
        self.setup_waypoints()
        self.phase = STATE_YONELME
        self.get_logger().info(
            f'PARK SEKANSI BASLIYOR: Cep {self.target_slot_id} (YONELME)')

    def setup_exploration_route(self):
        """Kesif surusu rotasini kur: hedef bir cep DEGIL, gorus noktasi.

        Pure pursuit ayni makineyi kullanir, iki bacakli:
          bacak 1: baslangic -> wp1 (hizalama noktasi: duragin yaw
                   yonunun EXPLORATION_ALIGN_DIST gerisi)
          bacak 2: wp1 -> wp2 (durak). Bu segment yaw dogrultusunda
                   uzandigi icin arac duraga yaw'a hizalanmis varir.
        Hedef/GIRIS/YANASMA kullanilmaz. Surus parametreleri icin standart
        kavis profili yuklenir - herhangi bir cebin dar-alan ayarlari
        (orn. Cep 1'in 0.22 rad/s direksiyonu) kesif surusune uygulanmaz.
        """
        self.load_slot_profile(DEFAULT_PROFILE_ID)
        for name, value in self.cli_overrides.items():
            setattr(self.cfg, name, value)
        stop_x, stop_y, stop_yaw = self.exploration_stops_local[self.current_stop_index]
        self.start_x = self.current_x
        self.start_y = self.current_y
        # Hizalama bacagi: sanal on-nokta, duragin yaw yonunun gerisinde.
        # Arac once wp1'e (on-nokta), sonra wp1->wp2 (durak) segmentini
        # izleyerek gider; boylece duraga istenen YAW ile hizalanmis varir.
        self.wp1_x = stop_x - EXPLORATION_ALIGN_DIST * math.cos(stop_yaw)
        self.wp1_y = stop_y - EXPLORATION_ALIGN_DIST * math.sin(stop_yaw)
        self.wp2_x, self.wp2_y = stop_x, stop_y
        self.kesif_leg = 1
        self.waypoints_ready = True
        self.get_logger().info(
            f'Kesif rotasi kuruldu (durak {self.current_stop_index + 1}/'
            f'{len(EXPLORATION_STOPS)}): '
            f'({self.start_x:.1f}, {self.start_y:.1f}) '
            f'-> hizalama ({self.wp1_x:.1f}, {self.wp1_y:.1f}) '
            f'-> durak ({stop_x:.1f}, {stop_y:.1f}, '
            f'yaw={math.degrees(stop_yaw):.0f} deg)')

    def setup_waypoints(self):
        """Waypoint'leri secim anindaki pozisyona gore bir kez kur."""
        # Baslangic noktasi YONELME segmentinin basi olarak kaydedilir
        # (pure pursuit izlenecek cizgiyi baslangic -> wp1 olarak bilir)
        self.start_x = self.current_x
        self.start_y = self.current_y
        sign_y = 1.0 if (self.current_y > self.target_y) else -1.0
        self.wp1_x = self.target_x + self.cfg.lead_x_offset
        self.wp1_y = self.target_y + (self.cfg.lead_y_offset * sign_y)
        self.wp2_x = self.target_x + self.cfg.mouth_x_offset
        # Giris Yanal Kaydirma: kose kesme telafisi icin cep agzi
        # yaklasim yonune gore hafifce kaydirilabilir (varsayilan 0)
        self.wp2_y = self.target_y + (self.cfg.mouth_y_offset * sign_y)
        self.waypoints_ready = True
        self.get_logger().info(
            f'Waypointler kuruldu (sign_y={sign_y:+.0f}): '
            f'YONELME=({self.wp1_x:.1f}, {self.wp1_y:.1f}) -> '
            f'GIRIS=({self.wp2_x:.1f}, {self.wp2_y:.1f}) -> '
            f'YANASMA=({self.target_x:.1f}, {self.target_y:.1f})')

    # -------------------------------------------------------------- Yardimci
    def load_slot_profile(self, slot_id):
        """SLOT_PROFILES'taki cep bazli optimize degerleri cfg'ye yukle.

        Sozlukte olmayan bir id gelirse standart kavis profili
        (SLOT_PROFILES[DEFAULT_PROFILE_ID]) kullanilir.
        """
        if slot_id in SLOT_PROFILES:
            profile = SLOT_PROFILES[slot_id]
        else:
            profile = SLOT_PROFILES[DEFAULT_PROFILE_ID]
            self.get_logger().warn(
                f'Cep {slot_id} icin profil tanimli degil; varsayilan profil '
                f'(Cep {DEFAULT_PROFILE_ID}, standart kavis) kullaniliyor.')

        for key, value in profile.items():
            setattr(self.cfg, key, value)
        self.get_logger().info(f'Cep {slot_id} profili yuklendi: {profile}')

    @staticmethod
    def normalize_angle(angle):
        """Aciyi [-pi, pi] araligina normalize et."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def lookahead_point(self, ax, ay, bx, by, ld):
        """Pure Pursuit: A->B yol segmenti uzerindeki sanal hedef noktayi bul.

        Aracin pozisyonu segmentin uzerine dik izdusurulur, izdusumden ld
        (bakis mesafesi) kadar segment boyunca ilerlenir. Nokta segment
        sonunu asarsa hedef waypoint'in kendisi dondurulur; boylece faz
        gecis mantigi (waypoint'e uzaklik) aynen calismaya devam eder.
        """
        seg_dx = bx - ax
        seg_dy = by - ay
        seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
        if seg_len_sq < 1e-9:
            return bx, by

        # Izdusum orani (0..1 araligina kirpilir: segmentin disina tasma)
        t = ((self.current_x - ax) * seg_dx +
             (self.current_y - ay) * seg_dy) / seg_len_sq
        t = max(0.0, min(1.0, t))

        # Izdusumden ld kadar ilerle; segment sonunda hedefe kilitlen
        t_la = min(1.0, t + ld / math.sqrt(seg_len_sq))
        return ax + t_la * seg_dx, ay + t_la * seg_dy

    def apply_slew_limit(self, steer):
        """Direksiyon Slew Rate Limiter: YONELME ve KESIF_SURUS'te aktif.

        Ilk kalkistaki ani direksiyon kirmasini yumusatir; komut bir donguda
        en fazla max_steer_rate * dt kadar degisebilir. GIRIS/YANASMA'da
        devre disi kalir ki dar alanda pure pursuit gecikmesiz tepki versin.
        last_steer her fazda guncellenir (sureklilik icin).
        """
        if self.phase in (STATE_YONELME, STATE_KESIF_SURUS):
            max_delta = self.cfg.max_steer_rate * self.control_dt
            steer = max(self.last_steer - max_delta,
                        min(self.last_steer + max_delta, steer))
        self.last_steer = steer
        return steer

    def publish_state(self, state):
        """Anlik durumu /parking_state topigine yayinla."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def publish_path_markers(self):
        """RViz'de waypoint'ler (pembe), hedef (kirmizi ok), takip yolu (mavi)."""
        arr = MarkerArray()
        if not self.waypoints_ready:
            self.path_markers_pub.publish(arr)
            return
        # Waypoint'ler
        for i, (wx, wy) in enumerate([(self.wp0_x, self.wp0_y),
                                        (self.wp1_x, self.wp1_y),
                                        (self.wp2_x, self.wp2_y),
                                        (self.target_x, self.target_y)]):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.15
            if i < 3:
                m.color.r, m.color.g, m.color.b = 1.0, 0.75, 0.8  # pembe
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0   # kirmizi (hedef)
            m.color.a = 0.8
            arr.markers.append(m)
        # Araç konumu -> hedef isaret
        if self.target_found:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 100
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = self.current_x
            m.pose.position.y = self.current_y
            m.pose.position.z = 0.0
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dist = math.hypot(dx, dy)
            if dist > 0.01:
                m.pose.orientation.z = math.sin(
                    math.atan2(dy, dx) / 2.0)
                m.pose.orientation.w = math.cos(
                    math.atan2(dy, dx) / 2.0)
            m.scale.x = max(0.2, dist)
            m.scale.y = m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            m.color.a = 0.6
            arr.markers.append(m)
        # Takip edilen yol (mavi cizgi)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 101
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0
        m.color.a = 0.6
        # wp0 -> wp1 -> wp2 -> target
        for wx, wy in [(self.wp0_x, self.wp0_y),
                       (self.wp1_x, self.wp1_y),
                       (self.wp2_x, self.wp2_y)]:
            p = m.pose.position.__class__()
            p.x, p.y, p.z = wx, wy, 0.05
            m.points.append(p)
        arr.markers.append(m)
        self.path_markers_pub.publish(arr)

    def elapsed_sec(self, start_time):
        """start_time'dan bu yana gecen sureyi saniye olarak dondur."""
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def publish_stationary(self, state, cmd):
        """Hareketsiz fazlarda (tarama/iptal) durum + sifir komut yayinla."""
        if state != self.last_state:
            self.get_logger().info(f'Durum degisti: {self.last_state} -> {state}')
            self.last_state = state
        self.publish_state(state)
        self.actuator.tick(cmd.linear.x, cmd.angular.z)

    def steer_and_speed(self, e_theta):
        """P kontrolcu + deadband (titreme engelleme) + dinamik yavaslama."""
        # Direksiyon Olu Bolgesi (Deadband) - Titremeyi engeller
        if abs(e_theta) < 0.05:  # Yaklasik 3 derece hata varsa direksiyonla oynama
            steer = 0.0
        else:
            steer = self.cfg.kp * e_theta

        # Toleransli hiz profili: hata buyudukce yavasla
        speed_factor = max(0.5, 1.0 - (abs(e_theta) / 2.0))
        speed = self.cfg.approach_speed * speed_factor
        return steer, speed

    def publish_perception_gate(self):
        """Algi Kilidi (Stop-and-Stare): planner'a algi izni yayinla.

        Kural tek ve nettir: arac SADECE dur-ve-tara fazlarinda hareketsiz
        oldugu icin algi yalnizca o fazlarda aciktir. Surus fazlarinda
        (KESIF_SURUS dahil), ENGEL duruslarinda ve gorev bitiminde kapali.
        20 Hz yayinlanir ki planner sonradan baslasa bile senkronlansin.
        """
        active = (not self.parking_completed and
                  self.phase in (STATE_ARAYIS_BASLANGIC, STATE_ARAYIS_DURAK))
        msg = Bool()
        msg.data = active
        self.perception_pub.publish(msg)
        if active != self.last_perception_active:
            self.get_logger().info(
                f'Algi Kilidi: {"ACIK (tarama)" if active else "KAPALI (hareket/bitis)"}')
            self.last_perception_active = active

    # ------------------------------------------------------- Durum Makinesi
    def control_loop(self):
        cmd = Twist()

        # Gorev Kapisi: master 'start' demeden dugum tamamen sessizdir. Hicbir
        # /cmd_vel, /parking_state veya /perception_active yayinlanmaz ki PARK
        # bolgesi disinda /beemobs uzerinde lane_following ile cakisma olmasin.
        if not self.mission_active:
            return

        # Algi Kilidi sinyali her donguda yayinlanir (faz ne olursa olsun)
        self.publish_perception_gate()

        # GPS Bolge Kapisi: parking_gps_zone_trigger'dan /parking_zone_reached
        # gelene kadar arac TAMAMEN hareketsiz kalir (LiDAR/YOLO taramasi da
        # baslamaz, cunku BEKLE_GPS, algi kilidi acik fazlarda degil).
        # GPS bolgesine girilince ARAYIS_BASLANGIC'a gecilir ve tarama
        # zaman asimi SIFIRDAN baslar.
        if self.phase == STATE_BEKLE_GPS:
            if self.gps_in_zone:
                self.phase = STATE_ARAYIS_BASLANGIC
                self.arayis_start_time = self.get_clock().now()
                self.get_logger().info(
                    'GPS PARK BOLGESINE GIRILDI - CEP ARAMASI BASLIYOR')
            else:
                self.get_logger().info(
                    'GPS park bolgesi bekleniyor (/parking_zone_reached)...',
                    throttle_duration_sec=5.0)
                self.publish_stationary(STATE_BEKLE_GPS, cmd)
                return

        # PARK_TAMAMLANDI kilidi: park bir kez tamamlandiysa arac bir daha hareket etmez
        # !!! 2026-07-30 DUZELTME: eskiden buradan tick(0,0) cagriliyordu. Vites
        # gecisi (DRIVE->NEUTRAL) SADECE current_speed_kmh > gear_change_speed
        # iken fren basiyor (bkz. beemobs_actuator.tick); FB_VehicleSpeed 1 km/h
        # cozunurluklu oldugu icin park hizinda (~1-1.8 km/h) cogu zaman 0 okur,
        # bu sart hic saglanmaz ve tick(0,0) fiilen release_brake() cagirir ->
        # arac FREN BASMADAN sadece gazi kesip surunerek durur (saha 30.07
        # bulgusu: cebe girdi ama durup fren yapmadi). safe_stop() hiz
        # okumasindan BAGIMSIZ olarak fren+direksiyon durdurma+vites N uygular.
        if self.parking_completed:
            self.publish_state(STATE_PARK_TAMAMLANDI)
            self.actuator.safe_stop(emergency=False)
            return

        # PARK_IPTAL kilidi (fail-safe): gorev iptal, arac tamamen kilitli
        if self.phase == STATE_PARK_IPTAL:
            self.get_logger().error(
                'GOREV IPTAL: Otoparkta uygun yer bulunamadi',
                throttle_duration_sec=5.0)
            self.publish_stationary(STATE_PARK_IPTAL, cmd)
            return

        # Odom gelmeden ne tarama zaman asimi islet ne hareket et
        if not self.odom_received:
            self.get_logger().warn('Odom verisi bekleniyor...',
                                   throttle_duration_sec=2.0)
            self.actuator.tick(cmd.linear.x, cmd.angular.z)
            return

        # --- Dur-ve-Tara fazlari (arac hareketsiz) ---------------------------
        if self.phase == STATE_ARAYIS_BASLANGIC:
            if self.target_found:
                self.start_park_sequence()
            elif self.elapsed_sec(self.arayis_start_time) > self.cfg.scan_timeout:
                # Hedef yok: ilk gorus noktasina kesif surusu. Bu bir cebin
                # park rotasi DEGILDIR; occlusion golgesindeki tabelalari
                # capraz gorecek bakis noktasidir. target_slot_id atanmaz.
                self.current_stop_index = 0
                self.setup_exploration_route()
                self.phase = STATE_KESIF_SURUS
                self.get_logger().info(
                    'BASLANGIC TARAMASINDA HEDEF BULUNAMADI - KESIF SURUSU '
                    f'BASLIYOR (durak 1/{len(EXPLORATION_STOPS)})')
            else:
                self.get_logger().info(
                    'ARAYIS (baslangic): uygun park cebi bekleniyor...',
                    throttle_duration_sec=2.0)
            self.publish_stationary(STATE_ARAYIS_BASLANGIC, cmd)
            return

        if self.phase == STATE_ARAYIS_DURAK:
            # Yayinlanan durum indeksle etiketlenir (gozlemlenebilirlik);
            # gecis olsa bile bu dongu mevcut duragin etiketini yayinlar.
            state = f'{STATE_ARAYIS_DURAK}_{self.current_stop_index + 1}'
            # Son durakta uzun bekleme (scan_timeout) sonrasi gorev iptali;
            # ara duraklarda kisa bekleme (corridor_scan_timeout) sonrasi
            # listedeki SONRAKI duraga gecilir (dinamik durak listesi).
            is_last_stop = (self.current_stop_index >=
                            len(EXPLORATION_STOPS) - 1)
            timeout = (self.cfg.scan_timeout if is_last_stop
                       else self.cfg.corridor_scan_timeout)
            if self.target_found:
                # Rota, duraktaki ANLIK konuma gore yeniden kurulur
                self.start_park_sequence()
            elif self.elapsed_sec(self.koridor_start_time) > timeout:
                if is_last_stop:
                    # SON CARE: tum duraklar tarandi, izni DOGRULANMIS cep yok.
                    # Iptal etmeden once olcut bir kez gevsetilir - izni
                    # okunamamis (unknown) cepler de aday olur. Acikca
                    # 'park_yasak' gorulmus cep burada da REDDEDILIR.
                    # Bu, kamera/YOLO olu oldugunda ya da tabelalar hicbir
                    # duraktan okunamadiginda gorevin komple bosa gitmesini
                    # onler (bkz. planner.slot_available_fallback).
                    if self.last_slots is not None and \
                            self.select_target(self.last_slots, use_fallback=True):
                        self.fallback_used = True
                        self.start_park_sequence()
                    else:
                        self.phase = STATE_PARK_IPTAL
                        self.get_logger().error(
                            'GOREV IPTAL: Otoparkta uygun yer bulunamadi '
                            '(son care olcutu de aday bulamadi)')
                else:
                    self.current_stop_index += 1
                    self.setup_exploration_route()
                    self.phase = STATE_KESIF_SURUS
                    self.get_logger().info(
                        f'{state}: hedef yok -> SONRAKI duraga '
                        f'({self.current_stop_index + 1}/'
                        f'{len(EXPLORATION_STOPS)}) kesif surusu')
            else:
                self.get_logger().info(
                    f'ARAYIS ({state}): uygun park cebi bekleniyor...',
                    throttle_duration_sec=2.0)
            self.publish_stationary(state, cmd)
            return

        # Surus fazlari: waypoint kurulmadan hareket etme (savunma kontrolu)
        if not self.waypoints_ready:
            self.get_logger().warn('Waypoint kurulumu bekleniyor...',
                                   throttle_duration_sec=2.0)
            self.actuator.tick(cmd.linear.x, cmd.angular.z)
            return

        # KESIF_SURUS iki bacakli: once hizalama noktasi (wp1), sonra durak
        # (wp2). Ikinci bacak duragin YAW yonunde uzandigi icin arac duraga
        # istenen bakis yonuyle hizalanmis varir (Ackermann yerinde donemez).
        if self.phase == STATE_KESIF_SURUS:
            if self.kesif_leg == 1:
                d1 = math.hypot(self.wp1_x - self.current_x,
                                self.wp1_y - self.current_y)
                if d1 <= self.cfg.kesif_stop_distance:
                    self.kesif_leg = 2
                    self.get_logger().info(
                        'Hizalama noktasi gecildi - duraga yaw hizali '
                        'yaklasma basliyor')
            if self.kesif_leg == 2:
                d2 = math.hypot(self.wp2_x - self.current_x,
                                self.wp2_y - self.current_y)
                if d2 <= self.cfg.kesif_stop_distance:
                    stop_yaw = self.exploration_stops_local[self.current_stop_index][2]
                    yaw_err = math.degrees(
                        self.normalize_angle(stop_yaw - self.current_yaw))
                    self.phase = STATE_ARAYIS_DURAK
                    self.koridor_start_time = self.get_clock().now()
                    self.last_steer = 0.0
                    state = f'{STATE_ARAYIS_DURAK}_{self.current_stop_index + 1}'
                    self.get_logger().info(
                        f'DURAK {self.current_stop_index + 1}/'
                        f'{len(EXPLORATION_STOPS)} ULASILDI '
                        f'(yaw hatasi {yaw_err:.1f} deg) - DUR VE TARA')
                    self.publish_stationary(state, cmd)
                    return

        # --- Faz gecisleri (kademeli, tek yonlu) -----------------------------
        if self.phase == STATE_YONELME:
            d1 = math.hypot(self.wp1_x - self.current_x,
                            self.wp1_y - self.current_y)
            if d1 < self.cfg.yonelme_tolerance:
                self.phase = STATE_GIRIS
                self.get_logger().info(
                    'YONELME TAMAMLANDI - CEP AGZINA (GIRIS) DONULUYOR')

        if self.phase == STATE_GIRIS:
            d2 = math.hypot(self.wp2_x - self.current_x,
                            self.wp2_y - self.current_y)
            if d2 < self.cfg.giris_tolerance:
                self.phase = STATE_YANASMA
                self.get_logger().info(
                    'CEP AGZINA ULASILDI - DUMDUZ ICERI GIRILIYOR (YANASMA)')

        # --- Aktif yol segmenti (A -> B) ve waypoint --------------------------
        # Pure pursuit izlenecek cizgiyi bilmek zorunda: her fazin segmenti,
        # onceki waypoint'ten (YONELME'de baslangic noktasindan) aktif
        # waypoint'e uzanan dogru parcasidir.
        if self.phase == STATE_KESIF_SURUS:
            if self.kesif_leg == 1:
                seg_ax, seg_ay = self.start_x, self.start_y
                waypoint_x, waypoint_y = self.wp1_x, self.wp1_y
            else:  # bacak 2: hizalama noktasi -> durak (yaw hizali varis)
                seg_ax, seg_ay = self.wp1_x, self.wp1_y
                waypoint_x, waypoint_y = self.wp2_x, self.wp2_y
        elif self.phase == STATE_YONELME:
            seg_ax, seg_ay = self.start_x, self.start_y
            waypoint_x, waypoint_y = self.wp1_x, self.wp1_y
        elif self.phase == STATE_GIRIS:
            seg_ax, seg_ay = self.wp1_x, self.wp1_y
            waypoint_x, waypoint_y = self.wp2_x, self.wp2_y
        else:  # YANASMA
            seg_ax, seg_ay = self.wp2_x, self.wp2_y
            waypoint_x, waypoint_y = self.target_x, self.target_y

        # Faz gecisleri ve park tamamlama kontrolu waypoint'e uzakliga bakar
        dx = waypoint_x - self.current_x
        dy = waypoint_y - self.current_y
        distance = math.hypot(dx, dy)
        e_theta = None  # surus fazlarinda asagida hesaplanir (log icin)

        if self.obstacle_detected:
            # DURUM: ENGEL -> acil fren (slew limiti uygulanmaz, aninda sifir)
            state = STATE_ENGEL
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.last_steer = 0.0

        elif self.phase == STATE_YANASMA and distance <= self.cfg.final_stop_tolerance:
            # DURUM: PARK TAMAMLANDI -> dur ve kilitle (fren + motor dur + vites N)
            # Araç merkeze final_stop_tolerance kala FREN çekilir. Bu noktaya
            # gelene kadar hiz zaten yanasma_ramp_start/min_speed ile kademeli
            # dusurulmustur (bkz. YANASMA suruş bloğu) - kayma miktari bu
            # sayede kucuk kalir, ama gercek fren mesafesi olculmedigi icin
            # kesin bir cm garantisi verilemez.
            state = STATE_PARK_TAMAMLANDI
            self.parking_completed = True
            self.actuator.safe_stop(emergency=False)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.last_steer = 0.0
            self.get_logger().info('PARK ISLEMI BASARIYLA TAMAMLANDI')

        else:
            # DURUM: YONELME / GIRIS / YANASMA -> Pure Pursuit ile segmenti izle
            state = self.phase

            # Dinamik Bakis Horizonu: cep icinde (YANASMA) lookahead kisalir,
            # boylece dar alanda hassas takip yapilir; koridorda uzun kalir,
            # boylece surus puruzsuz olur.
            if self.phase == STATE_YANASMA:
                ld = self.cfg.yanasma_lookahead
            else:
                ld = self.cfg.lookahead_dist

            la_x, la_y = self.lookahead_point(
                seg_ax, seg_ay, waypoint_x, waypoint_y, ld)
            psi_target = math.atan2(la_y - self.current_y,
                                    la_x - self.current_x)
            e_theta = self.normalize_angle(psi_target - self.current_yaw)
            steer, cmd.linear.x = self.steer_and_speed(e_theta)
            cmd.angular.z = self.apply_slew_limit(steer)

            # YANASMA Yavaslama Rampasi (2026-07-31): fren aninda gercek
            # yavaslama/kayma mesafesi OLCULMEDI - sabit bir "N cm'de fren
            # yeter" varsayimi sahada YANLIS cikti (final_stop_tolerance=0.03
            # ile bile arac cep disina tasti), cunku o ana kadar arac hala
            # tam approach_speed'te (e_theta kucukken speed_factor~1.0) surer.
            # Dogru yaklasim: hedefe yaklastikca hizi mesafeye gore KADEMELI
            # dusurmek - boylece safe_stop() ne zaman/nerede tetiklenirse
            # tetiklensin, o andaki gercek hiz zaten dusuk oldugu icin kayma
            # miktarindan BAGIMSIZ olarak kucuk kalir (kesin cm garantisi
            # DEGIL - gercek fren mesafesi olculmeden hicbir sayı kesin
            # olamaz - ama ani tam hizda fren yerine kademeli yavaslama).
            if self.phase == STATE_YANASMA:
                ramp_frac = min(1.0, distance / self.cfg.yanasma_ramp_start)
                cmd.linear.x = max(self.cfg.yanasma_min_speed,
                                   cmd.linear.x * ramp_frac)

        if state != self.last_state:
            self.get_logger().info(f'Durum degisti: {self.last_state} -> {state}')
            self.last_state = state
        elif state in (STATE_KESIF_SURUS, STATE_YONELME, STATE_GIRIS, STATE_YANASMA):
            self.get_logger().info(
                f'{state} | uzaklik={distance:.2f} m | '
                f'e_theta={math.degrees(e_theta):.1f} deg (look-ahead)',
                throttle_duration_sec=1.0)
        elif state == STATE_ENGEL:
            self.get_logger().warn('ENGEL TESPIT EDILDI - ACIL FREN',
                                   throttle_duration_sec=1.0)

        self.publish_state(state)
        self.publish_path_markers()
        # PARK_TAMAMLANDI'ya bu dongude ilk giriste de (bir sonraki dongude
        # yukaridaki erken-donus degil, bu dal calisir) fren hiz okumasindan
        # BAGIMSIZ basilsin - ayni gerekce icin yukaridaki 2026-07-30 notuna bkz.
        if state == STATE_PARK_TAMAMLANDI:
            self.actuator.safe_stop(emergency=False)
        else:
            self.actuator.tick(cmd.linear.x, cmd.angular.z)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingDecisionAndControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
