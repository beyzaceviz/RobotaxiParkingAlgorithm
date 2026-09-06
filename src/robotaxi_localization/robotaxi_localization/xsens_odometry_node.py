#!/usr/bin/env python3
"""Xsens MTi GNSS/INS -> /odometry/filtered + /odometry/filtered_map (DOGRUDAN).

NEDEN BU DUGUM VAR (2026-07-28 saha verisi kararı)
==================================================
Arac uzerinde iki ayri yon/konum kaynagi var:

  * Xsens MTi (xsens_mti_node): /filter/positionlla, /filter/quaternion,
    /filter/velocity, /gnss, /imu/data     -> GNSS/INS, KENDI ICINDE FUZYONLU
  * ZED2 kamerasi: /zed/zed_node/imu/data  -> salt IMU (GNSS yok)

ukf_fusion.yaml (robot_localization cift-UKF) ZED IMU'sunu okuyacak sekilde
yapilandirilmis. Oysa saha bag'lerinden olculen kalite Xsens lehine cok acik:

    arac cepte DURURKEN (5 cep bag'i, 2200-3900 ornek):
        konum sacilmasi   0.01 - 0.03 m
        quaternion yaw sd 0.03 - 0.15 derece
    surus sirasinda ornekler arasi konum adimi:
        medyan 0.0 - 4.0 cm,  maksimum 6.8 cm  (metre mertebesinde SICRAMA YOK)

Yani Xsens'in kendi cozumu, uzerine bir UKF kurmaya gerek kalmayacak kadar
puruzsuz. UKF katmani su anki halinde fayda saglamak yerine uc bagimlilik
ekliyor: (a) ZED IMU'su, (b) /wheel/odometry ureten wheel_odometry_node,
(c) navsat_transform + manyetik sapma ayari. Ucu de sahada dogrulanmadi;
manyetometrenin hard-iron bozulmasi ayrica olculdu (yatay bilesen iki kayit
arasinda %27 degisiyor -> manyetik yon KULLANILAMAZ).

Bu dugum, park boru hattinin ihtiyac duydugu iki topic'i Xsens'ten DOGRUDAN
uretir. UKF'i SILMEZ; fusion_localization.launch.py'deki secim argumaniyla
(localization_source) hangisinin calisacagi belirlenir.

URETTIGI TOPIC'LER
==================
  /odometry/filtered_map  (frame: map)
      ENU-MUTLAK poz. Orijin = ENU_LAT0/ENU_LON0 (parking_mission_planner'daki
      cep koordinatlariyla AYNI orijin olmak ZORUNDA). yaw = ham quaternion.
      parking_mission_planner bunu SADECE yaw_offset'i bir kez yakalamak icin
      okur (maybe_capture_enu_transform).

  /odometry/filtered      (frame: odom)
      Kontrol icin kullanilan puruzsuz/yerel poz. UKF_local'in davranisini
      taklit eder: yaw, dugumun ILK okudugu yone gore rölatiftir (yaw=0 =
      acilis yonu), konum da ilk poza gore. Park dugumleri bunu /odom olarak
      remap edip kullanir; ENU cep koordinatlari enu_to_local() ile bu
      cerceveye tasinir.

SINIRLARI (bilerek yapilmadi)
=============================
  * Tekerlek odometrisi ile fuzyon YOK. GNSS kesilirse (tunel, kopru alti)
    poz DONAR - park alani acik gokyuzu oldugu icin kabul edildi. Kesinti
    tespiti icin gnss_timeout parametresi var; asilirsa UYARI loglanir ve
    /odometry/* yayini DURUR (park dugumleri odom'suz kalinca hareket etmez).
  * Kovaryans, Xsens'in bildirdigi degil sabit bir tahmindir (asagi bkz.).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from nav_msgs.msg import Odometry


# WGS84 elipsoit sabitleri. 2026-07-28: onceki kod tek bir kuresel yaricap
# (R=6378137, ekvator) kullaniyordu. 40.79 derece enlemde dogru yaricaplar:
#   N (birinci dikey, DOGU yonu icin) = 6387268 m  -> eski deger %0.143 KISA
#   M (meridyen,      KUZEY yonu icin) = 6362688 m -> eski deger %0.243 UZUN
# Ikisi farkli oldugu icin tek yaricap sadece olcek degil SEKIL de bozuyordu.
# parking_mission_planner'daki cep koordinatlari da ayni formulle uretildi;
# ikisi AYNI kalmak ZORUNDA, yoksa cepler sistematik kayar.
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


class XsensOdometryNode(Node):

    def __init__(self):
        super().__init__('xsens_odometry_node')

        # ENU orijini: parking_mission_planner.parking_slots ile AYNI olmali.
        # Degistirilirse cep koordinatlari da ayni orijine gore yeniden
        # hesaplanmalidir - yoksa cepler sistematik olarak kayar.
        self.declare_parameter('enu_lat0', 40.7901429)
        self.declare_parameter('enu_lon0', 29.5092136)
        # GNSS kesintisi esigi [s]. Asilirsa yayin durur (sessiz bayat poz YOK).
        self.declare_parameter('gnss_timeout', 1.0)
        # Sabit kovaryans tahminleri. Saha olcumu: durgun sacilma 0.01-0.03 m,
        # NavSatFix bildirilen sigma 0.42 m (status=0, RTK YOK). Kontrol icin
        # ONEMLI olan kisa vadeli gurultudur; mutlak hata enu_to_local()
        # yakalamasiyla zaten sogurulur.
        self.declare_parameter('pos_variance', 0.04)     # (0.2 m)^2
        self.declare_parameter('yaw_variance', 0.0003)   # (~1 derece)^2

        self.lat0 = float(self.get_parameter('enu_lat0').value)
        self.lon0 = float(self.get_parameter('enu_lon0').value)
        self.gnss_timeout = float(self.get_parameter('gnss_timeout').value)
        self.pos_var = float(self.get_parameter('pos_variance').value)
        self.yaw_var = float(self.get_parameter('yaw_variance').value)

        # Yerel duzlem olcekleri (orijin enleminde bir kez hesaplanir).
        _p = math.radians(self.lat0)
        _s = math.sin(_p)
        _den = 1.0 - WGS84_E2 * _s * _s
        self._m_per_rad_lon = (WGS84_A / math.sqrt(_den)) * math.cos(_p)
        self._m_per_rad_lat = WGS84_A * (1.0 - WGS84_E2) / _den ** 1.5

        self.yaw = None
        self.yaw0 = None
        self.x0 = None
        self.y0 = None
        self.last_pos_t = None
        self._warned = False

        self.create_subscription(QuaternionStamped, '/filter/quaternion',
                                 self.quat_callback, 50)
        self.create_subscription(Vector3Stamped, '/filter/positionlla',
                                 self.pos_callback, 50)

        self.pub_local = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.pub_map = self.create_publisher(Odometry, '/odometry/filtered_map', 10)

        self.create_timer(1.0, self.health_check)

        self.get_logger().info(
            'xsens_odometry_node hazir (Xsens -> odometry DOGRUDAN, UKF YOK). '
            f'ENU orijini=({self.lat0:.7f}, {self.lon0:.7f})')

    # ------------------------------------------------------------- Callbacks
    def quat_callback(self, msg):
        q = msg.quaternion
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.yaw0 is None:
            self.yaw0 = self.yaw
            self.get_logger().info(
                f'Acilis yonu yakalandi: yaw0={math.degrees(self.yaw0):.2f} deg '
                '(/odometry/filtered yaw=0 bu yonu gosterir)')

    def pos_callback(self, msg):
        # Vector3Stamped: x=enlem, y=boylam, z=yukseklik (Xsens konvansiyonu)
        lat, lon = msg.vector.x, msg.vector.y
        x = math.radians(lon - self.lon0) * self._m_per_rad_lon
        y = math.radians(lat - self.lat0) * self._m_per_rad_lat

        self.last_pos_t = self.get_clock().now()

        if self.yaw is None:
            return                       # yon gelmeden poz yayinlanmaz
        if self.x0 is None:
            self.x0, self.y0 = x, y

        stamp = msg.header.stamp

        # --- MUTLAK (ENU) -------------------------------------------------
        self.pub_map.publish(
            self._make_odom('map', x, y, self.yaw, stamp))

        # --- YEREL (kontrol icin) -----------------------------------------
        # Acilis pozuna/yonune gore: UKF_local davranisinin taklidi.
        dyaw = math.atan2(math.sin(self.yaw - self.yaw0),
                          math.cos(self.yaw - self.yaw0))
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        dx, dy = x - self.x0, y - self.y0
        self.pub_local.publish(
            self._make_odom('odom', c * dx - s * dy, s * dx + c * dy,
                            dyaw, stamp))

    # -------------------------------------------------------------- Yardimci
    def _make_odom(self, frame_id, x, y, yaw, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = frame_id
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = float(x)
        o.pose.pose.position.y = float(y)
        o.pose.pose.position.z = 0.0
        o.pose.pose.orientation.z = math.sin(yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(yaw / 2.0)
        o.pose.covariance[0] = self.pos_var    # x
        o.pose.covariance[7] = self.pos_var    # y
        o.pose.covariance[35] = self.yaw_var   # yaw
        return o

    def health_check(self):
        """GNSS kesilirse bayat poz yayinlamaya DEVAM ETME - sessiz hata yok."""
        if self.last_pos_t is None:
            self.get_logger().warn(
                '/filter/positionlla hic gelmedi - xsens_mti_node calisiyor mu?',
                throttle_duration_sec=5.0)
            return
        age = (self.get_clock().now() - self.last_pos_t).nanoseconds * 1e-9
        if age > self.gnss_timeout:
            self.get_logger().error(
                f'/filter/positionlla {age:.1f} s bayat - odometri yayini DURDU. '
                'Park dugumleri odom gelmeyince hareket etmez (guvenli).',
                throttle_duration_sec=5.0)
            self._warned = True
        elif self._warned:
            self.get_logger().info('/filter/positionlla geri geldi.')
            self._warned = False


def main(args=None):
    rclpy.init(args=args)
    node = XsensOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
