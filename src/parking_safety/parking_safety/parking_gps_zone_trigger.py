#!/usr/bin/env python3
"""Park Bolgesi GPS Tetikleyici (Global Gorev Katmani).

NEDEN VAR:
    Park algoritmasi (parking_decision_and_control_node + parking_mission_planner)
    TAMAMEN LOKAL calisir: cepleri LIDAR ile bulur, waypoint'leri odom (metre)
    uzerinden kurar. Icinde GPS/enlem/boylam KAVRAMI YOKTUR.

    Sahadan gamepad ile surulerek toplanan GPS waypoint'leri ise park BOLGESININ
    KONUMUNU tanimlar. Bu dugum o kopruyu kurar: canli GPS (NavSatFix) ile kayitli
    bolgeyi karsilastirir; arac bolgeye girince /parking_zone_reached uzerinden
    parking_decision_and_control_node'u UYANDIRIR (bkz. o dugumdeki
    STATE_BEKLE_GPS: GPS bolgesine girilene kadar arac tamamen hareketsiz kalir).

    Boylece GPS "nerede park edilecegini" soyler; LIDAR+odom ise "nasil park
    edilecegini" yurutur.

CIKTI:
    /parking_zone_reached (std_msgs/Bool) -> arac park bolgesinde mi?
    parking_decision_and_control_node bu topigi dogrudan dinler (require_gps_zone
    parametresiyle zorunlu/opsiyonel yapilabilir).

GIRDI:
    gps_topic (varsayilan /gnss, KD Doc 3.1/4.2 XSens GPS ciktisi) -> araca
    gore REMAP edilmeli. Aractaki gercek GPS topic adini `ros2 topic list`
    ile ogrenip ver.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


# Park bolgesi GPS waypoint'leri [enlem, boylam].
#
# 2026-07-28 GUNCELLEME: eski liste elle okunan tek atislik GPS gosterimleriydi
# ve gercek konumlardan 2.5-2.9 m sapiyordu. Yeni degerler araci ilgili noktada
# DURDURUP 20-40 s bag kaydi alarak, /filter/positionlla akisinin son %40'inin
# ortalamasindan cikarildi (cep_bagleri/). Tek sayili cepler dogrudan olculdu;
# cift sayililar 5 olculen cebe oturtulan en kucuk kareler dogrusundan
# ara-degerlendi (fit artigi max 0.095 m). Ayrintili gerekce:
# parking_mission_planner.py parking_slots yorumu.
#
# Ilk kayit park GIRISI (cep_bagleri/baslangıc_0727_1756) - geofence'in asil
# tetigi burasi; kalanlar 9 cep merkezi.
DEFAULT_ZONE_WAYPOINTS = [
    [40.7901667, 29.5092143],   # park girisi / baslangic  (OLCULDU)
    [40.7903840, 29.5091450],   # cep 1  (OLCULDU)
    [40.7903671, 29.5091655],   # cep 2  (ara-degerleme)
    [40.7903497, 29.5091867],   # cep 3  (OLCULDU)
    [40.7903325, 29.5092056],   # cep 4  (ara-degerleme)
    [40.7903152, 29.5092245],   # cep 5  (OLCULDU)
    [40.7902972, 29.5092448],   # cep 6  (ara-degerleme)
    [40.7902803, 29.5092637],   # cep 7  (OLCULDU)
    [40.7902640, 29.5092869],   # cep 8  (ara-degerleme)
    [40.7902477, 29.5093100],   # cep 9  (OLCULDU)
]


def haversine_m(lat1, lon1, lat2, lon2):
    """Iki GPS noktasi arasindaki mesafe [metre]."""
    R = 6371000.0  # Dunya yaricapi [m]
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


class ParkingGpsZoneTrigger(Node):
    def __init__(self):
        super().__init__('parking_gps_zone_trigger')

        # --- Parametreler -----------------------------------------------------
        # Gercek XSens GPS topic'i: /gnss (sensor_msgs/NavSatFix)
        self.declare_parameter('gps_topic', '/gnss')
        # Waypoint'ler duz liste olarak verilir: [lat0, lon0, lat1, lon1, ...]
        flat_default = [c for wp in DEFAULT_ZONE_WAYPOINTS for c in wp]
        self.declare_parameter('zone_waypoints', flat_default)
        # Bolgeye "girildi" sayilan yaricap [m]
        self.declare_parameter('trigger_radius_m', 8.0)
        # Bolge merkezine ne kadar yaklasinca ARM edilecegi vs. cikinca DISARM
        self.declare_parameter('release_radius_m', 15.0)  # histerezis

        gps_topic = self.get_parameter('gps_topic').value
        flat = self.get_parameter('zone_waypoints').value
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.trigger_radius = float(self.get_parameter('trigger_radius_m').value)
        self.release_radius = float(self.get_parameter('release_radius_m').value)

        # Bolge merkezi (waypoint'lerin ortalamasi)
        self.zone_lat = sum(w[0] for w in self.waypoints) / len(self.waypoints)
        self.zone_lon = sum(w[1] for w in self.waypoints) / len(self.waypoints)

        # --- Abonelik / yayin -------------------------------------------------
        self.gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self.gps_callback, 10)
        self.zone_pub = self.create_publisher(Bool, '/parking_zone_reached', 10)

        self.in_zone = False

        self.get_logger().info(
            f'GPS bolge tetikleyici hazir. Topic="{gps_topic}", '
            f'merkez=({self.zone_lat:.7f}, {self.zone_lon:.7f}), '
            f'yaricap={self.trigger_radius} m, wp={len(self.waypoints)}')

    def gps_callback(self, msg: NavSatFix):
        # En yakin waypoint'e mesafe (bolge bir cizgi/rota oldugu icin
        # merkeze degil, EN YAKIN noktaya bakmak daha dogru)
        dist = min(
            haversine_m(msg.latitude, msg.longitude, wlat, wlon)
            for wlat, wlon in self.waypoints)

        # Histerezisli bolge tespiti (girip-cikma titremesini onler)
        if not self.in_zone and dist <= self.trigger_radius:
            self.in_zone = True
            self.get_logger().info(
                f'>>> PARK BOLGESINE GIRILDI (mesafe={dist:.1f} m) <<<')
            self.publish_zone(True)
        elif self.in_zone and dist > self.release_radius:
            self.in_zone = False
            self.get_logger().info(
                f'Park bolgesinden cikildi (mesafe={dist:.1f} m)')
            self.publish_zone(False)

    def publish_zone(self, value: bool):
        m = Bool()
        m.data = value
        self.zone_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingGpsZoneTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
