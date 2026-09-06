#!/usr/bin/env python3
"""
Teker + direksiyon odometrisi (bisiklet modeli) — robot_localization için sürekli
hız kaynağı.

Araç CAN feedback'inden hız ve direksiyon açısını alıp bisiklet (Ackermann)
kinematik modeliyle body-frame hızları üretir ve nav_msgs/Odometry olarak yayınlar.
Konum İNTEGRALLENMEZ — sadece twist (vx, vyaw) verilir; konumu ukf_local üretir.

  vx   = fb_reelvehiclespeed_ms                    (m/s)
  delta = feedbacksteeringangle * steering_scale   (rad)  ← ölçek SAHADA kalibre
  vyaw = vx * tan(delta) / wheelbase               (rad/s)

Girdi:
  /beemobs/FB_VehicleSpeed        (smart_can_msgs/Fbvehiclespeed)
  /beemobs/FeedbackSteeringAngle  (smart_can_msgs/Feedbacksteeringangle)
Çıktı:
  /wheel/odometry                 (nav_msgs/Odometry — sadece twist anlamlı)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from smart_can_msgs.msg import Fbvehiclespeed, Feedbacksteeringangle


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # -------- Parametreler --------
        self.declare_parameter('wheelbase', 1.86)          # m (BEE1 dingil mesafesi)
        self.declare_parameter('steering_scale', 0.0174533)  # int8 -> rad (varsayım: derece)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('speed_topic', '/beemobs/FB_VehicleSpeed')
        self.declare_parameter('steering_topic', '/beemobs/FeedbackSteeringAngle')
        self.declare_parameter('vx_variance', 0.05)        # (m/s)^2
        self.declare_parameter('vyaw_variance', 0.05)      # (rad/s)^2

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steering_scale = float(self.get_parameter('steering_scale').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        speed_topic = self.get_parameter('speed_topic').value
        steering_topic = self.get_parameter('steering_topic').value
        self.vx_var = float(self.get_parameter('vx_variance').value)
        self.vyaw_var = float(self.get_parameter('vyaw_variance').value)

        # Son ölçümler
        self.vx = 0.0
        self.delta = 0.0

        # CAN feedback genelde best_effort
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Fbvehiclespeed, speed_topic, self.on_speed, qos)
        self.create_subscription(Feedbacksteeringangle, steering_topic, self.on_steer, qos)

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_odom)

        self.get_logger().info(
            f"Teker odometrisi aktif. wheelbase={self.wheelbase} m, "
            f"steering_scale={self.steering_scale}, {rate:.0f} Hz")

    def on_speed(self, msg: Fbvehiclespeed):
        # Gerçek hız (m/s)
        self.vx = float(msg.fb_reelvehiclespeed_ms)

    def on_steer(self, msg: Feedbacksteeringangle):
        # int8 teker açısı -> radyan (ölçek sahada kalibre)
        self.delta = float(msg.feedbacksteeringangle) * self.steering_scale

    def publish_odom(self):
        vyaw = self.vx * math.tan(self.delta) / self.wheelbase if self.wheelbase > 0 else 0.0

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Sadece twist anlamlı (konumu ukf üretir)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = vyaw

        # Kovaryans: pose bilinmiyor -> büyük; twist anlamlı
        # 6x6 satır-major: indeks 0=x,7=y,14=z,21=roll,28=pitch,35=yaw
        big = 1e6
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = big
        odom.twist.covariance[0] = self.vx_var     # vx
        odom.twist.covariance[35] = self.vyaw_var  # vyaw
        for i in (7, 14, 21, 28):                  # vy, vz, wx, wy güvenilmez
            odom.twist.covariance[i] = big

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
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
