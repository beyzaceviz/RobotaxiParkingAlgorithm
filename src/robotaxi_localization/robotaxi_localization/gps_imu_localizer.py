# ~/robotaxi_ws/src/robotaxi_localization/robotaxi_localization/gps_imu_localizer.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from pyproj import Proj
from scipy.spatial.transform import Rotation
import time

# ROS 2 messages
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# TF2
import tf2_ros
from tf2_ros import TransformBroadcaster


class GPSIMULocalizer(Node):
    def __init__(self):
        super().__init__('gps_imu_localizer')
        
        # Declare all parameters from config
        self._declare_parameters()
        
        # Load parameters
        self._load_parameters()
        
        # State variables
        self.current_pose = np.zeros(3)  # [x, y, yaw]
        self.current_velocity = np.zeros(3)  # [vx, vy, vyaw]
        
        # UTM projection
        self.utm_proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', datum='WGS84')
        self.origin_set = False
        self.origin_utm = None
        self.origin_gps = None
        
        # Time tracking
        self.last_gps_time = None
        self.last_gps_pose = None
        self.last_imu_time = None
        
        # Quality monitoring
        self.gps_msg_count = 0
        self.imu_msg_count = 0
        self.last_status_time = time.time()
        
        # Initialize subscribers and publishers
        self._setup_communication()
        
        # Timer for high-frequency publishing
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        
        # Status monitoring timer
        if hasattr(self, 'report_interval'):
            self.status_timer = self.create_timer(self.report_interval, self.status_callback)
        
        self.get_logger().info(f'GPS-IMU Localizer başlatıldı (Config-based)')
        self.get_logger().info(f'UTM Zone: {self.utm_zone}, Rate: {self.publish_rate}Hz')
        if hasattr(self, 'use_zed_imu'):
            imu_type = "ZED2" if self.use_zed_imu else "XSENS"
            self.get_logger().info(f'IMU Type: {imu_type}')

    def _declare_parameters(self):
        """Declare all parameters from config file"""
        # Core parameters
        self.declare_parameter('utm_zone', 35)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('tf_broadcast', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # Topic parameters
        self.declare_parameter('gps_topic', '/gnss')
        self.declare_parameter('imu_topic', '/imu/data')
        
        # GPS parameters
        self.declare_parameter('gps_timeout', 2.0)
        self.declare_parameter('min_gps_accuracy', 5.0)
        
        # IMU parameters
        self.declare_parameter('imu_timeout', 1.0)
        self.declare_parameter('use_zed_imu', False)
        
        # Filter parameters
        self.declare_parameter('position_covariance', [2.25, 2.25, 4.0])
        self.declare_parameter('orientation_covariance', [0.01, 0.01, 0.1])
        
        # Quality parameters
        self.declare_parameter('min_gps_rate', 8.0)
        self.declare_parameter('min_imu_rate', 100.0)
        self.declare_parameter('max_data_age', 0.5)
        self.declare_parameter('report_interval', 5.0)

    def _load_parameters(self):
        """Load all parameters"""
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.tf_broadcast = self.get_parameter('tf_broadcast').get_parameter_value().bool_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
        self.gps_timeout = self.get_parameter('gps_timeout').get_parameter_value().double_value
        self.min_gps_accuracy = self.get_parameter('min_gps_accuracy').get_parameter_value().double_value
        
        self.imu_timeout = self.get_parameter('imu_timeout').get_parameter_value().double_value
        self.use_zed_imu = self.get_parameter('use_zed_imu').get_parameter_value().bool_value
        
        self.position_covariance = self.get_parameter('position_covariance').get_parameter_value().double_array_value
        self.orientation_covariance = self.get_parameter('orientation_covariance').get_parameter_value().double_array_value
        
        self.min_gps_rate = self.get_parameter('min_gps_rate').get_parameter_value().double_value
        self.min_imu_rate = self.get_parameter('min_imu_rate').get_parameter_value().double_value
        self.max_data_age = self.get_parameter('max_data_age').get_parameter_value().double_value
        self.report_interval = self.get_parameter('report_interval').get_parameter_value().double_value

    def _setup_communication(self):
        """Setup subscribers and publishers"""
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers with configurable topics
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, sensor_qos)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vehicle_odom', 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/pose_with_covariance', 10)
        
        # TF broadcaster
        if self.tf_broadcast:
            self.tf_broadcaster = TransformBroadcaster(self)

    def gps_callback(self, msg):
        """GPS callback with quality checking"""
        self.gps_msg_count += 1
        current_time = self.get_clock().now()
        
        # GPS quality checks
        if msg.status.status < 0:
            self.get_logger().warn('GPS fix yok!')
            return
        
        # GPS accuracy check (if available in message)
        if hasattr(msg, 'position_covariance') and msg.position_covariance[0] > 0:
            gps_accuracy = np.sqrt(msg.position_covariance[0])
            if gps_accuracy > self.min_gps_accuracy:
                self.get_logger().warn(f'GPS accuracy poor: {gps_accuracy:.2f}m')
                return
            
        try:
            # GPS koordinatlarını UTM'e çevir
            utm_x, utm_y = self.utm_proj(msg.longitude, msg.latitude)
            
            # İlk GPS verisiyle origin belirle
            if not self.origin_set:
                self.origin_utm = (utm_x, utm_y)
                self.origin_gps = (msg.latitude, msg.longitude)
                self.origin_set = True
                self.get_logger().info(
                    f'Origin set: {msg.latitude:.7f}, {msg.longitude:.7f}')
                return
            
            # Local koordinatlara çevir
            local_x = utm_x - self.origin_utm[0]
            local_y = utm_y - self.origin_utm[1]
            
            # GPS pozisyonunu direkt kullan
            self.current_pose[0] = local_x
            self.current_pose[1] = local_y
            
            # Velocity hesapla (GPS tabanlı)
            if self.last_gps_time is not None and self.last_gps_pose is not None:
                dt = (current_time - self.last_gps_time).nanoseconds / 1e9
                if dt > 0.01:  # 100Hz'den hızlı değilse
                    self.current_velocity[0] = (local_x - self.last_gps_pose[0]) / dt
                    self.current_velocity[1] = (local_y - self.last_gps_pose[1]) / dt
            
            self.last_gps_pose = (local_x, local_y)
            self.last_gps_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'GPS processing error: {e}')

    def imu_callback(self, msg):
        """IMU callback with type-specific processing"""
        self.imu_msg_count += 1
        current_time = self.get_clock().now()
        
        try:
            # IMU type-specific processing
            if self.use_zed_imu:
                # ZED2 IMU processing (if needed)
                pass
            
            # Quaternion'dan yaw açısını çıkar
            r = Rotation.from_quat([
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w
            ])
            euler = r.as_euler('xyz')
            self.current_pose[2] = euler[2]  # yaw
            
            # Angular velocity
            self.current_velocity[2] = msg.angular_velocity.z
            
            self.last_imu_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'IMU processing error: {e}')

    def status_callback(self):
        """Status monitoring callback"""
        current_time = time.time()
        dt = current_time - self.last_status_time
        
        gps_rate = self.gps_msg_count / dt
        imu_rate = self.imu_msg_count / dt
        
        # Rate quality check
        gps_ok = gps_rate >= self.min_gps_rate
        imu_ok = imu_rate >= self.min_imu_rate
        
        status_msg = f'GPS: {gps_rate:.1f}Hz{"✓" if gps_ok else "✗"}, IMU: {imu_rate:.1f}Hz{"✓" if imu_ok else "✗"}'
        
        if gps_ok and imu_ok:
            self.get_logger().info(status_msg)
        else:
            self.get_logger().warn(status_msg)
        
        # Reset counters
        self.gps_msg_count = 0
        self.imu_msg_count = 0
        self.last_status_time = current_time

    def timer_callback(self):
        """High-frequency publishing timer"""
        if not self.origin_set:
            return
            
        current_time = self.get_clock().now()
        self.publish_pose(current_time)
        self.publish_odometry(current_time)
        
        if self.tf_broadcast:
            self.broadcast_tf(current_time)

    def publish_pose(self, timestamp):
        """Publish current pose with configurable covariance"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp.to_msg()
        pose_msg.header.frame_id = self.map_frame
        
        pose_msg.pose.position.x = self.current_pose[0]
        pose_msg.pose.position.y = self.current_pose[1]
        pose_msg.pose.position.z = 0.0
        
        # Yaw'dan quaternion'a çevir
        r = Rotation.from_euler('z', self.current_pose[2])
        quat = r.as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
        
        # Pose with covariance - use config values
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = pose_msg.header
        pose_cov_msg.pose.pose = pose_msg.pose
        
        # Configurable covariance
        pose_cov_msg.pose.covariance[0] = self.position_covariance[0]    # x variance
        pose_cov_msg.pose.covariance[7] = self.position_covariance[1]    # y variance
        pose_cov_msg.pose.covariance[14] = self.position_covariance[2]   # z variance
        pose_cov_msg.pose.covariance[21] = self.orientation_covariance[0] # roll variance
        pose_cov_msg.pose.covariance[28] = self.orientation_covariance[1] # pitch variance
        pose_cov_msg.pose.covariance[35] = self.orientation_covariance[2] # yaw variance
        
        self.pose_cov_pub.publish(pose_cov_msg)

    def publish_odometry(self, timestamp):
        """Publish odometry with configurable covariance"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Position
        odom_msg.pose.pose.position.x = self.current_pose[0]
        odom_msg.pose.pose.position.y = self.current_pose[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        r = Rotation.from_euler('z', self.current_pose[2])
        quat = r.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.angular.z = self.current_velocity[2]
        
        # Configurable covariance
        odom_msg.pose.covariance[0] = self.position_covariance[0]     # x
        odom_msg.pose.covariance[7] = self.position_covariance[1]     # y  
        odom_msg.pose.covariance[35] = self.orientation_covariance[2] # yaw
        
        self.odom_pub.publish(odom_msg)

    def broadcast_tf(self, timestamp):
        """TF transform broadcast et"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.current_pose[0]
        t.transform.translation.y = self.current_pose[1]
        t.transform.translation.z = 0.0
        
        r = Rotation.from_euler('z', self.current_pose[2])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    localizer = GPSIMULocalizer()
    
    try:
        rclpy.spin(localizer)
    except KeyboardInterrupt:
        pass
    finally:
        localizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()