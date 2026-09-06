# ~/robotaxi_ws/src/robotaxi_localization/robotaxi_localization/test_data_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
import math
import time

class TestDataPublisher(Node):
    def __init__(self):
        super().__init__('test_data_publisher')
        
        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gnss', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Test timers
        self.gps_timer = self.create_timer(0.1, self.publish_gps)  # 10Hz GPS
        self.imu_timer = self.create_timer(0.0025, self.publish_imu)  # 400Hz IMU
        
        # Test trajectory parameters
        self.time_start = time.time()
        
        # Robotaxi test coordinates (from your GeoJSON)
        self.center_lat = 40.7903314
        self.center_lon = 29.50896659
        self.radius = 0.0001  # ~10m radius in degrees
        
        self.get_logger().info('Test data publisher başlatıldı - Circular trajectory')
    
    def publish_gps(self):
        """Mock GPS data - circular trajectory around start point"""
        elapsed = time.time() - self.time_start
        angle = elapsed * 0.2  # 0.2 rad/s angular velocity
        
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        
        # Circular GPS trajectory around center point
        msg.latitude = self.center_lat + self.radius * math.cos(angle)
        msg.longitude = self.center_lon + self.radius * math.sin(angle)
        msg.altitude = 100.0
        
        # GPS status - GOOD FIX
        msg.status.status = 0  # STATUS_FIX 
        msg.status.service = 1  # SERVICE_GPS
        
        # Position covariance (1.5m standard deviation)
        msg.position_covariance = [0.0] * 9
        msg.position_covariance[0] = 2.25  # x variance (1.5^2)
        msg.position_covariance[4] = 2.25  # y variance (1.5^2)  
        msg.position_covariance[8] = 4.0   # z variance (2.0^2)
        msg.position_covariance_type = 2   # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_pub.publish(msg)
    
    def publish_imu(self):
        """Mock IMU data - consistent with GPS motion"""
        elapsed = time.time() - self.time_start
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientation - rotating with same angular velocity as GPS
        yaw = elapsed * 0.2  # Same as GPS angular motion
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw/2)
        msg.orientation.w = math.cos(yaw/2)
        
        # Angular velocity - constant rotation
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.2  # 0.2 rad/s
        
        # Linear acceleration - centripetal + gravity
        radius_m = 10.0  # ~10m radius
        linear_velocity = 2.0  # m/s tangential velocity
        centripetal_accel = (linear_velocity ** 2) / radius_m
        
        msg.linear_acceleration.x = centripetal_accel * math.cos(yaw + math.pi/2)
        msg.linear_acceleration.y = centripetal_accel * math.sin(yaw + math.pi/2)
        msg.linear_acceleration.z = 9.81  # Gravity
        
        # Covariance matrices (simplified)
        msg.orientation_covariance = [0.0] * 9
        msg.orientation_covariance[0] = 0.01  # Small uncertainty
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.1   # More uncertain in yaw
        
        msg.angular_velocity_covariance = [0.0] * 9
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001
        
        msg.linear_acceleration_covariance = [0.0] * 9
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.01
        
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = TestDataPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()