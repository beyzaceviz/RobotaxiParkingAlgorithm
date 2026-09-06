import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class SensorQualityMonitor(Node):
    def __init__(self):
        super().__init__('sensor_quality_monitor')
        
        # Stats tracking
        self.gps_count = 0
        self.imu_count = 0
        self.last_gps_time = None
        self.last_imu_time = None
        self.gps_status_counts = {}
        
        # QoS
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribers 
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gnss', self.gps_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos)
            
        # Status report timer
        self.report_timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info('Sensor Quality Monitor başlatıldı')
    
    def gps_callback(self, msg):
        self.gps_count += 1
        self.last_gps_time = time.time()
    
    def imu_callback(self, msg):
        self.imu_count += 1
        self.last_imu_time = time.time()
    
    def report_status(self):
        gps_rate = self.gps_count / 5.0 if self.gps_count > 0 else 0
        imu_rate = self.imu_count / 5.0 if self.imu_count > 0 else 0
        
        self.get_logger().info(f'GPS: {gps_rate:.1f}Hz, IMU: {imu_rate:.1f}Hz')
        
        # Reset counters
        self.gps_count = 0
        self.imu_count = 0

def main(args=None):
    rclpy.init(args=args)
    monitor = SensorQualityMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()