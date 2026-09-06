from setuptools import setup
import os
from glob import glob

package_name = 'robotaxi_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='GPS+IMU Localization for Robotaxi',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'gps_imu_localizer = robotaxi_localization.gps_imu_localizer:main',
            'test_data_publisher = robotaxi_localization.test_data_publisher:main',
            'sensor_monitor = robotaxi_localization.sensor_monitor:main',
            'wheel_odometry_node = robotaxi_localization.wheel_odometry_node:main',
            'xsens_odometry_node = robotaxi_localization.xsens_odometry_node:main',
        ],
    },
)