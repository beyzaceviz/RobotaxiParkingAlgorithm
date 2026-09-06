import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'parking_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mertberke',
    maintainer_email='github-mberkedemrn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'parking_decision_and_control_node = parking_safety.parking_decision_and_control_node:main',
            'parking_mission_planner = parking_safety.parking_mission_planner:main',
            'parking_gps_zone_trigger = parking_safety.parking_gps_zone_trigger:main',
        ],
    },
)
