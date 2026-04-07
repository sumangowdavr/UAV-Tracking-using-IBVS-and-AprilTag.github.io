from setuptools import setup
import os
from glob import glob

package_name = 'ros_uav_apriltag'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codex Agent',
    maintainer_email='you@example.com',
    description='ROS2 GPS-denied UAV exploration and AprilTag localization mission package for X500V2 + MAVROS.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager = ros_uav_apriltag.mission_manager:main',
            'apriltag_bridge = ros_uav_apriltag.apriltag_bridge:main',
            'search_planner = ros_uav_apriltag.search_planner:main',
            'tracking_controller = ros_uav_apriltag.tracking_controller:main',
        ],
    },
)
