from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_uav_apriltag')
    mission_params = os.path.join(pkg_share, 'config', 'mission.yaml')

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14557'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('camera_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),

        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{'fcu_url': fcu_url, 'gcs_url': gcs_url}],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='vio_to_mavros',
            arguments=['/vio/pose', '/mavros/vision_pose/pose'],
            output='screen',
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            remappings=[('/image_rect', camera_image_topic), ('/camera_info', camera_info_topic)],
            output='screen',
        ),

        Node(package='ros_uav_apriltag', executable='search_planner', parameters=[mission_params], output='screen'),
        Node(package='ros_uav_apriltag', executable='apriltag_bridge', parameters=[mission_params], output='screen'),
        Node(package='ros_uav_apriltag', executable='tracking_controller', parameters=[mission_params], output='screen'),
        Node(package='ros_uav_apriltag', executable='mission_manager', parameters=[mission_params], output='screen'),
    ])
