import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _config_yaml():
    """Return path to config.yaml: workspace root (dev) or install share (installed)."""
    ws_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')
    dev_path = os.path.normpath(os.path.join(ws_root, 'config.yaml'))
    if os.path.exists(dev_path):
        return dev_path
    share = get_package_share_directory('capstone_bringup')
    return os.path.join(share, 'config', 'config.yaml')


def generate_launch_description():
    cfg = _config_yaml()

    return LaunchDescription([
        # --- Frequently changed at runtime (CLI override) ---
        DeclareLaunchArgument('target_class', default_value='laptop'),
        DeclareLaunchArgument('model_path', default_value='models/yolov8s.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.40'),
        DeclareLaunchArgument('process_every_n_frames', default_value='3'),
        DeclareLaunchArgument('linear_speed', default_value='0.18'),
        DeclareLaunchArgument('stop_distance', default_value='0.55'),
        DeclareLaunchArgument('ocr_timeout_sec', default_value='4.0'),
        DeclareLaunchArgument('ocr_interval_sec', default_value='0.5'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('return_home', default_value='true'),
        DeclareLaunchArgument('use_ocr_next_target', default_value='true'),
        DeclareLaunchArgument('mission_sequence', default_value='laptop'),

        Node(
            package='capstone_motion',
            executable='yolo_lidar_mission_node',
            name='yolo_lidar_mission_node',
            output='screen',
            parameters=[
                ParameterFile(cfg, allow_substs=True),
                {
                    'target_class': LaunchConfiguration('target_class'),
                    'model_path': LaunchConfiguration('model_path'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'stop_distance': LaunchConfiguration('stop_distance'),
                    'ocr_timeout_sec': LaunchConfiguration('ocr_timeout_sec'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'return_home': LaunchConfiguration('return_home'),
                    'use_ocr_next_target': LaunchConfiguration('use_ocr_next_target'),
                    'mission_sequence': LaunchConfiguration('mission_sequence'),
                },
            ],
        ),

        Node(
            package='capstone_ocr',
            executable='ocr_reader_node',
            name='ocr_reader_node',
            output='screen',
            parameters=[
                ParameterFile(cfg, allow_substs=True),
                {
                    'image_topic': LaunchConfiguration('image_topic'),
                    'ocr_interval_sec': LaunchConfiguration('ocr_interval_sec'),
                },
            ],
        ),
    ])
