from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo_detector = Node(
        package='capstone_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
    )

    mission = Node(
        package='capstone_motion',
        executable='yolo_lidar_mission_node',
        name='yolo_lidar_mission_node',
        output='screen',
        parameters=[{
            'linear_speed': 0.04,
            'stop_distance': 0.30,
            'max_drive_sec': 8.0,
            'stop_hold_sec': 1.5,
            'force_drive': True,
        }],
    )

    return LaunchDescription([
        yolo_detector,
        mission,
    ])
