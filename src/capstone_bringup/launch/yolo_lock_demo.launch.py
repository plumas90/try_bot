from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo_detector = Node(
        package='capstone_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
    )

    yolo_lock_drive = Node(
        package='capstone_motion',
        executable='yolo_lock_drive_node',
        name='yolo_lock_drive_node',
        output='screen',
        parameters=[{
            'linear_speed': 0.04,
            'stop_distance': 0.30,
            'max_drive_sec': 9.0,
            'hold_sec': 1.5,
        }],
    )

    return LaunchDescription([
        yolo_detector,
        yolo_lock_drive,
    ])
