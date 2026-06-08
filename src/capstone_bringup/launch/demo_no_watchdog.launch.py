from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo_detector = Node(
        package='capstone_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
    )

    target_finder = Node(
        package='capstone_perception',
        executable='target_finder_node',
        name='target_finder_node',
        output='screen',
    )

    mission = Node(
        package='capstone_motion',
        executable='yolo_lidar_mission_node',
        name='yolo_lidar_mission_node',
        output='screen',
        parameters=[{
            'linear_speed': 0.04,
            'angular_gain': 0.0,
            'center_deadband': 9999.0,
        }],
    )

    return LaunchDescription([
        yolo_detector,
        target_finder,
        mission,
    ])
