from setuptools import setup

package_name = 'capstone_motion'

setup(
    name=package_name,
    version='0.0.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugim',
    maintainer_email='hugim@example.com',
    description='YOLO target mission with LiDAR stop and odom return home',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_lidar_mission_node = capstone_motion.yolo_lidar_mission_node:main',
            'yolo_lock_drive_node = capstone_motion.yolo_lock_drive_node:main',
            'force_demo_node = capstone_motion.force_demo_node:main',
            'yolo_lidar_approach_node = capstone_motion.yolo_lidar_approach_node:main',
        ],
    },
)
