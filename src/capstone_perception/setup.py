from setuptools import setup

package_name = 'capstone_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugim',
    maintainer_email='hugim@example.com',
    description='Capstone perception package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'target_finder_node = capstone_perception.target_finder_node:main',
            'yolo_detector_node = capstone_perception.yolo_detector_node:main',
        ],
    },
)
