from glob import glob
from setuptools import setup

package_name = 'capstone_bringup'

setup(
    name=package_name,
    version='0.0.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugim',
    maintainer_email='hugim@example.com',
    description='Capstone launch package',
    license='MIT',
)
