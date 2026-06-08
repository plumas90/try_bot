from setuptools import find_packages, setup

package_name = 'capstone_ocr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugim',
    maintainer_email='hugim@example.com',
    description='Capstone OCR package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ocr_reader_node = capstone_ocr.ocr_reader_node:main',
        ],
    },
)
