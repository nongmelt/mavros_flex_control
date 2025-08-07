from setuptools import setup
import os
from glob import glob

package_name = 'flex_sensor_reader'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='user@example.com',
    description='ROS2 package for reading flex sensor data via ADS1115',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flex_sensor_node = flex_sensor_reader.flex_sensor_node:main',
            'flex_process=flex_sensor_reader.flex_process:main',
        ],
    },
)