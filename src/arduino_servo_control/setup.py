from setuptools import setup
import os
from glob import glob

package_name = 'arduino_servo_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 interface for Arduino servo control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_control = arduino_servo_control.servo_control:main',
            'keyboard_publisher = arduino_servo_control.keyboard_publisher:main',
            'sequence_publisher = arduino_servo_control.sequence_publisher:main',
        ],
    },
) 