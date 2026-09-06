from setuptools import setup
import os
from glob import glob

package_name = 'hiway_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liliwang',
    maintainer_email='liliwang@todo.com',
    description='Receive raw encoder/servo UDP from ESP32 and publish wheel odometry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_udp_node = hiway_odom.odom_udp_node:main',
        ],
    },
)
