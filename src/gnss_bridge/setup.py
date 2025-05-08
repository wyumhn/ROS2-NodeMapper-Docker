from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'gnss_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS2 to WebSocket GNSS bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gnss_bridge = gnss_bridge.gnss_bridge:main',
        ],
    },
)
