from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'gnss_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 設定ファイルとLaunchファイルをインストール対象に含める
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'websockets'],
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
