from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arjuna'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('arjuna/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('arjuna/config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('arjuna/rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('arjuna/maps/*')),
        (os.path.join('share', package_name, 'webpage'), glob('arjuna/webpage/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Newrro Tech',
    maintainer_email='info@newrro.in',
    description='Arjuna autonomous robot ROS 2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            
            # OpenCV scripts
            'camera_pub = arjuna.scripts.Arjuna_opencv.Arjuna_camera_pub:main',
            'camera_sub = arjuna.scripts.Arjuna_opencv.Arjuna_camera_sub:main',
            'QR_Recog = arjuna.scripts.Arjuna_opencv.Arjuna_QR_Rec:main',
            'QR_Tracking = arjuna.scripts.Arjuna_opencv.Arjuna_QR_Tracking:main',
            'QR_CT = arjuna.scripts.Arjuna_opencv.Arjuna_QR_CT:main',
            
            # Navigation scripts
            'go_to_point_terminal = arjuna.scripts.Arjuna_navigation.Arjuna_go_to_point_terminal:main',
            'obstacle_avoidance = arjuna.scripts.Arjuna_navigation.Arjuna_obstacle_avoidance:main',
            
            # CPU/RAM monitoring
            'cpu_ram_pub = arjuna.scripts.Arjuna_cpu_ram.cpu_ram:main',
        ],
    },
)