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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Arduino scripts
            'US_pub = arjuna.scripts.Arjuna_arduino.US_pub:main',
            'US_sub = arjuna.scripts.Arjuna_arduino.US_sub:main',
            
            # OpenCV scripts
            'camera_pub = arjuna.scripts.Arjuna_opencv.Arjuna_camera_pub:main',
            'camera_sub = arjuna.scripts.Arjuna_opencv.Arjuna_camera_sub:main',
            'QR_recog = arjuna.scripts.Arjuna_opencv.Arjuna_QR_recog:main',
            'simple_qr_rec = arjuna.scripts.Arjuna_opencv.simple_qr_rec:main',
            'QR_tracking = arjuna.scripts.Arjuna_opencv.Arjuna_QR_tracking:main',
            'docking_controller = arjuna.scripts.Arjuna_opencv.Arjuna_docking_controller:main',
            'qr_detector = arjuna.scripts.Arjuna_opencv.Arjuna_QR_detector:main',
            'qr_controller = arjuna.scripts.Arjuna_opencv.Arjuna_QR_Controller:main',
            'qr_recog_ct = arjuna.scripts.Arjuna_opencv.Arjuna_QR_recog_for_CT:main',
            
            # Navigation scripts
            'go_to_point_terminal = arjuna.scripts.Arjuna_navigation.Arjuna_go_to_point_terminal:main',
            'go_to_point_rviz = arjuna.scripts.Arjuna_navigation.Arjuna_go_to_point_rviz:main',
            'multi_point_terminal = arjuna.scripts.Arjuna_navigation.Arjuna_multi_point_terminal:main',
            'multi_point_rviz = arjuna.scripts.Arjuna_navigation.Arjuna_multi_point_rviz:main',
            'obstacle_avoidance = arjuna.scripts.Arjuna_navigation.Arjuna_obstacle_avoidance:main',
            
            # Voice scripts
            'speech_recognition = arjuna.scripts.Arjuna_voice.Arjuna_SR_Script:main',
            'voice_control = arjuna.scripts.Arjuna_voice.Arjuna_voice_control:main',
            'simple_voice = arjuna.scripts.Arjuna_voice.Arjuna_simple_voice:main',
            'voice_feedback = arjuna.scripts.Arjuna_voice.Arjuna_voice_feedback:main',
            
            # Web scripts
            'mecanum_control = arjuna.scripts.Arjuna_web.Mecanum_control_for_webpage:main',
            'torch_control = arjuna.scripts.Arjuna_web.Torch:main',
            'voltage_pub = arjuna.scripts.Arjuna_web.voltage_to_webpage:main',
            'web_server = arjuna.scripts.Arjuna_web.Arjuna_web_server:main',
            'api_server = arjuna.scripts.Arjuna_web.Arjuna_api_server:main',
            'web_control = arjuna.scripts.Arjuna_web.Arjuna_web_control:main',
            
            # CPU/RAM monitoring
            'cpu_ram_pub = arjuna.scripts.Arjuna_cpu_ram.cpu_ram:main',
            
            # Ticks publisher
            'ticks_pub = arjuna.scripts.Ticks_Publisher:main',
        ],
    },
)