from setuptools import find_packages, setup

package_name = 'motor_ops'

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
    maintainer='samartha-s-newrro',
    maintainer_email='work@newrro.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'Arjuna_cmd_vel_sub = motor_ops.Arjuna_cmd_vel_sub:main',
            'Arjuna_mec_cmd_vel_sub = motor_ops.Arjuna_mec_cmd_vel_sub:main',
            'Arjuna_Ticks_Pub = motor_ops.Arjuna_Ticks_Pub:main'
        ],
    },
)
