from setuptools import find_packages, setup

package_name = 'arjuna_pub_sub'

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
            'Publisher = arjuna_pub_sub.Publisher:main',
            'Subscriber = arjuna_pub_sub.Subscriber:main',
            'US_Pub = arjuna_pub_sub.US_Pub:main',
            'US_Sub = arjuna_pub_sub.US_Sub:main'
        ],
    },
)
