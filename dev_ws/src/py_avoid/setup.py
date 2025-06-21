from setuptools import find_packages, setup

package_name = 'py_avoid'

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
    maintainer='colin',
    maintainer_email='colinc131@gmail.com',
    description='About obstacle avoidance in ROS2, based on mid360 lidar data',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = py_avoid.talker:main',
            'odom = py_avoid.odom:main'
        ],
    },
)
