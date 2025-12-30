from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mks_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsyrek',
    maintainer_email='user@example.com',
    description='Minimal ROS2 package for table robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_table_map = mks_motor_control.generate_table_map:main',
            'initialize_robot_on_table = mks_motor_control.initialize_robot_on_table:main',
            'hybrid_localization = mks_motor_control.hybrid_localization:main',
        ],
    },
)
