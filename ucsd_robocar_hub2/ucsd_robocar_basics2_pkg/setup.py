from setuptools import setup
import os
from glob import glob

package_name = 'ucsd_robocar_basics2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_camera_node = ucsd_robocar_basics2_pkg.sub_camera_node:main',
            'sub_lidar_node = ucsd_robocar_basics2_pkg.sub_lidar_node:main',
            'subpub_camera_actuator_node = ucsd_robocar_basics2_pkg.subpub_camera_actuator_node:main',
            'subpub_lidar_actuator_node = ucsd_robocar_basics2_pkg.subpub_lidar_actuator_node:main',
        ],
    },
)
