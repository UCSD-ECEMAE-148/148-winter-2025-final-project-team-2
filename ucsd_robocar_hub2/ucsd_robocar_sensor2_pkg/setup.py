from setuptools import setup
import os
from glob import glob


package_name = 'ucsd_robocar_sensor2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json'))
    ],
    install_requires=['setuptools','pynmea2'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_node = ucsd_robocar_sensor2_pkg.webcam_node:main',
            'ublox_gps_node = ucsd_robocar_sensor2_pkg.ublox_gps_node:main'
        ],
    },
)
