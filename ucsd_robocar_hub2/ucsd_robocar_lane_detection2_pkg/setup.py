from setuptools import setup
import os
from glob import glob


package_name = 'ucsd_robocar_lane_detection2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = ucsd_robocar_lane_detection2_pkg.calibration_node:main',
            'lane_detection_node = ucsd_robocar_lane_detection2_pkg.lane_detection_node:main',
            'lane_guidance_node = ucsd_robocar_lane_detection2_pkg.lane_guidance_node:main'
        ],
    },
)

