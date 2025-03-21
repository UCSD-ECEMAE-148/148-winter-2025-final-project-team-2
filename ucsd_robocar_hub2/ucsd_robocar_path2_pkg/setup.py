from setuptools import setup
import os
from glob import glob

package_name = 'ucsd_robocar_path2_pkg'
submodule = str(package_name +"/path_submodule")

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "param"), glob('param/*.yaml')),
        (os.path.join('share', package_name, "paths"), glob('paths/*.csv')),
        (os.path.join('share', package_name, "config"), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'path_submodule'), glob('path_submodule/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sisaha',
    maintainer_email='omadityasiddharth123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_provider_node = ucsd_robocar_path2_pkg.path_provider_node:main',
            'tube_follower_node = ucsd_robocar_path2_pkg.tube_follower_node:main',
            'curve_localizer_node = ucsd_robocar_path2_pkg.curve_localizer_node:main',
            'gps_path_provider_node = ucsd_robocar_path2_pkg.gps_path_provider_node:main',
            'gps_error_node = ucsd_robocar_path2_pkg.gps_error_node:main'
        ],
    },
)
