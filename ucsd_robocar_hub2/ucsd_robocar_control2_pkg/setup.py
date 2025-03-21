from setuptools import setup
import os
from glob import glob

package_name = 'ucsd_robocar_control2_pkg'
controller_submodule = str(package_name + "/controller_submodule")
state_estimate_submodule = str(package_name + "/state_estimate_submodule")

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, controller_submodule, state_estimate_submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'controller_submodule'), glob('controller_submodule/*.py')),
        (os.path.join('share', package_name, 'state_estimate_submodule'),glob('state_estimate_submodule/*.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.csv'))
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
            'lqr_node = ucsd_robocar_control2_pkg.lqr_node:main',
            'lqg_node = ucsd_robocar_control2_pkg.lqg_node:main',
            'lqg_w_node = ucsd_robocar_control2_pkg.lqg_w_node:main',
            'mpc_node = ucsd_robocar_control2_pkg.mpc_node:main',
            'pid_node = ucsd_robocar_control2_pkg.pid_node:main',
            'pid_gps_node = ucsd_robocar_control2_pkg.pid_gps_node:main',
            'pid_gps_calibration_node = ucsd_robocar_control2_pkg.pid_gps_calibration_node:main',
            'pid_e_node = ucsd_robocar_control2_pkg.pid_e_node:main',
            'pid_llh_node = ucsd_robocar_control2_pkg.pid_llh_node:main',
            'pid_servo_node = ucsd_robocar_control2_pkg.pid_servo_node:main'
        ],
    },
)
