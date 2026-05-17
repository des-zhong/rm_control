from glob import glob
import os
from setuptools import setup

package_name = 'rm_grip_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm_control maintainers',
    maintainer_email='zhuhaier1992@163.com',
    description='HSV/contour based visual grasp-success detector for RoboMaster EP gripper camera.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grip_vision_detector = rm_grip_vision.grip_vision_detector:main',
            'hsv_tuner = rm_grip_vision.hsv_tuner:main',
        ],
    },
)
