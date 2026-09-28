from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fer_gripper_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='georg.katranis@gmail.com',
    maintainer_email='georg.katranis@gmail.com',
    description='Gripper server of the FER platform: MoveGripper, Grasp and Release '
                'through fer_interfaces, grasp state reported to the world model.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper_server = fer_gripper_server.gripper_server:main',
        ],
    },
)
