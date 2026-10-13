from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fer_world_model'

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
    description='World model of the FER platform: objects built from detections, '
                'served through fer_interfaces.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_model_server = fer_world_model.world_model_server:main',
            'mock_perception = fer_world_model.mock_perception_node:main',
        ],
    },
)
