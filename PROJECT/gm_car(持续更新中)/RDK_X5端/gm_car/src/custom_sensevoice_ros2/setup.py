from setuptools import setup
import os
from glob import glob

package_name = 'custom_sensevoice_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Custom SenseVoice offline ASR for RDK X5 car control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensevoice_asr_node = custom_sensevoice_ros2.sensevoice_asr_node:main',
            'car_control_node = custom_sensevoice_ros2.car_control_node:main',
            'master_controller = custom_sensevoice_ros2.master_controller:main',
        ],
    },
)
