import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tts_asr_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='zgm353059@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'Asr_node=tts_asr_node.Asr_node:main',
            'Tts_node=tts_asr_node.Tts_node:main',
            'manual_trigger=tts_asr_node.manual_trigger:main',
            'remote_start=tts_asr_node.remote_start:main'
        ],
    },
)
