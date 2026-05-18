from setuptools import setup
import os
from glob import glob

package_name = 'voice_control_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 必须加这两行，把launch和config文件打包进去！
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Voice control car with LLM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asr_node = voice_control_car.asr_node:main',
            'llm_node = voice_control_car.llm_node:main',
            'control_node = voice_control_car.control_node:main',
            'tts_node = voice_control_car.tts_node:main',
        ],
    },
)
