from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'llm_vlm_brain_node'

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
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='gm_ubuntu',
    maintainer_email='gm_ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_chat_node = llm_vlm_brain_node.llm_chat_node:main',
            'vlm_describe_node = llm_vlm_brain_node.vlm_describe_node:main',
        ],
    },
)
