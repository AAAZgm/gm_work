import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gm_4dof'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'dof_control = gm_4dof.dof_control:main',
            'vision_display = gm_4dof.vision_display:main',
            'arm_keyboard = gm_4dof.arm_keyboard:main',
            'arm_joint_keyboard = gm_4dof.arm_joint_keyboard:main',
        ],
    },
)
