from setuptools import find_packages, setup
import os
import glob
package_name = 'gm_web_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # 安装 www 目录
        (os.path.join('share', package_name, 'www'),
         glob.glob('gm_web_dashboard/www/*')),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.py')),
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
            'camera_manager_node = gm_web_dashboard.camera_manager_node:main',
        ],
    },
)
