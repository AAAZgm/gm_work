from setuptools import setup

package_name = 'llm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='LLM-based intelligent car control with YOLO vision',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'llm_yolo_car = llm_control.llm_yolo_car:main',
        ],
    },
)
