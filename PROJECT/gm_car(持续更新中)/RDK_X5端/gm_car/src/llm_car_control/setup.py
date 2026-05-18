from setuptools import setup
import os
from glob import glob

package_name = "llm_car_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 安装配置文件
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # 安装启动文件
        (os.path.join("share", package_name, "launch"), glob("launch/*.py"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="your_email@example.com",
    description="ROS2智能小车大模型控制包（适配RDK官方YOLOv8）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_parser = llm_car_control.vision_parser:main",  # 替换原vision_node
            "llm_node = llm_car_control.llm_node:main",
            "control_node = llm_car_control.control_node:main"
        ],
    },
)