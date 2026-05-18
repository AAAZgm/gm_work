from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包路径
    pkg_dir = get_package_share_directory("llm_car_control")
    config_path = os.path.join(pkg_dir, "config", "car_config.yaml")

    # 1. 启动官方YOLOv8节点（BPU加速）
    yolo8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, "launch", "yolo8_launch.py")),
        launch_arguments={}.items()
    )

    # 2. 视觉解析节点（解析官方YOLO结果）
    vision_parser_node = Node(
        package="llm_car_control",
        executable="vision_parser",
        name="vision_parser_node",
        parameters=[config_path],
        output="screen"
    )

    # 3. 大模型决策节点
    llm_node = Node(
        package="llm_car_control",
        executable="llm_node",
        name="llm_node",
        parameters=[config_path],
        output="screen"
    )

    # 4. 运动控制节点
    control_node = Node(
        package="llm_car_control",
        executable="control_node",
        name="control_node",
        parameters=[config_path],
        output="screen"
    )

    return LaunchDescription([
        yolo8_launch,
        vision_parser_node,
        llm_node,
        control_node
    ])