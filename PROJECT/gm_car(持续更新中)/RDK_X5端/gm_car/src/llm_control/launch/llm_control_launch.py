from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_control',
            executable='llm_yolo_car',
            name='llm_yolo_car_node',
            output='screen'
        )
    ])
