from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SenseVoice ASR节点（离线语音识别）
        Node(
            package='custom_sensevoice_ros2',
            executable='sensevoice_asr_node',
            name='sensevoice_asr_node',
            output='screen'
        ),
        # 小车控制节点
        Node(
            package='custom_sensevoice_ros2',
            executable='car_control_node',
            name='car_control_node',
            output='screen'
        )
    ])
