from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # micro-ROS Agent 节点（串口连接微控制器）
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["udp4", "--port", "8888"],
            output='screen',
        ),

        # 机械臂控制节点
        Node(
            package='gm_4dof',
            executable='vision_display',
            name='vision_display',
            output='screen',
        ),
                Node(
            package='gm_4dof',
            executable='dof_control',
            name='dof_control',
            output='screen',
        ),
    ])
