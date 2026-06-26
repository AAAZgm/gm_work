#!/usr/bin/env python3
"""
自主探索建图启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    exploration_dir = get_package_share_directory('gm_exploration')
    
    exploration_node = Node(
        package='gm_exploration',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[os.path.join(exploration_dir, 'config', 'exploration_params.yaml')]
    )
    

    return LaunchDescription([
        exploration_node
    ])
