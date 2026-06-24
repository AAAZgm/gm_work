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
    nav2_bringup_dir = get_package_share_directory('gm_navigation')
    bringup_dir = get_package_share_directory('gm_robot_bringup')
    

    map_path = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map file to load'
    )


# SLAM launch（关闭 RViz）
    slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(bringup_dir, 'launch', 'bringup_slamtool_raw.py')
    ),
    launch_arguments={'use_rviz': 'False'}.items()  # 关闭 SLAM 的 RViz
    )

# Nav2 launch（保持 RViz）
    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_bringup_dir, 'launch', 'navigation2_nomap.py')
    ),
    launch_arguments={
            'map': LaunchConfiguration('map')
        }.items(),
    # 默认 use_rviz=true，所以会启动 RViz
    )

    # ========== 4. 探索节点（延迟启动）==========
    exploration_node = Node(
        package='gm_exploration',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[os.path.join(exploration_dir, 'config', 'exploration_params.yaml')]
    )
    
    # 延迟启动探索节点（等待其他节点就绪）
    exploration_delay = TimerAction(
        period=2.0,
        actions=[exploration_node]
    )
    
    return LaunchDescription([
        map_path,
        slam_launch,
        nav2_launch,
        exploration_delay,
    ])
