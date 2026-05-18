from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():

    package_path = get_package_share_path('imu_ros2_device')

    imu_filter_config = os.path.join(              
        package_path,
        'config',
        'imu_filter_param.yaml'
    )
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'ybimu.rviz')

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node,
        device_node,
        imu_filter_node,
    ])
