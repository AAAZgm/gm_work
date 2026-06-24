import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # 获取与拼接默认路径
    gm_navigation2_dir = get_package_share_directory('gm_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    rviz_config_path_default = os.path.join(
        gm_navigation2_dir, 'config', 'config.rviz')

    nav_param_path_default = os.path.join(
        gm_navigation2_dir, 'config', 'nav2_params_nomap.yaml')

    # 声明配置参数
    if_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    map_path = launch.actions.DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map file to load'
    )

    param_path = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=nav_param_path_default,
        description='Full path to param file to load'
    )

    # ========== 模式1：有地图模式（bringup_launch.py）==========
    nav2_with_map = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items(),
    )
    # ========== RViz2 节点 ==========
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path_default],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        if_use_sim_time,
        map_path,
        param_path,
        nav2_with_map,      # 有地图时启动
        rviz2_node
    ])
