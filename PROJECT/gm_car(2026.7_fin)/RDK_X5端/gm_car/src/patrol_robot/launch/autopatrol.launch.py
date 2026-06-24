import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
def generate_launch_description():
    autopatrol_dir = get_package_share_directory('patrol_robot')
    nav2_dir = get_package_share_directory('gm_navigation')
    bringup_dir = get_package_share_directory('gm_robot_bringup')
    # 在 bringup_dir 下面加一行：
    default_map_path = os.path.join(nav2_dir, 'maps', 'my_cartographer_map.yaml')

    # 默认配置文件路径
    default_config_path = os.path.join(autopatrol_dir, 'config', 'patrol_config.yaml')

    map_path = launch.actions.DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load'
    )

    declare_log_level = launch.actions.DeclareLaunchArgument(
    'log_level',
    default_value='WARN',
    description='Log level for patrol_node (DEBUG, INFO, WARN, ERROR)'
    )

    # ============ 声明可配置参数 ============
    # 使用方法: ros2 launch patrol_robot autopatrol.launch.py config:=/path/to/config.yaml
    
    declare_config_file = launch.actions.DeclareLaunchArgument(
        'config',
        default_value=default_config_path,
        description='Full path to patrol config yaml file'
    )
    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_dir, 'launch', 'navigation2.py')
    ),
    launch_arguments={
        'map': LaunchConfiguration('map'),
    }.items()
)

    carto_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(bringup_dir, 'launch', 'bringup_car_raw.py')
    ),
    launch_arguments={'use_rviz': 'false','use_cam': 'true'}.items() 
    )
    
    # ============ 巡航导航节点 ============
    patrol_node = launch_ros.actions.Node(
        package='patrol_robot',
        executable='patrol_node',
        parameters=[launch.substitutions.LaunchConfiguration('config')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    # ============ 语音播报节点 ============
    tts_node = launch_ros.actions.Node(
        package='tts_asr_node',
        executable='Tts_node',
        name='tts',
        output='screen'
        )

    # ============ 延迟启动巡航节点（等Nav2就绪）============
    delayed_patrol = TimerAction(
        period=3.0,
        actions=[patrol_node]
    )

    return launch.LaunchDescription([
        map_path,
        declare_config_file,
        carto_launch,
        nav2_launch,
        delayed_patrol,
        tts_node,
        declare_log_level
    ])
