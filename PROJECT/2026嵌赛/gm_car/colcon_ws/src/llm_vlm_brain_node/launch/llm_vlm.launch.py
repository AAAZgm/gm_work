import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('gm_robot_bringup')
    nav_dir = get_package_share_directory('gm_navigation')

    carto_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_cartographer_raw.py')
        ),
        launch_arguments={'use_rviz': 'False','use_cam': LaunchConfiguration('use_cam')}.items()  # 关闭 SLAM 的 RViz
        )

# Nav2 launch（保持 RViz）
    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav_dir, 'launch', 'navigation2_nomap.py')
    )
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_cam', default_value='False'),
        DeclareLaunchArgument('llm_url', default_value='http://10.114.172.135:8080'),
        DeclareLaunchArgument('vlm_url', default_value='http://10.114.172.135:8080'),

        Node(
            package='llm_vlm_brain_node',
            executable='llm_chat_node',
            name='llm_chat_node',
            output='screen',
            parameters=[{
                'llm_url': LaunchConfiguration('llm_url'),
                'model_name': 'qwen3',
                'temperature': 0.7,
                'max_tokens': 512,
            }],
        ),
        Node(
            package='llm_vlm_brain_node',
            executable='vlm_describe_node',
            name='vlm_describe_node',
            output='screen',
            parameters=[{
                'vlm_url': LaunchConfiguration('vlm_url'),
            }],
        ),
        Node(
        package='tts_asr_node',
        executable='Tts_node',
        name='tts',
        output='screen'
        ),
        nav2_launch,
        carto_launch,
    ])
