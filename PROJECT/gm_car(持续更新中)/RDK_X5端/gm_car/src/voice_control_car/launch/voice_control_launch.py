from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('voice_control_car')
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')

    # 声明配置文件参数
    declare_config_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Full path to config file'
    )

    # ASR节点
    asr_node = Node(
        package='voice_control_car',
        executable='asr_node',
        name='asr_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # LLM节点
    llm_node = Node(
        package='voice_control_car',
        executable='llm_node',
        name='llm_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # 控制节点
    control_node = Node(
        package='voice_control_car',
        executable='control_node',
        name='control_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # TTS节点
    tts_node = Node(
        package='voice_control_car',
        executable='tts_node',
        name='tts_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        declare_config_cmd,
        asr_node,
        llm_node,
        control_node,
        tts_node
    ])
