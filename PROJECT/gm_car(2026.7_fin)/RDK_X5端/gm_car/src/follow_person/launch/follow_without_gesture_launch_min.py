import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ========== 包路径 ==========

    mono2d_dir = get_package_share_directory('mono2d_body_detection')
#    gesture_dir=get_package_share_directory('hand_gesture_detection')
    # ========== 路径拼接 ==========
    mono2d_launch_file = os.path.join(mono2d_dir, 'launch', 'mono2d_body_detection.launch.py')
#    gesture_detect=os.path.join(gesture_dir,'launch/hand_gesture_detection.launch.py')



    # nogesture_launch=IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(
    #                 get_package_share_directory('hand_gesture_detection'),
    #                 'launch/hand_gesture_detection.launch.py'))
    #     )

    nogesture_launch=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mono2d_launch_file),
                launch_arguments={
            'smart_topic': '/hobot_mono2d_body_detection',
            'mono2d_body_pub_topic': '/hobot_mono2d_body_detection'
        }.items()
        )

    follow_node = Node(
        package='follow_person',
        executable='tracking_no_gesture_node',
        output='screen',
        parameters=[{#可以融合以1决定是否启用手势
            'ai_msg_topic':'/hobot_mono2d_body_detection',
            'image_width': 640,
            'image_height': 480,
            'linear_velocity': LaunchConfiguration('linear_velocity'),
            'angular_velocity': LaunchConfiguration('angular_velocity'),
            'stop_move_ratio': LaunchConfiguration('stop_move_ratio'),
            'activate_move_thr': LaunchConfiguration('activate_move_thr'),
        }],
        # arguments=['--ros-args', '--log-level', 'warn']
    )

    # # ========== 手势跟随节点（手势模式）==========
    # tracking_with_gesture_node = Node(
    #     package='follow_person',
    #     executable='tracking_with_gesture_node',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_gesture'))
    # )

    return launch.LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('stop_move_ratio', default_value='0.7',#越大越近
            description='太近停止阈值'),
        DeclareLaunchArgument('activate_move_thr', default_value='2',
            description='激活帧数阈值'),
        DeclareLaunchArgument('linear_velocity', default_value='0.2',
            description='最大前进速度 m/s'),
        DeclareLaunchArgument('angular_velocity', default_value='1.0',
            description='最大旋转角速度 rad/s'),
        # 节点
        follow_node,
        nogesture_launch,
    ])
