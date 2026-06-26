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

#    mono2d_dir = get_package_share_directory('mono2d_body_detection')
    gesture_dir=get_package_share_directory('hand_gesture_detection')
    # ========== 路径拼接 ==========
#    mono2d_launch_file = os.path.join(mono2d_dir, 'launch', 'mono2d_body_detection.launch.py')
    gesture_detect=os.path.join(gesture_dir,'launch/hand_gesture_detection.launch.py')

    gesture_launch=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gesture_detect)
        )

    # nogesture_launch=IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(mono2d_launch_file),
    #             launch_arguments={
    #         'smart_topic': '/hobot_mono2d_body_detection',
    #         'mono2d_body_pub_topic': '/hobot_mono2d_body_detection'
    #     }.items()
    #     )

    follow_node = Node(
        package='follow_person',
        executable='tracking_with_gesture_node',
        output='screen',
        parameters=[{
            "ai_msg_topic": "/hobot_hand_gesture_detection",
            "track_serial_lost_num_thr": 100,
            "move_step": 0.5,
            "rotate_step": 0.5
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
        follow_node,                  # 条件启动：非手势模式才有
        gesture_launch,
    ])
