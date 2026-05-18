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
    bringup_dir = get_package_share_directory('gm_robot_bringup')
    mono2d_dir = get_package_share_directory('mono2d_body_detection')
#    gesture_dir=get_package_share_directory('hand_gesture_detection')
    # ========== 路径拼接 ==========
    mono2d_launch_file = os.path.join(mono2d_dir, 'launch', 'mono2d_body_detection.launch.py')
#    gesture_detect=os.path.join(gesture_dir,'launch/hand_gesture_detection.launch.py')
    car_launch_path = os.path.join(bringup_dir, 'launch', 'bringup_car.py')

    # ========== 小车底层驱动 ==========
    car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(car_launch_path),
        launch_arguments={
            'use_cam': 'false'
        }.items()
    )
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
            'linear_velocity': 0.2,
            'angular_velocity': 1.0,
            'stop_move_ratio': 0.4,
            'activate_move_thr': 5,
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
        car_launch,
        follow_node,                  # 条件启动：非手势模式才有
        nogesture_launch,
    ])
