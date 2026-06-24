import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    imu_package_dir = get_package_share_directory('imu_ros2_device')
    bringup_dir = get_package_share_directory('gm_robot_bringup')
    laser_dir = get_package_share_directory('lslidar_driver')


    # 配置文件路径
    imu_filter_config = os.path.join(imu_package_dir, 'config', 'imu_filter_param.yaml')
    laser_yaml_path = os.path.join(laser_dir, 'params', 'lidar_uart_ros2', 'lsn10.yaml')
    cartographer_config = os.path.join(bringup_dir, 'config', 'cartographer.lua')
    default_rviz_path = os.path.join(bringup_dir, 'config', 'config.rviz')
    urdf2tf_path = os.path.join(bringup_dir, 'launch', 'gm_urdf2tf.py')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    
    if_use_sim_time=launch.actions.DeclareLaunchArgument(
    name='use_sim_time',default_value="False",
        description='Whether to use simulation time')#在 ROS 2 启动文件中声明一个名为 model 的启动arg，为它设置默认值，并添加描述信息
    if_use_cam=launch.actions.DeclareLaunchArgument(
        name='use_cam',default_value="False",
        description='Whether to use camera')#在 ROS 2 启动文件中声明一个名为 model 的启动arg，为它设置默认值，并添加描述信息
    if_use_rviz=launch.actions.DeclareLaunchArgument(
        name='use_rviz',default_value="False",
        description='Whether to launch rviz2')

    # URDF 发布
    urdf2tf_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(urdf2tf_path)
    )

    # ===== 节点定义 =====
    
    # 串口通信
    serial2ros2_node = launch_ros.actions.Node(
        package='serial2ros2',
        executable='serial2ros2_node',
        output='screen'
    )

    # 激光雷达
    laser_driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='log',
        emulate_tty=True,
        namespace='',
        parameters=[laser_yaml_path],
    )

    # IMU
    imu_node = launch_ros.actions.Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
    )

    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    # 里程计
    odom2tf_node = launch_ros.actions.Node(
        package='gm_robot_bringup',
        executable='odom2tf',
        output='screen'
    )

    # ===== Cartographer SLAM =====
    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='log',
        parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}],
        arguments=['-configuration_directory', os.path.dirname(cartographer_config),
                   '-configuration_basename', 'cartographer.lua'],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu/data'),
            ('odom', '/odometry/filtered'),  # 使用滤波后的里程计
        ]
    )

    # 地图发布，因为不是标准地图，需要转换
    cartographer_occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')},
                    {'resolution': 0.05},
                    {'publish_period_sec': 0.01}],  
    )

    ekf_node = launch_ros.actions.Node(
    package='robot_localization',
    executable='ekf_node',
    output='screen',
    parameters=[ekf_config],
    )
    # 键盘控制
    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
    )

    # RViz
    rviz_node = launch_ros.actions.Node(
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path]
    )
    
    cam_node=launch_ros.actions.Node(
    condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('use_cam')),
    package='usb_cam',
    executable='usb_cam_node',
    output="screen"
    )
    # 延迟启动
   # odom2tf_delay = launch.actions.TimerAction(period=1.0, actions=[odom2tf_node])
    cartographer_delay = launch.actions.TimerAction(period=2.0, actions=[cartographer_node])
    ekf_delay = launch.actions.TimerAction(period=1.0, actions=[ekf_node])
    return launch.LaunchDescription([
        serial2ros2_node,
        if_use_cam,
        if_use_rviz,
        if_use_sim_time,
        ekf_delay,
        laser_driver_node,
        imu_node,
        imu_filter_node,
    #    odom2tf_node,
        urdf2tf_launch,
        cartographer_delay,
        cartographer_occupancy_grid_node,
        teleop_node,
        cam_node,
        rviz_node
    ])
