import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    imu_package_dir = get_package_share_directory('imu_ros2_device')
    bringup_dir = get_package_share_directory('gm_robot_bringup')
    laser_dir=get_package_share_directory('lslidar_driver')
    slamtool_dir=get_package_share_directory('slam_toolbox')

    slamtool_params = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    imu_filter_config = os.path.join(              
        imu_package_dir,
        'config',
        'imu_filter_param.yaml'
    )
    default_rviz_path=bringup_dir+'/config/config.rviz'  #拼接
    laser_yaml_path = os.path.join(laser_dir, 'params','lidar_uart_ros2', 'lsn10.yaml')#同上
    urdf2tf_path=os.path.join(bringup_dir,'launch','gm_urdf2tf.py')
    slamtool_path=os.path.join(slamtool_dir, 'launch', 'online_async_launch.py')


    if_use_sim_time=launch.actions.DeclareLaunchArgument(
        name='use_sim_time',default_value="False",
        description='Whether to use simulation time')#在 ROS 2 启动文件中声明一个名为 model 的启动arg，为它设置默认值，并添加描述信息
    if_use_cam=launch.actions.DeclareLaunchArgument(
        name='use_cam',default_value="False",
        description='Whether to use camera')#在 ROS 2 启动文件中声明一个名为 model 的启动arg，为它设置默认值，并添加描述信息
    if_use_rviz=launch.actions.DeclareLaunchArgument(
        name='use_rviz',default_value="False",
        description='Whether to launch rviz2')

    urdf2tf_launch=launch.actions.IncludeLaunchDescription(#在一个 launch 文件中调用/包含另一个 launch 文件。
        launch.launch_description_sources.PythonLaunchDescriptionSource(urdf2tf_path)
    )

    slam_launch= launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(slamtool_path),
        launch_arguments={
            'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
            'slam_params_file': slamtool_params  # 添加参数文件
    }.items()
    )


    imu_node = launch_ros.actions.Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
    )

    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    teleop_node = launch_ros.actions.Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    name='teleop_keyboard',
    output='screen',
    prefix='xterm -e',  # xterm：启动一个新终端窗口 -e：后面跟要执行的命令
    #'gnome-terminal --',  # ← Ubuntu 默认终端 -- 表示后面的内容是命令，不是终端的参数。
    )


    odom2tf_node=launch_ros.actions.Node(
    package='gm_robot_bringup',
    executable='odom2tf',
    output="screen"#日志输出到终端
    )

    serial2ros2_node=launch_ros.actions.Node(
    package='serial2ros2',
    executable='serial2ros2_node',
    output="screen"
    )

    laser_driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='log',#别both
                                emulate_tty=True,
                                namespace='',
                                parameters=[laser_yaml_path],
                                )
    
    rviz_node=launch_ros.actions.Node(
    condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('use_rviz')),
    package='rviz2',
    executable='rviz2',
    output='screen',
    arguments=['-d', default_rviz_path]#传给命令行
    )

    cam_node=launch_ros.actions.Node(
    condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('use_cam')),
    package='usb_cam',
    executable='usb_cam_node',
    output="screen"
    )

    ekf_node = launch_ros.actions.Node(
    package='robot_localization',
    executable='ekf_node',
    output='screen',
    parameters=[ekf_config],
    )
  #  odom2tf_delay=launch.actions.TimerAction(period=1.0,actions=[odom2tf_node])  
    slam_delay=launch.actions.TimerAction(period=2.0,actions=[slam_launch])  
    ekf_delay=launch.actions.TimerAction(period=1.0,actions=[ekf_node]) 
    return launch.LaunchDescription([
        serial2ros2_node,
        if_use_sim_time,
        if_use_cam,
        if_use_rviz,
        imu_node,
        imu_filter_node,
        ekf_delay,
        laser_driver_node,
    #    odom2tf_node,
        urdf2tf_launch,
        slam_delay,
        cam_node,
        teleop_node,
        rviz_node,
    ])