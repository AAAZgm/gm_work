import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_tutorial_path=get_package_share_directory('gm_robot_1_description')
    
    default_model_path=urdf_tutorial_path+'/urdf/gm_robot_1/gm_robot_1.xacro'  #拼接
    

    action_declare_arg_model_path=launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_model_path),
        description='绝对路径')#在 ROS 2 启动文件中声明一个名为 model 的启动arg，为它设置默认值，并添加描述信息

    

    #在启动流程执行时，运行一个外部的系统终端命令，并将该命令的「标准输出（stdout）」作为返回值
    robot_description=launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')]),value_type=str)

    joint_state_publisher_node=launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    robot_state_publisher_node=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]#传给节点机器人信息
    )
   
    return launch.LaunchDescription([
        action_declare_arg_model_path,
        joint_state_publisher_node,
        robot_state_publisher_node
    ])
#args =  文件的参数，控制启动逻辑_命令行
#parameter = 节点的参数，配置节点行为