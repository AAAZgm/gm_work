import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('gm_web_dashboard')
    www_dir = os.path.join(pkg_dir, 'www')

    # 1. rosbridge WebSocket 服务器
    rosbridge_node = launch_ros.actions.Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}],
        output='screen'
    )

    # 2. web_video_server (MJPEG 流，端口 8080)
    web_video_node = launch_ros.actions.Node(
        package='web_video_server',
        executable='web_video_server',
        parameters=[{'port': 8888, 'address': '0.0.0.0'}],
        output='screen'
    )

    # 3. 摄像头管理节点
    camera_manager_node = launch_ros.actions.Node(
        package='gm_web_dashboard',
        executable='camera_manager_node',
        output='screen'
    )

    # 4. 静态文件 HTTP 服务器（Python 内置）
    # 用 Python 起一个简单 HTTP 服务器来提供 index.html
    http_server = launch.actions.ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000', '--directory', www_dir],
        output='screen'
    )

    return launch.LaunchDescription([
        rosbridge_node,
        web_video_node,
        camera_manager_node,
        http_server,
    ])
