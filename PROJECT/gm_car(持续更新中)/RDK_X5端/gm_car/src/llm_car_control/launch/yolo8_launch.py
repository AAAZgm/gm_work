from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # 必须缩进！所有代码都要在这个函数内部
    set_cam_type = SetEnvironmentVariable(
        name="CAM_TYPE",
        value="usb"  # MIPI摄像头改成 "mipi"
    )

    start_yolo8 = ExecuteProcess(
        cmd=[
            "ros2", "launch", "dnn_node_example", "dnn_node_example.launch.py",
            "dnn_example_config_file:=config/yolov8workconfig.json",
            "dnn_example_image_width:=640",
            "dnn_example_image_height:=640"
        ],
        output="screen",
        cwd="/opt/tros/humble/lib/dnn_node_example"
    )

    ld = LaunchDescription()
    ld.add_action(set_cam_type)
    ld.add_action(start_yolo8)
    return ld