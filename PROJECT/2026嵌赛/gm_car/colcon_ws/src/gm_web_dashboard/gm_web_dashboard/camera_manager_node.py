#!/usr/bin/env python3
"""摄像头管理节点 - 通过 ROS2 服务控制 usb_cam_node 的启停"""
import subprocess
import signal
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class CameraManagerNode(Node):
    def __init__(self):
        super().__init__('camera_manager_node')

        self.cam_process = None
        self.cam_running = False

        self.srv = self.create_service(
            SetBool, 'camera_enable', self.callback
        )
        self.get_logger().info('摄像头管理节点已启动，等待服务调用...')

    def callback(self, request, response):
        if request.data:
            # 开启摄像头
            if self.cam_running and self.cam_process is not None:
                response.success = True
                response.message = '摄像头已在运行'
                return response
            try:
                self.cam_process = subprocess.Popen(
                    ['ros2', 'run', 'usb_cam', 'usb_cam_node'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )
                self.cam_running = True
                response.success = True
                response.message = '摄像头已启动'
                self.get_logger().info(f'摄像头已启动 (pid={self.cam_process.pid})')
            except Exception as e:
                response.success = False
                response.message = f'启动失败: {e}'
                self.get_logger().error(f'启动摄像头失败: {e}')
        else:
            # 关闭摄像头
            if not self.cam_running or self.cam_process is None:
                response.success = True
                response.message = '摄像头未在运行'
                return response
            try:
                os.killpg(os.getpgid(self.cam_process.pid), signal.SIGTERM)
                self.cam_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.cam_process.pid), signal.SIGKILL)
            except Exception:
                pass
            self.cam_process = None
            self.cam_running = False
            response.success = True
            response.message = '摄像头已关闭'
            self.get_logger().info('摄像头已关闭')
        return response

    def destroy_node(self):
        # 节点退出时清理摄像头进程
        if self.cam_process is not None:
            try:
                os.killpg(os.getpgid(self.cam_process.pid), signal.SIGTERM)
                self.cam_process.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.cam_process.pid), signal.SIGKILL)
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
