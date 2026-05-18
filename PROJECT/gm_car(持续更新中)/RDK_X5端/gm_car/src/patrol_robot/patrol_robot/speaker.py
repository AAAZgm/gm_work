# 导入ROS2 Python核心库
import rclpy
from rclpy.node import Node
from autopatrol_interface.srv import SpeachText
import subprocess


class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        
        # 创建ROS2服务
        self.srv = self.create_service(SpeachText, 'speach_text', self.speach_text_callback)
        self.get_logger().info('语音服务已启动')

    def speach_text_callback(self, request, response):
        self.get_logger().info(f"接收到文字：{request.text}")
        
        try:
            # 使用 espeak-ng 命令行工具（更稳定）
            subprocess.run([
                'espeak-ng',
                '-v', 'zh',           # 中文语音
                '-s', '150',          # 语速
                '-p', '50',           # 音调
                '-a', '100',          # 音量
                request.text
            ], check=True)
            response.result = True
        except Exception as e:
            self.get_logger().error(f'语音播报失败: {e}')
            response.result = False
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Speaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
