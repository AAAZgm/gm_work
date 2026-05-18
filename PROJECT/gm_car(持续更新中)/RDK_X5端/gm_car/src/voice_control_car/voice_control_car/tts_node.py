#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class TTSNode(Node):
    def __init__(self):
        super().__init__("tts_node")
        # 从配置文件加载参数
        self.declare_parameter("tts_topic", "/car/tts_play")
        self.tts_topic = self.get_parameter("tts_topic").value

        # 初始化TTS引擎
        self.engine = pyttsx3.init()
        # 设置中文语音（适配espeak）
        self.engine.setProperty('rate', 150)  # 语速
        self.engine.setProperty('volume', 1.0)  # 音量

        # 订阅器
        self.sub_tts = self.create_subscription(
            String, self.tts_topic, self.tts_callback, 10
        )

        self.get_logger().info("🔊 TTS语音播报节点启动成功")

    def tts_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f"📢 播报内容：{text}")
        self.engine.say(text)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
