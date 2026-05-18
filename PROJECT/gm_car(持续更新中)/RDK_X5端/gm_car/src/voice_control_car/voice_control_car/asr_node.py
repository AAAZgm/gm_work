#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class ASRNode(Node):
    def __init__(self):
        super().__init__("asr_node")
        # 1. 正确声明ROS2日志（修复致命错误）
        self.logger = self.get_logger()
        
        # 2. 声明话题参数
        self.declare_parameter("asr_topic", "/asr_output")
        self.declare_parameter("user_cmd_topic", "/car/user_cmd")
        self.asr_topic = self.get_parameter("asr_topic").value
        self.user_cmd_topic = self.get_parameter("user_cmd_topic").value

        # 3. 初始化发布器
        self.pub_asr = self.create_publisher(String, self.asr_topic, 10)
        self.pub_cmd = self.create_publisher(String, self.user_cmd_topic, 10)

        # 4. 初始化语音识别器
        self.recognizer = sr.Recognizer()
        
        # 5. 适配RDK X5音频：手动指定麦克风，避免ALSA警告
        try:
            # 直接使用默认麦克风，跳过无效设备检测
            self.microphone = sr.Microphone()
            self.logger.info("🎤 麦克风初始化成功")
        except Exception as e:
            self.logger.error(f"❌ 麦克风打开失败: {str(e)}")
            raise

    def run(self):
        with self.microphone as source:
            # 延长环境噪声校准，适配嵌入式环境
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.logger.info("✅ 环境校准完成，等待说话...")
            
            while rclpy.ok():
                try:
                    # 延长监听超时，避免误判超时
                    audio = self.recognizer.listen(
                        source, 
                        timeout=10, 
                        phrase_time_limit=10
                    )
                    # 中文语音识别
                    text = self.recognizer.recognize_google(audio, language="zh-CN")
                    self.logger.info(f"🗣️ 识别结果: {text}")
                    # 发布识别结果
                    self.pub_asr.publish(String(data=text))
                    self.pub_cmd.publish(String(data=text))
                except sr.WaitTimeoutError:
                    # 静默超时，不打印错误，避免刷屏
                    continue
                except sr.UnknownValueError:
                    # 无法识别语音，静默跳过
                    continue
                except sr.RequestError as e:
                    self.logger.error(f"❌ 网络/识别服务异常: {str(e)}")
                except Exception as e:
                    self.logger.error(f"❌ ASR运行异常: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
