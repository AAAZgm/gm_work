#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import time

class PureASR(Node):
    def __init__(self):
        super().__init__('pure_asr')
        # 只发布识别结果，不做任何回复
        self.asr_pub = self.create_publisher(String, '/asr_text', 10)
        self.logger = self.get_logger()

        # 百度ASR密钥（你自己填）
        self.API_KEY = "T76dgpJ1rUiMoWTf2tce96AM"
        self.SECRET_KEY = "Ib4iYyIPL74WjDjyYvedjpOHVehz1zXJ"
        self.token = self.get_token()

        # 电脑IP（已填好你的192.168.137.17）
        self.pc_ip = "192.168.137.17"
        self.stream_url = f"http://{self.pc_ip}:12345/stream"

        self.audio_buffer = b''
        self.logger.info("✅ 纯ASR节点启动：仅语音转文字，无自动回复")

    def get_token(self):
        url = "https://aip.baidubce.com/oauth/2.0/token"
        data = {
            "grant_type": "client_credentials",
            "client_id": self.API_KEY,
            "client_secret": self.SECRET_KEY
        }
        try:
            r = requests.post(url, data=data, timeout=5)
            return r.json()["access_token"]
        except Exception as e:
            self.logger.error(f"❌ token获取失败: {e}")
            return None

    def recognize(self, pcm_data):
        if not self.token:
            return
        url = f"https://vop.baidu.com/server_api?cuid=rdk_x5&token={self.token}"
        headers = {"Content-Type": "audio/pcm; rate=16000"}
        try:
            r = requests.post(url, headers=headers, data=pcm_data, timeout=3)
            j = r.json()
            if j.get("err_no") == 0:
                text = j["result"][0].strip()
                if text:
                    self.logger.info(f"🗣️ 识别结果: {text}")
                    # 只发布文字，不做任何回复
                    self.asr_pub.publish(String(data=text))
        except Exception as e:
            pass

    def run(self):
        while rclpy.ok():
            try:
                with requests.get(self.stream_url, stream=True, timeout=5) as resp:
                    for chunk in resp.iter_content(chunk_size=1024):
                        self.audio_buffer += chunk
                        # 每2秒识别一次，避免刷屏
                        if len(self.audio_buffer) >= 16000 * 2 * 2:
                            self.recognize(self.audio_buffer)
                            self.audio_buffer = b''
            except Exception as e:
                self.logger.warn("⚠️ 音频流断开，重连中...")
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = PureASR()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
