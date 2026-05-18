import rclpy
from rclpy.node import Node
from hobot_dnn_msgs.msg import AIResult
import requests
import json

# ==================== 你的配置 ====================
ZHIPU_API_KEY = "06677cb763954ca198af337e03f999d2.1cwlf162lhvsRmYQ"
TTS_URL = "http://192.168.137.121:9999/tts"
# ===================================================

class YoloDescribeNode(Node):
    def __init__(self):
        super().__init__("yolo_zhipu_tts_node")

        # 订阅官方YOLO输出话题 ✅
        self.sub = self.create_subscription(
            AIResult,
            "/hobot_dnn_detection",
            self.yolo_callback,
            10
        )

        self.last_objects = ""
        self.get_logger().info("✅ 订阅官方YOLO成功，等待目标检测...")

    def yolo_callback(self, msg):
        """接收官方YOLO结果"""
        obj_list = []
        for obj in msg.objects:
            obj_list.append(f"{obj.class_name}")

        if not obj_list:
            return

        obj_str = "、".join(obj_list)
        if obj_str == self.last_objects:
            return
        self.last_objects = obj_str

        self.get_logger().info(f"👁️ YOLO检测：{obj_str}")

        # 让智谱生成自然描述
        desc = self.get_zhipu_desc(obj_str)
        self.get_logger().info(f"🗣️ 智谱描述：{desc}")

        # TTS朗读
        self.speak(desc)

    def get_zhipu_desc(self, obj_str):
        prompt = f"""你是一个智能小车。
请用一句非常简短、自然、口语化的话描述你看到了什么。
不要符号，不要格式，不要多余内容。

我看到了：{obj_str}
"""
        try:
            resp = requests.post(
                url="https://open.bigmodel.cn/api/paas/v4/chat/completions",
                headers={
                    "Authorization": f"Bearer {ZHIPU_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "glm-4-flash",
                    "messages": [{"role": "user", "content": prompt}]
                },
                timeout=5
            )
            return resp.json()["choices"][0]["message"]["content"]
        except:
            return f"我看到了{obj_str}"

    def speak(self, text):
        try:
            requests.post(TTS_URL, json={"text": text}, timeout=2)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloDescribeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()