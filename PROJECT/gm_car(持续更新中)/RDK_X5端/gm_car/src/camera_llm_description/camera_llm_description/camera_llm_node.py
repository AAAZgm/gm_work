import rclpy
from rclpy.node import Node
import cv2
import requests

class CameraLLMNode(Node):
    def __init__(self):
        super().__init__('camera_llm_node')
        self.get_logger().info('✅ 摄像头 + AI 描述启动')
        self.last = ''
        self.timer = self.create_timer(3.0, self.work)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def get_desc(self):
        api_key = "你的智谱KEY"
        prompt = "你是智能小车，用一句简短口语描述摄像头看到的画面，只说物体和场景。"
        try:
            r = requests.post("https://open.bigmodel.cn/api/paas/v4/chat/completions",
                headers={"Authorization": f"Bearer {api_key}"},
                json={"model": "glm-4-flash","messages": [{"role":"user","content":prompt}]},
                timeout=8)
            return r.json()["choices"][0]["message"]["content"]
        except:
            return "获取描述失败"

    def work(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        txt = self.get_desc()
        if txt != self.last:
            self.get_logger().info(f'👁️ 看到：{txt}')
            self.last = txt

def main(args=None):
    rclpy.init(args=args)
    node = CameraLLMNode()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
