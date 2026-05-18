#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from flask import Flask, Response
import threading
import time

# ================== Flask 网页流服务 ==================
app = Flask(__name__)
current_frame = None  # 全局变量，存储最新的带检测框的画面

# 生成 MJPEG 流
def generate_stream():
    global current_frame
    while True:
        if current_frame is not None:
            # 转成 JPEG 字节流
            _, jpeg = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = jpeg.tobytes()
            # 按照 MJPEG 格式返回
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.03)  # 30fps 控制

# 网页首页
@app.route('/')
def index():
    return """
    <html>
        <head><title>YOLO 实时检测画面</title></head>
        <body>
            <h1>智能小车 YOLO 视觉检测</h1>
            <img src="/video_feed" width="800" />
        </body>
    </html>
    """

# 视频流接口
@app.route('/video_feed')
def video_feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# 启动 Flask 服务（后台线程）
def run_flask():
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

# ================== ROS2 YOLO 节点 ==================
class YoloVisionNode(Node):
    def __init__(self):
        super().__init__("gm_yolo_vision_node")
        self.bridge = CvBridge()
        
        # 加载 YOLOv8n 模型
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("✅ YOLOv8 + Flask 网页流 节点启动")

        # 订阅摄像头话题（和你的 usb_cam 完全匹配）
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        # 发布检测结果话题（给大模型/TTS 用）
        self.pub_objects = self.create_publisher(String, "/yolo_objects", 10)
        self.pub_objects_dist = self.create_publisher(String, "/yolo_objects_with_dist", 10)

        # 防重复刷屏
        self.last_txt = ""
        self.last_dist_txt = ""

        # ================== 单目测距参数 ==================
        self.FOCAL_LENGTH = 600         # 相机焦距（通用值）
        self.KNOWN_WIDTH = {            # 物体真实宽度（cm）
            "bottle":    6.5,
            "cup":       8.0,
            "cell phone":7.2,
            "person":    40.0,
            "chair":     45.0,
        }
        # ==================================================

    def calculate_dist(self, class_name, pixel_width):
        """单目测距核心函数"""
        if class_name not in self.KNOWN_WIDTH:
            return None
        return round((self.KNOWN_WIDTH[class_name] * self.FOCAL_LENGTH) / pixel_width, 1)

    def image_callback(self, msg):
        global current_frame
        try:
            # 1. ROS 图像转 OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            display_img = cv_img.copy()

            # 2. YOLO 推理
            results = self.model(cv_img, conf=0.5, verbose=False)
            
            obj_list = []
            dist_list = []

            # 3. 画检测框 + 测距 + 标注
            for r in results:
                for box in r.boxes:
                    cls = self.model.names[int(box.cls[0])]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w = x2 - x1  # 计算框宽度

                    # 测距
                    dist = self.calculate_dist(cls, w)

                    # 画绿色检测框
                    cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # 标注文字（名称+距离）
                    if dist:
                        text = f"{cls} {dist}cm"
                        dist_list.append(text)
                    else:
                        text = cls
                        dist_list.append(cls)
                    obj_list.append(cls)

                    cv2.putText(display_img, text, (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 4. 更新全局画面（给 Flask 流用）
            current_frame = display_img.copy()

            # 5. 发布检测结果话题（去重+防重复）
            if obj_list:
                unique_obj = list(set(obj_list))
                obj_str = "、".join(unique_obj)
                unique_dist = list(set(dist_list))
                dist_str = "、".join(unique_dist)

                if obj_str != self.last_txt or dist_str != self.last_dist_txt:
                    self.pub_objects.publish(String(data=obj_str))
                    self.pub_objects_dist.publish(String(data=dist_str))
                    self.last_txt = obj_str
                    self.last_dist_txt = dist_str
                    self.get_logger().info(f"👀 检测到: {dist_str}")

        except Exception as e:
            # 捕获所有异常，避免节点崩溃
            self.get_logger().error(f"❌ 图像回调异常: {str(e)}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=None)
    node = YoloVisionNode()
    
    # 启动 Flask 后台线程
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    node.get_logger().info("🌐 Flask 网页流已启动，访问 http://RDK_IP:8080 查看画面")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("✅ 节点正常退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()