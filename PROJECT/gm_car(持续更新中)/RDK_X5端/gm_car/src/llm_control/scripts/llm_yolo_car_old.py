#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import os
# ========== 新增这一行 ==========
from ultralytics import YOLO
# ================================

# 绝对路径（无需修改）
CONFIG_PATH = "/home/sunrise/gm_car/llm_control/config/config.yaml"

# 加载配置
with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

# ========== 新增：拼接模型完整绝对路径 ==========
# 从配置文件读取模型目录+文件名，拼接成完整路径
cfg["vit_model_full_path"] = os.path.join(cfg["model_dir"], cfg["vit_model_name"])
cfg["llm_model_full_path"] = os.path.join(cfg["model_dir"], cfg["llm_model_name"])

class LLMYOLOSCarControl(Node):
    def __init__(self):
        super().__init__("llm_yolo_car_node")
        
        # 1. 初始化YOLO（CPU模式）
        self.yolo = YOLO("yolov8n.pt").to("cpu")
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # 2. ROS2订阅/发布（绝对话题）
        self.sub_img = self.create_subscription(
            Image, cfg["camera_topic"], self.img_callback, 10
        )
        self.sub_cmd = self.create_subscription(
            String, cfg["user_cmd_topic"], self.cmd_callback, 10
        )
        self.pub_vel = self.create_publisher(
            Twist, cfg["cmd_vel_topic"], 10
        )
        self.pub_res = self.create_publisher(
            String, cfg["result_topic"], 10
        )
        
        self.get_logger().info("✅ LLM+YOLO小车节点启动成功")

    def img_callback(self, msg):
        """接收相机帧"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"相机帧转换失败：{e}")

    def yolo_detect(self, target=None):
        """YOLO检测"""
        if self.latest_frame is None:
            return "❌ 未获取到相机帧"
        
        results = self.yolo(self.latest_frame, conf=cfg["yolo_conf"], device="cpu")
        classes = [results[0].names[int(c)] for c in results[0].boxes.cls]
        
        if target:
            count = classes.count(target)
            return f"✅ 检测到 {count} 个「{target}」"
        else:
            unique_classes = list(set(classes))
            return f"✅ 检测到目标：{unique_classes}"

    def cmd_callback(self, msg):
        """处理用户指令"""
        cmd = msg.data.strip()
        self.get_logger().info(f"\n📢 收到指令：{cmd}")
        twist = Twist()
        result = ""

        # 运动指令
        if "前进" in cmd:
            twist.linear.x = 0.2
            self.pub_vel.publish(twist)
            result = "✅ 执行：前进（线速度0.2）"
        elif "后退" in cmd:
            twist.linear.x = -0.2
            self.pub_vel.publish(twist)
            result = "✅ 执行：后退（线速度-0.2）"
        elif "左转" in cmd:
            twist.angular.z = 0.5
            self.pub_vel.publish(twist)
            result = "✅ 执行：左转（角速度0.5）"
        elif "右转" in cmd:
            twist.angular.z = -0.5
            self.pub_vel.publish(twist)
            result = "✅ 执行：右转（角速度-0.5）"
        elif "停止" in cmd:
            self.pub_vel.publish(Twist())
            result = "✅ 执行：停止"
        # 检测指令
        elif "检测" in cmd:
            if "人" in cmd:
                result = self.yolo_detect("person")
            elif "车" in cmd:
                result = self.yolo_detect("car")
            elif "障碍物" in cmd:
                result = self.yolo_detect("person")
            else:
                result = self.yolo_detect()
        # 图像描述指令（触发hobot）
        elif "图片" in cmd or "描述" in cmd or "有啥" in cmd:
            result = "✅ 已触发图像描述，请查看hobot_llamacpp终端输出"
        # 未知指令
        else:
            result = "❌ 未知指令，支持：前进/后退/左转/右转/停止/检测人/检测车/图片有啥"

        # 发布结果
        self.pub_res.publish(String(data=result))
        self.get_logger().info(f"🔍 执行结果：{result}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMYOLOSCarControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 用户终止程序")
    node.destroy_node()
    rclpy.shutdown()

# 新增：打印模型路径（调试用）
    self.get_logger().info(f"✅ ViT模型路径：{cfg['vit_model_full_path']}")
    self.get_logger().info(f"✅ LLM模型路径：{cfg['llm_model_full_path']}")

if __name__ == "__main__":
    main()
