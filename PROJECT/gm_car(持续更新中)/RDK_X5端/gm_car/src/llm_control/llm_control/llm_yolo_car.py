#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import os
import time
import re
import json
from ultralytics import YOLO
from llama_cpp import Llama

# ========== 配置路径（仅改这里） ==========
CONFIG_PATH = "/home/sunrise/gm_car/llm_control/config/config.yaml"
# =========================================

# 加载配置
with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)
cfg["vit_model_full_path"] = os.path.join(cfg["model_dir"], cfg["vit_model_name"])
cfg["llm_model_full_path"] = os.path.join(cfg["model_dir"], cfg["llm_model_name"])

# 初始化大模型（Qwen2.5-0.5B，推理级能力）
llm = Llama(
    model_path=cfg["llm_model_full_path"],
    n_ctx=1024,
    n_threads=4,
    n_gpu_layers=0,
    verbose=False
)

class SmartCarNode(Node):
    def __init__(self):
        super().__init__("smart_car_core_node")
        
        # 1. 视觉初始化
        self.yolo = YOLO("yolov8n.pt").to("cpu")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_width = 640
        self.frame_height = 480
        self.yolo_conf = cfg["yolo_conf"]
        
        # 2. 控制参数
        self.base_linear_speed = 0.2
        self.base_angular_speed = 0.5
        self.rad_per_degree = 0.01745
        
        # 3. ROS2通信
        self.sub_img = self.create_subscription(Image, cfg["camera_topic"], self.img_callback, 10)
        self.sub_user_cmd = self.create_subscription(String, cfg["user_cmd_topic"], self.cmd_callback, 10)
        self.pub_vel = self.create_publisher(Twist, cfg["cmd_vel_topic"], 10)
        self.pub_vision_desc = self.create_publisher(String, "/car/vision_description", 10)
        self.pub_object_coords = self.create_publisher(String, "/car/object_coords", 10)
        self.pub_exec_result = self.create_publisher(String, cfg["result_topic"], 10)
        
        self.get_logger().info("✅ 智能小车核心节点启动成功")
        self.get_logger().info(f"✅ 大模型加载完成：{os.path.basename(cfg['llm_model_full_path'])}")

    def img_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_height, self.frame_width = self.latest_frame.shape[:2]
        except Exception as e:
            self.get_logger().error(f"相机帧处理失败：{str(e)}")

    def yolo_detect_with_coords(self):
        if self.latest_frame is None:
            return {"status": "error", "msg": "无相机帧", "objects": []}
        results = self.yolo(self.latest_frame, conf=self.yolo_conf, device="cpu")[0]
        objects = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            cx_offset = cx - self.frame_width/2
            cy_offset = cy - self.frame_height/2
            cls = results.names[int(box.cls[0])]
            conf = round(float(box.conf[0]), 2)
            obj = {
                "class": cls,
                "confidence": conf,
                "pixel_coords": {"x1":x1, "y1":y1, "x2":x2, "y2":y2},
                "center_coords": {"cx":cx, "cy":cy},
                "offset": {"cx_offset":cx_offset, "cy_offset":cy_offset},
                "world_coords": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
            objects.append(obj)
        self.pub_object_coords.publish(String(data=json.dumps(objects, ensure_ascii=False)))
        return {"status": "success", "frame_size": {"width": self.frame_width, "height": self.frame_height}, "objects": objects}

    def llm_vision_reasoning(self, vision_data):
        if vision_data["status"] == "error":
            return f"视觉推理失败：{vision_data['msg']}"
        prompt = f"""
你是专业的视觉推理助手，基于以下视觉检测数据，回答问题并做详细描述：
1. 画面中有哪些物体？每个物体的位置、大小、相对关系是什么？
2. 物体是否可被抓取？（判断依据：清晰可见、无遮挡、尺寸适配机械臂）
3. 小车需要如何移动才能让机械臂对准目标物体？

视觉检测数据：
{json.dumps(vision_data, ensure_ascii=False, indent=2)}

要求：
- 描述要详细且精准，比如“画面中心偏左20像素处有一个可乐瓶，高度约占画面1/4，无遮挡，可抓取，小车需右转5度对准”；
- 仅返回自然语言描述，无多余格式。
"""
        output = llm.create_completion(prompt=prompt, max_tokens=256, temperature=0.2, stop=["\n\n"])
        desc = output["choices"][0]["text"].strip()
        self.pub_vision_desc.publish(String(data=desc))
        return desc

    def llm_cmd_parsing(self, user_cmd, vision_data):
        prompt_template = """
你是智能小车的控制大脑，需完成以下任务：
1. 理解用户指令，结合当前视觉数据，生成可执行的小车控制参数；
2. 控制参数包含：动作（前进/后退/左转/右转/停止/抓取）、距离（米）/角度（度）、速度（m/s/rad/s）；
3. 若指令涉及物体操作，需结合视觉数据给出精准移动方案；
4. 仅返回JSON格式，无任何多余文字，JSON字段：action, value, speed, desc。

示例1：用户说"往前走3米" → {{"action":"前进","value":3.0,"speed":0.2,"desc":"小车以0.2m/s的速度前进3米"}}
示例2：用户说"对准画面中的可乐瓶" → {{"action":"右转","value":5.0,"speed":0.5,"desc":"小车右转5度，对准可乐瓶"}}

当前视觉数据：
{vision_json}

用户指令：{user_cmd}
JSON结果：
        """
        prompt = prompt_template.format(
            vision_json=json.dumps(vision_data, ensure_ascii=False, indent=2),
            user_cmd=user_cmd
        )
        try:
            output = llm.create_completion(prompt=prompt, max_tokens=128, temperature=0.1, stop=["\n"])
            return json.loads(output["choices"][0]["text"].strip())
        except Exception as e:
            self.get_logger().warning(f"大模型指令解析失败，使用兜底逻辑：{str(e)}")
            cmd_data = {"action":"未知","value":0.0,"speed":0.0,"desc":"无法解析指令"}
            if "前进" in user_cmd:
                cmd_data["action"] = "前进"
                cmd_data["value"] = float(re.findall(r"\d+", user_cmd)[0]) if re.findall(r"\d+", user_cmd) else 1.0
                cmd_data["speed"] = self.base_linear_speed
                cmd_data["desc"] = f"前进{cmd_data['value']}米，速度{cmd_data['speed']}m/s"
            elif "后退" in user_cmd:
                cmd_data["action"] = "后退"
                cmd_data["value"] = float(re.findall(r"\d+", user_cmd)[0]) if re.findall(r"\d+", user_cmd) else 1.0
                cmd_data["speed"] = self.base_linear_speed
                cmd_data["desc"] = f"后退{cmd_data['value']}米，速度{cmd_data['speed']}m/s"
            elif "左转" in user_cmd:
                cmd_data["action"] = "左转"
                cmd_data["value"] = float(re.findall(r"\d+", user_cmd)[0]) if re.findall(r"\d+", user_cmd) else 90.0
                cmd_data["speed"] = self.base_angular_speed
                cmd_data["desc"] = f"左转{cmd_data['value']}度，角速度{cmd_data['speed']}rad/s"
            elif "右转" in user_cmd:
                cmd_data["action"] = "右转"
                cmd_data["value"] = float(re.findall(r"\d+", user_cmd)[0]) if re.findall(r"\d+", user_cmd) else 90.0
                cmd_data["speed"] = self.base_angular_speed
                cmd_data["desc"] = f"右转{cmd_data['value']}度，角速度{cmd_data['speed']}rad/s"
            elif "停止" in user_cmd:
                cmd_data["action"] = "停止"
                cmd_data["desc"] = "小车停止运动"
            elif "看" in user_cmd or "描述" in user_cmd:
                cmd_data["action"] = "视觉描述"
                cmd_data["desc"] = "执行视觉推理并返回画面描述"
            elif "抓取" in user_cmd:
                cmd_data["action"] = "抓取"
                cmd_data["desc"] = "控制机械臂抓取目标物体"
            return cmd_data

    def execute_movement(self, action, value, speed):
        twist = Twist()
        result = ""
        if action == "前进":
            duration = value / speed
            twist.linear.x = speed
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub_vel.publish(twist)
                time.sleep(0.05)
            self.pub_vel.publish(Twist())
            result = f"✅ 完成前进{value}米（实际耗时：{time.time()-start_time:.2f}秒）"
        elif action == "后退":
            duration = value / speed
            twist.linear.x = -speed
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub_vel.publish(twist)
                time.sleep(0.05)
            self.pub_vel.publish(Twist())
            result = f"✅ 完成后退{value}米（实际耗时：{time.time()-start_time:.2f}秒）"
        elif action == "左转":
            rad = value * self.rad_per_degree
            duration = rad / speed
            twist.angular.z = speed
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub_vel.publish(twist)
                time.sleep(0.05)
            self.pub_vel.publish(Twist())
            result = f"✅ 完成左转{value}度（实际耗时：{time.time()-start_time:.2f}秒）"
        elif action == "右转":
            rad = value * self.rad_per_degree
            duration = rad / speed
            twist.angular.z = -speed
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub_vel.publish(twist)
                time.sleep(0.05)
            self.pub_vel.publish(Twist())
            result = f"✅ 完成右转{value}度（实际耗时：{time.time()-start_time:.2f}秒）"
        elif action == "停止":
            self.pub_vel.publish(Twist())
            result = "✅ 小车已停止"
        elif action == "抓取":
            result = "❌ 机械臂模块未就绪，无法抓取"
        else:
            result = f"❌ 不支持的动作：{action}"
        return result

    def cmd_callback(self, msg):
        user_cmd = msg.data.strip()
        self.get_logger().info(f"\n📢 收到指令：{user_cmd}")
        vision_data = self.yolo_detect_with_coords()
        if "看" in user_cmd or "描述" in user_cmd or "有什么" in user_cmd or "抓取" in user_cmd:
            vision_desc = self.llm_vision_reasoning(vision_data)
            self.pub_exec_result.publish(String(data=f"✅ 大模型视觉推理结果：{vision_desc}"))
            self.get_logger().info(f"🤖 视觉推理结果：{vision_desc}")
        else:
            cmd_data = self.llm_cmd_parsing(user_cmd, vision_data)
            self.get_logger().info(f"🤖 大模型解析结果：{json.dumps(cmd_data, ensure_ascii=False)}")
            exec_result = self.execute_movement(cmd_data["action"], cmd_data["value"], cmd_data.get("speed", self.base_linear_speed))
            self.pub_exec_result.publish(String(data=exec_result))
            self.get_logger().info(f"🔍 执行结果：{exec_result}")

def main(args=None):
    rclpy.init(args=args)
    node = SmartCarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 用户终止程序")
    finally:
        node.pub_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
