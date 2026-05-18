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
    n_ctx=1024,  # 增大上下文，容纳视觉描述+指令解析
    n_threads=4, # RDK X5核心数
    n_gpu_layers=0, # 纯CPU推理（适配RDK X5）
    verbose=False
)

class SmartCarNode(Node):
    def __init__(self):
        super().__init__("smart_car_core_node")
        
        # 1. 视觉初始化
        self.yolo = YOLO("/home/sunrise/gm_car/colcon_ws/yolov8n.pt").to("cpu")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_width = 640  # 摄像头分辨率（可自动获取）
        self.frame_height = 480
        self.yolo_conf = cfg["yolo_conf"]
        
        # 2. 控制参数（可动态调整）
        self.base_linear_speed = 0.2  # m/s
        self.base_angular_speed = 0.5 # rad/s
        self.rad_per_degree = 0.01745 # 角度转弧度
        self.car_wheel_base = 0.2     # 轮距（用于坐标换算，适配你的小车）
        
        # 3. ROS2通信
        # 订阅
        self.sub_img = self.create_subscription(Image, cfg["camera_topic"], self.img_callback, 10)
        self.sub_user_cmd = self.create_subscription(String, cfg["user_cmd_topic"], self.cmd_callback, 10)
        # 发布
        self.pub_vel = self.create_publisher(Twist, cfg["cmd_vel_topic"], 10)
        self.pub_vision_desc = self.create_publisher(String, "/car/vision_description", 10)  # 视觉描述
        self.pub_object_coords = self.create_publisher(String, "/car/object_coords", 10)    # 物体坐标（给机械臂）
        self.pub_exec_result = self.create_publisher(String, cfg["result_topic"], 10)      # 执行结果
        
        # 4. 预留接口标志
        self.voice_module_ready = False  # 语音模块
        self.gesture_module_ready = False # 手势识别
        self.arm_module_ready = False    # 机械臂
        
        self.get_logger().info("✅ 智能小车核心节点启动成功")
        self.get_logger().info(f"✅ 大模型加载完成：{os.path.basename(cfg['llm_model_full_path'])}")

    # ========== 视觉模块：YOLO精准坐标识别 ==========
    def img_callback(self, msg):
        """接收相机帧+自动获取分辨率"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_height, self.frame_width = self.latest_frame.shape[:2]
        except Exception as e:
            self.get_logger().error(f"相机帧处理失败：{str(e)}")

    def yolo_detect_with_coords(self):
        """YOLO检测：输出物体类别+像素坐标+中心坐标+置信度"""
        if self.latest_frame is None:
            return {"status": "error", "msg": "无相机帧", "objects": []}
        
        results = self.yolo(self.latest_frame, conf=self.yolo_conf, device="cpu")[0]
        objects = []
        
        for box in results.boxes:
            # 像素坐标（x1,y1:左上角；x2,y2:右下角）
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # 中心坐标
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            # 相对画面中心的偏移（用于机械臂/小车定位）
            cx_offset = cx - self.frame_width/2
            cy_offset = cy - self.frame_height/2
            # 类别+置信度
            cls = results.names[int(box.cls[0])]
            conf = round(float(box.conf[0]), 2)
            
            # 封装物体信息
            obj = {
                "class": cls,
                "confidence": conf,
                "pixel_coords": {"x1":x1, "y1":y1, "x2":x2, "y2":y2},
                "center_coords": {"cx":cx, "cy":cy},
                "offset": {"cx_offset":cx_offset, "cy_offset":cy_offset},
                # 预留世界坐标（需相机标定，适配机械臂）
                "world_coords": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
            objects.append(obj)
        
        # 发布物体坐标（给机械臂）
        self.pub_object_coords.publish(String(data=json.dumps(objects, ensure_ascii=False)))
        
        return {
            "status": "success",
            "frame_size": {"width": self.frame_width, "height": self.frame_height},
            "objects": objects
        }

    # ========== 大模型模块：视觉推理+指令解析 ==========
    def llm_vision_reasoning(self, vision_data):
        """大模型视觉推理：基于YOLO数据做自然语言描述（推理级）"""
        if vision_data["status"] == "error":
            return f"视觉推理失败：{vision_data['msg']}"
        
        # 构建视觉推理提示词（让大模型做深度推理）
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
        
        # 大模型推理
        output = llm.create_completion(
            prompt=prompt,
            max_tokens=256,
            temperature=0.2,  # 低温度保证精准
            stop=["\n\n"]
        )
        
        desc = output["choices"][0]["text"].strip()
        # 发布视觉描述
        self.pub_vision_desc.publish(String(data=desc))
        return desc

    def llm_cmd_parsing(self, user_cmd, vision_data):
        """大模型指令解析：理解自然语言→生成动态控制指令"""
        # 构建指令解析提示词（结合视觉数据）
        prompt_template = """
你是智能小车的控制大脑，需完成以下任务：
1. 理解用户指令，结合当前视觉数据，生成可执行的小车控制参数；
2. 控制参数包含：动作（前进/后退/左转/右转/停止/抓取）、距离（米）/角度（度）、速度（m/s/rad/s）；
3. 若指令涉及物体操作，需结合视觉数据给出精准移动方案；
4. 仅返回JSON格式，无任何多余文字，JSON字段：action, value, speed, desc。

示例1（纯移动）：
用户指令："往前走3米"
返回：{{"action":"前进","value":3.0,"speed":0.2,"desc":"小车以0.2m/s的速度前进3米"}}

示例2（结合视觉）：
用户指令："对准画面中的可乐瓶"
视觉数据：{{"objects": [{{"class":"bottle","offset":{{"cx_offset":-20,"cy_offset":5}}}]}}
返回：{{"action":"右转","value":5.0,"speed":0.5,"desc":"小车右转5度，对准可乐瓶"}}

示例3（抓取指令）：
用户指令："抓取画面中的杯子"
返回：{{"action":"抓取","value":0.0,"speed":0.0,"desc":"控制机械臂抓取画面中的杯子，小车保持静止"}}

当前视觉数据：
{vision_json}

用户指令：{user_cmd}
JSON结果：
        """
        prompt = prompt_template.format(
            vision_json=json.dumps(vision_data, ensure_ascii=False, indent=2),
            user_cmd=user_cmd
        )
        
        # 大模型推理
        try:
            output = llm.create_completion(
                prompt=prompt,
                max_tokens=128,
                temperature=0.1,
                stop=["\n"]
            )
            cmd_data = json.loads(output["choices"][0]["text"].strip())
            return cmd_data
        except Exception as e:
            # 兜底：基础指令解析（无视觉结合）
            self.get_logger().warning(f"大模型指令解析失败，使用兜底逻辑：{str(e)}")
            return self.basic_cmd_parsing(user_cmd)

    def basic_cmd_parsing(self, user_cmd):
        """兜底指令解析（无大模型时）"""
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

    # ========== 控制模块：动态执行指令 ==========
    def execute_movement(self, action, value, speed):
        """执行小车运动（动态参数）"""
        twist = Twist()
        result = ""
        
        if action == "前进":
            duration = value / speed
            twist.linear.x = speed
            self.get_logger().info(f"📌 执行前进：{value}米，速度{speed}m/s，预计{duration:.2f}秒")
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub_vel.publish(twist)
                time.sleep(0.05)
            self.pub_vel.publish(Twist())
            result = f"✅ 完成前进{value}米（实际耗时：{time.time()-start_time:.2f}秒）"
        
        elif action == "后退":
            duration = value / speed
            twist.linear.x = -speed
            self.get_logger().info(f"📌 执行后退：{value}米，速度{speed}m/s，预计{duration:.2f}秒")
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
            self.get_logger().info(f"📌 执行左转：{value}度，角速度{speed}rad/s，预计{duration:.2f}秒")
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
            self.get_logger().info(f"📌 执行右转：{value}度，角速度{speed}rad/s，预计{duration:.2f}秒")
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
            if self.arm_module_ready:
                result = "✅ 发送抓取指令给机械臂（需对接机械臂接口）"
            else:
                result = "❌ 机械臂模块未就绪，无法抓取"
        
        else:
            result = f"❌ 不支持的动作：{action}"
        
        return result

    # ========== 核心回调：处理用户指令 ==========
    def cmd_callback(self, msg):
        """处理用户指令：视觉推理+指令解析+执行"""
        user_cmd = msg.data.strip()
        self.get_logger().info(f"\n📢 收到用户指令：{user_cmd}")
        
        # 1. 获取最新视觉数据
        vision_data = self.yolo_detect_with_coords()
        
        # 2. 分指令类型处理
        if "看" in user_cmd or "描述" in user_cmd or "有什么" in user_cmd:
            # 视觉推理指令
            vision_desc = self.llm_vision_reasoning(vision_data)
            self.pub_exec_result.publish(String(data=f"✅ 视觉推理结果：{vision_desc}"))
            self.get_logger().info(f"🤖 视觉推理结果：{vision_desc}")
        
        else:
            # 控制指令
            # 大模型解析指令
            cmd_data = self.llm_cmd_parsing(user_cmd, vision_data)
            self.get_logger().info(f"🤖 大模型解析结果：{json.dumps(cmd_data, ensure_ascii=False)}")
            # 执行指令
            exec_result = self.execute_movement(
                action=cmd_data["action"],
                value=cmd_data["value"],
                speed=cmd_data.get("speed", self.base_linear_speed)
            )
            # 发布执行结果
            self.pub_exec_result.publish(String(data=exec_result))
            self.get_logger().info(f"🔍 执行结果：{exec_result}")

# ========== 预留接口：语音/手势/机械臂 ==========
def voice_module_interface(node):
    """语音模块接口（示例）"""
    # 对接语音识别SDK（如百度/科大讯飞），将语音转文字后发布到/car/user_cmd话题
    node.voice_module_ready = True
    node.get_logger().info("📢 语音模块已就绪")

def gesture_recognition_interface(node):
    """手势识别接口（示例）"""
    # 对接YOLO手势识别模型，将手势指令转文字后发布到/car/user_cmd话题
    node.gesture_module_ready = True
    node.get_logger().info("🤏 手势识别模块已就绪")

def arm_control_interface(node):
    """机械臂接口（示例）"""
    # 对接机械臂ROS2节点，订阅/car/object_coords话题获取坐标后发送抓取指令
    node.arm_module_ready = True
    node.get_logger().info("🦾 机械臂模块已就绪")

# ========== 主函数 ==========
def main(args=None):
    rclpy.init(args=args)
    node = SmartCarNode()
    
    # 初始化预留模块（按需启用）
    # voice_module_interface(node)
    # gesture_recognition_interface(node)
    # arm_control_interface(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 用户终止程序")
    finally:
        # 停止小车
        node.pub_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()