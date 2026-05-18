#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_cpp import Llama
import json
import re

class LLMNode(Node):
    def __init__(self):
        super().__init__("llm_node")
        # 从配置文件加载参数
        self.declare_parameter("model_path", "/home/sunrise/gm_car/colcon_ws/src/lla_cpp/config/Qwen2-0.5B-Instruct-Q4_0.gguf")
        self.declare_parameter("user_cmd_topic", "/car/user_cmd")
        self.declare_parameter("llm_cmd_topic", "/car/llm_cmd")
        self.declare_parameter("tts_topic", "/car/tts_play")
        self.declare_parameter("n_ctx", 2048)
        self.declare_parameter("n_threads", 4)  # RDK X5核心数

        # 加载参数
        self.model_path = self.get_parameter("model_path").value
        self.user_cmd_topic = self.get_parameter("user_cmd_topic").value
        self.llm_cmd_topic = self.get_parameter("llm_cmd_topic").value
        self.tts_topic = self.get_parameter("tts_topic").value
        self.n_ctx = self.get_parameter("n_ctx").value
        self.n_threads = self.get_parameter("n_threads").value

        # 初始化大模型（适配RDK X5 CPU推理）
        self.get_logger().info(f"🧠 加载大模型：{self.model_path}")
        self.llm = Llama(
            model_path=self.model_path,
            n_ctx=self.n_ctx,
            n_threads=self.n_threads,
            n_gpu_layers=0,  # RDK X5用CPU推理，设为0
            verbose=False
        )
        self.get_logger().info("✅ 大模型加载完成")

        # 订阅/发布器
        self.sub_user_cmd = self.create_subscription(
            String, self.user_cmd_topic, self.user_cmd_callback, 10
        )
        self.pub_llm_cmd = self.create_publisher(String, self.llm_cmd_topic, 10)
        self.pub_tts = self.create_publisher(String, self.tts_topic, 10)

        # 系统提示词（核心：定义大模型的角色和输出格式）
        self.system_prompt = """
你是智能小车的决策大模型，需要解析用户的自然语言指令，输出严格的JSON格式控制指令，同时生成自然语言回复。
规则：
1. 动作仅支持：前进、后退、左转、右转、停止
2. 自动提取距离（单位：米）、角度（单位：度），无数值默认：前进/后退1米，左转/右转90度
3. 自动提取速度：快速=0.3m/s，慢速=0.1m/s，默认0.2m/s
4. 输出必须是严格的JSON，包含以下字段：
   - action: 动作名称（前进/后退/左转/右转/停止）
   - value: 距离/角度数值（停止时为0）
   - speed: 线性速度（m/s，仅前进/后退有效）
   - angular_speed: 角速度（rad/s，仅左转/右转有效，默认0.5）
   - desc: 自然语言回复（如“好的，为你前进2米”）
5. 禁止输出任何额外内容，仅保留JSON
示例：
用户指令：前进2米，快点
输出：{"action":"前进","value":2.0,"speed":0.3,"angular_speed":0.5,"desc":"好的，为你快速前进2米"}
用户指令：左转45度
输出：{"action":"左转","value":45.0,"speed":0.2,"angular_speed":0.5,"desc":"好的，为你左转45度"}
用户指令：停止
输出：{"action":"停止","value":0.0,"speed":0.0,"angular_speed":0.0,"desc":"小车已停止"}
"""

    def user_cmd_callback(self, msg):
        user_cmd = msg.data.strip()
        self.get_logger().info(f"🧠 收到用户指令：{user_cmd}")

        # 构造prompt
        prompt = f"<|im_start|>system\n{self.system_prompt}<|im_end|>\n<|im_start|>user\n{user_cmd}<|im_end|>\n<|im_start|>assistant\n"

        try:
            # 大模型推理
            output = self.llm(
                prompt=prompt,
                max_tokens=256,
                temperature=0.1,  # 低温度，保证输出稳定
                stop=["<|im_end|>"],
                echo=False
            )
            llm_output = output["choices"][0]["text"].strip()
            self.get_logger().info(f"🤖 大模型输出：{llm_output}")

            # 提取JSON（处理大模型可能的多余输出）
            json_match = re.search(r'\{.*\}', llm_output, re.DOTALL)
            if not json_match:
                raise ValueError("未提取到有效JSON")
            cmd_json = json.loads(json_match.group())

            # 校验字段
            required_fields = ["action", "value", "speed", "angular_speed", "desc"]
            for field in required_fields:
                if field not in cmd_json:
                    raise ValueError(f"JSON缺少字段：{field}")

            # 发布控制指令和TTS回复
            self.pub_llm_cmd.publish(String(data=json.dumps(cmd_json)))
            self.pub_tts.publish(String(data=cmd_json["desc"]))
            self.get_logger().info(f"✅ 发布控制指令：{json.dumps(cmd_json)}")

        except Exception as e:
            error_msg = f"❌ 大模型推理失败：{str(e)}"
            self.get_logger().error(error_msg)
            self.pub_tts.publish(String(data="抱歉，我没听懂你的指令"))

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
