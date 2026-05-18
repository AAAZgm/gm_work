#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests
import json
import time
import threading

# ====================== 智谱 AI GLM 配置（你自己填）======================
ZHIPU_API_KEY = "06677cb763954ca198af337e03f999d2.1cwlf162lhvsRmYQ"
ZHIPU_API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
ZHIPU_MODEL = "glm-4"

# ====================== 安全参数 ======================
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 0.8
DEFAULT_DURATION = 2.0
DEFAULT_LINEAR = 0.2
DEFAULT_ANGULAR = 0.4

# ====================== 电脑TTS服务配置 ======================
PC_TTS_URL = "http://192.168.137.17:5000/tts"

class ZhipuMasterController(Node):
    def __init__(self):
        super().__init__("zhipu_master_controller")
        self.logger = self.get_logger()

        # 1. 订阅ASR纯文本（只识别，不自动回复）
        self.asr_sub = self.create_subscription(
            String, "/asr_text", self.on_asr_received, 10
        )

        # 2. 发布小车运动指令
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.logger.info("="*70)
        self.logger.info("✅ 总控：智谱 GLM 大模型 + 参数化运动控制（最终稳定版）")
        self.logger.info("📌 仅处理运动指令：前进/后退/左转/右转/停止，带参数")
        self.logger.info("📌 仅说「回复我」「播报」才语音回复，否则仅执行/不回复")
        self.logger.info("="*70)

    # ====================== 核心：调用智谱API，严格限制输出 ======================
    def call_zhipu(self, user_text):
        headers = {
            "Authorization": f"Bearer {ZHIPU_API_KEY}",
            "Content-Type": "application/json"
        }

        # 🔥 关键优化：严格限制大模型输出，彻底杜绝乱回复
        system_prompt = f"""
你是智能小车的运动控制工具，**只处理小车运动指令，不进行任何对话闲聊**。
输入用户指令：{user_text}

必须严格输出JSON格式，**无任何多余文字、解释、闲聊**，仅包含以下6个字段：
1. action: 动作类型，只能选：forward(前进)、backward(后退)、turn_left(左转)、turn_right(右转)、stop(停止)、chat(非运动/闲聊)
2. linear: 线速度(m/s)，正数前进、负数后退，停止=0，范围[-{MAX_LINEAR_SPEED}, {MAX_LINEAR_SPEED}]
3. angular: 角速度(rad/s)，左转正、右转负，停止=0，范围[-{MAX_ANGULAR_SPEED}, {MAX_ANGULAR_SPEED}]
4. duration: 执行时长(秒)，默认{DEFAULT_DURATION}，仅运动指令有效
5. reply: 简短中文回复（10字内），非运动指令填空字符串
6. need_tts: 是否语音播报，布尔值true/false（**只有用户明确说「回复我」「播报」才为true，否则一律false**）

严格规则：
- 输入不是运动指令（如闲聊、无意义内容），action=chat，linear=0，angular=0，reply=""，need_tts=false
- 速度/角度超出范围，自动截断到上下限
- 停止时linear=0、angular=0
- 绝对禁止输出「不知道」「我听不懂」等闲聊内容
- 严格JSON格式，确保可直接解析
"""

        payload = {
            "model": ZHIPU_MODEL,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_text}
            ],
            "temperature": 0.0,  # 🔥 0温度，完全稳定，杜绝幻觉
            "max_tokens": 200
        }

        try:
            resp = requests.post(ZHIPU_API_URL, headers=headers, json=payload, timeout=10)
            resp.raise_for_status()
            result = resp.json()
            content = result["choices"][0]["message"]["content"].strip()

            # 提取纯JSON
            json_start = content.find("{")
            json_end = content.rfind("}") + 1
            if json_start >= 0 and json_end > json_start:
                return json.loads(content[json_start:json_end])
            else:
                self.logger.error("❌ 智谱输出非JSON，降级规则解析")
                return self.fallback_parse(user_text)
        except Exception as e:
            self.logger.error(f"❌ 智谱调用失败: {str(e)}")
            return self.fallback_parse(user_text)

    # ====================== 兜底规则解析（API异常时用，杜绝乱回复）======================
    def fallback_parse(self, text):
        # 🔥 严格过滤，仅识别明确运动关键词
        text_lower = text.lower()
        action = "chat"
        linear = 0.0
        angular = 0.0
        duration = DEFAULT_DURATION
        reply = ""
        need_tts = "回复我" in text or "播报" in text

        if "前进" in text_lower or "向前走" in text_lower:
            action = "forward"
            linear = DEFAULT_LINEAR
            reply = "前进中" if need_tts else ""
        elif "后退" in text_lower or "向后退" in text_lower:
            action = "backward"
            linear = -DEFAULT_LINEAR
            reply = "后退中" if need_tts else ""
        elif "左转" in text_lower or "向左转" in text_lower:
            action = "turn_left"
            angular = DEFAULT_ANGULAR
            reply = "左转中" if need_tts else ""
        elif "右转" in text_lower or "向右转" in text_lower:
            action = "turn_right"
            angular = -DEFAULT_ANGULAR
            reply = "右转中" if need_tts else ""
        elif "停止" in text_lower:
            action = "stop"
            reply = "已停止" if need_tts else ""
        else:
            # 非运动指令，一律chat，不回复
            action = "chat"
            reply = ""
            need_tts = False

        return {
            "action": action, "linear": linear, "angular": angular,
            "duration": duration, "reply": reply, "need_tts": need_tts
        }

    # ====================== 核心：运动执行（彻底修复类型错误）======================
    def execute_motion(self, linear, angular, duration):
        try:
            # 🔥 强制类型转换+默认值，彻底解决ROS类型错误
            linear_float = float(linear) if linear is not None else 0.0
            angular_float = float(angular) if angular is not None else 0.0
            duration_float = float(duration) if duration is not None else DEFAULT_DURATION

            # 安全限制速度
            linear_x = max(min(linear_float, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
            angular_z = max(min(angular_float, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

            # 发布运动指令
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)

            self.logger.info(f"🚗 执行运动：v={linear_x:.2f}m/s, w={angular_z:.2f}rad/s, t={duration_float:.1f}s")

            # 执行指定时长
            time.sleep(duration_float)

            # 自动停止
            self.cmd_vel_pub.publish(Twist())
            self.logger.info("🛑 运动执行完成，已自动停止")

        except Exception as e:
            self.logger.error(f"❌ 运动执行异常: {str(e)}")
            # 异常强制停车
            self.cmd_vel_pub.publish(Twist())

    # ====================== 发送TTS到电脑 ======================
    def send_tts_to_pc(self, text):
        if not text or text.strip() == "":
            return
        try:
            requests.post(PC_TTS_URL, json={"text": text}, timeout=3)
            self.logger.info(f"🔊 TTS指令已发送到电脑: {text}")
        except Exception as e:
            self.logger.error(f"❌ TTS发送失败: {str(e)}")

    # ====================== ASR回调：总控主逻辑 ======================
    def on_asr_received(self, msg):
        text = msg.data.strip()
        if not text:
            return
        self.logger.info(f"🧠 收到ASR识别文本：{text}")

        # 调用智谱解析
        parsed = self.call_zhipu(text)
        action = parsed.get("action", "chat")
        linear = parsed.get("linear", 0.0)
        angular = parsed.get("angular", 0.0)
        duration = parsed.get("duration", DEFAULT_DURATION)
        reply = parsed.get("reply", "")
        need_tts = parsed.get("need_tts", False)

        # 仅运动指令执行，chat指令直接跳过
        if action in ["forward", "backward", "turn_left", "turn_right", "stop"]:
            threading.Thread(
                target=self.execute_motion,
                args=(linear, angular, duration),
                daemon=True
            ).start()
        else:
            self.logger.info(f"ℹ️ 非运动指令，跳过执行: {text}")

        # 仅need_tts=true且reply非空，才发送TTS
        if need_tts and reply and reply.strip() != "":
            self.send_tts_to_pc(reply)
        else:
            self.logger.info(f"ℹ️ 不触发TTS播报")

def main(args=None):
    rclpy.init(args=args)
    node = ZhipuMasterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()