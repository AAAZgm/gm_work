#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class LLMNode(Node):
    def __init__(self):
        super().__init__("llm_node")
        
        # 参数配置
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic.user_cmd", "/car/user_cmd"),
                ("topic.llm_cmd", "/car/llm_cmd")
            ]
        )
        self.user_cmd_topic = self.get_parameter("topic.user_cmd").value
        self.llm_cmd_topic = self.get_parameter("topic.llm_cmd").value

        # ROS2订阅/发布器
        self.sub_user_cmd = self.create_subscription(
            String, self.user_cmd_topic, self.user_cmd_callback, 10
        )
        self.pub_llm_cmd = self.create_publisher(
            String, self.llm_cmd_topic, 10
        )

        self.get_logger().info("✅ 大模型决策节点启动成功")

    def user_cmd_callback(self, msg):
        """解析用户自然语言指令，输出标准JSON控制指令"""
        user_cmd = msg.data.strip()
        self.get_logger().info(f"📥 收到用户指令：{user_cmd}")
        
        # 核心：指令映射（自然语言→JSON）
        cmd_json = {}
        if "前进" in user_cmd:
            # 提取距离（默认1米）
            value = 1.0
            if "2米" in user_cmd:
                value = 2.0
            elif "3米" in user_cmd:
                value = 3.0
            cmd_json = {
                "action": "前进",
                "value": value,
                "speed": 0.2,
                "desc": f"往前走{value}米"
            }
        elif "后退" in user_cmd:
            value = 1.0
            if "2米" in user_cmd:
                value = 2.0
            cmd_json = {
                "action": "后退",
                "value": value,
                "speed": 0.2,
                "desc": f"往后退{value}米"
            }
        elif "左转" in user_cmd:
            value = 90.0
            if "45度" in user_cmd:
                value = 45.0
            cmd_json = {
                "action": "左转",
                "value": value,
                "desc": f"左转{value}度"
            }
        elif "右转" in user_cmd:
            value = 90.0
            if "45度" in user_cmd:
                value = 45.0
            cmd_json = {
                "action": "右转",
                "value": value,
                "desc": f"右转{value}度"
            }
        elif "停止" in user_cmd:
            cmd_json = {
                "action": "停止",
                "desc": "小车停止运动"
            }
        else:
            self.get_logger().error(f"❌ 不支持的指令：{user_cmd}")
            return

        # 发布JSON指令
        self.pub_llm_cmd.publish(String(data=json.dumps(cmd_json)))
        self.get_logger().info(f"📤 发布LLM指令：{json.dumps(cmd_json)}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()