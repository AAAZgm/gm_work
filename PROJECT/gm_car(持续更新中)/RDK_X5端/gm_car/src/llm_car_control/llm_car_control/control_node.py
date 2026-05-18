#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        
        # 从配置文件加载参数（可根据底盘修改）
        self.declare_parameters(
            namespace="",
            parameters=[
                ("car.base_linear_speed", 0.2),    # 线性速度 m/s
                ("car.base_angular_speed", 0.5),   # 角速度 rad/s
                ("car.rad_per_degree", 0.01745),   # 角度转弧度
                ("topic.llm_cmd", "/car/llm_cmd"), # LLM指令话题
                ("topic.cmd_vel", "/cmd_vel"),     # 小车运动话题（重点！改这里适配底盘）
                ("topic.exec_result", "/car/exec_result")
            ]
        )
        self.base_linear_speed = self.get_parameter("car.base_linear_speed").value
        self.base_angular_speed = self.get_parameter("car.base_angular_speed").value
        self.rad_per_degree = self.get_parameter("car.rad_per_degree").value
        self.llm_cmd_topic = self.get_parameter("topic.llm_cmd").value
        self.cmd_vel_topic = self.get_parameter("topic.cmd_vel").value
        self.exec_result_topic = self.get_parameter("topic.exec_result").value

        # ROS2订阅/发布器
        self.sub_llm_cmd = self.create_subscription(
            String, self.llm_cmd_topic, self.llm_cmd_callback, 10
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )
        self.pub_exec_result = self.create_publisher(
            String, self.exec_result_topic, 10
        )

        self.get_logger().info("✅ 运动控制节点启动成功")
        self.get_logger().info(f"📌 小车运动话题：{self.cmd_vel_topic}")

    def llm_cmd_callback(self, msg):
        """执行大模型解析后的指令（增加详细日志）"""
        self.get_logger().info(f"📥 收到LLM指令：{msg.data}")
        try:
            cmd_data = json.loads(msg.data)
            action = cmd_data["action"]
            value = float(cmd_data.get("value", 0.0))  # 兼容停止指令无value
            speed = float(cmd_data.get("speed", self.base_linear_speed))
            desc = cmd_data["desc"]

            self.get_logger().info(f"🔧 执行指令：{desc}")
            twist = Twist()
            result = ""

            # 执行动作（通用逻辑）
            if action == "前进":
                duration = value / speed if speed != 0 else 0
                twist.linear.x = speed
                self._execute_movement(twist, duration)
                result = f"✅ 完成前进{value}米（速度{speed}m/s，耗时{duration:.2f}秒）"

            elif action == "后退":
                duration = value / speed if speed != 0 else 0
                twist.linear.x = -speed
                self._execute_movement(twist, duration)
                result = f"✅ 完成后退{value}米（速度{speed}m/s，耗时{duration:.2f}秒）"

            elif action == "左转":
                rad = value * self.rad_per_degree
                duration = rad / self.base_angular_speed if self.base_angular_speed != 0 else 0
                twist.angular.z = self.base_angular_speed
                self._execute_movement(twist, duration)
                result = f"✅ 完成左转{value}度（耗时{duration:.2f}秒）"

            elif action == "右转":
                rad = value * self.rad_per_degree
                duration = rad / self.base_angular_speed if self.base_angular_speed != 0 else 0
                twist.angular.z = -self.base_angular_speed
                self._execute_movement(twist, duration)
                result = f"✅ 完成右转{value}度（耗时{duration:.2f}秒）"

            elif action == "停止":
                self.pub_cmd_vel.publish(Twist())
                result = "✅ 小车已停止"

            else:
                result = f"❌ 不支持的动作：{action}"

            # 发布执行结果
            self.pub_exec_result.publish(String(data=result))
            self.get_logger().info(result)

        except json.JSONDecodeError as e:
            error_msg = f"❌ 指令解析失败（格式错误）：{str(e)} → 正确格式：{{'action':'前进','value':1.0,'desc':'往前走1米'}}"
            self.get_logger().error(error_msg)
            self.pub_exec_result.publish(String(data=error_msg))
        except Exception as e:
            error_msg = f"❌ 指令执行失败：{str(e)}"
            self.get_logger().error(error_msg)
            self.pub_exec_result.publish(String(data=error_msg))


        def _execute_movement(self, twist, duration):
            """修复版：确保运动时长到了必停"""
            if duration <= 0:
                self.get_logger().warn("⚠️ 运动时长为0，跳过执行")
                return
            self.get_logger().info(f"🚗 开始运动：目标时长{duration:.2f}秒，线性速度{twist.linear.x}")
            
            # 修复：用ROS2的Rate和精确计时，避免time.time()漂移
            rate = self.create_rate(50)  # 50Hz高频发布，避免指令中断
            end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)
            
            # 循环发布指令，直到超时
            while self.get_clock().now() < end_time:
                self.pub_cmd_vel.publish(twist)
                rate.sleep()
            
            # 强制发布停止指令（关键！确保必停）
            stop_twist = Twist()
            self.pub_cmd_vel.publish(stop_twist)
            self.get_logger().info(f"🛑 运动结束：已发布停止指令，实际运动时长{duration:.2f}秒")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()