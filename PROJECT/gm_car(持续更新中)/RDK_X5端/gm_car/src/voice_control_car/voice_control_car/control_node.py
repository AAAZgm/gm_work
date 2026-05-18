#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        # 从配置文件加载参数
        self.declare_parameter("llm_cmd_topic", "/car/llm_cmd")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("rad_per_degree", 0.0174533)  # 角度转弧度

        self.llm_cmd_topic = self.get_parameter("llm_cmd_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.rad_per_degree = self.get_parameter("rad_per_degree").value

        # 订阅/发布器
        self.sub_llm_cmd = self.create_subscription(
            String, self.llm_cmd_topic, self.llm_cmd_callback, 10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info("✅ 运动控制节点启动成功")
        self.get_logger().info(f"📌 小车运动话题：{self.cmd_vel_topic}")

    def llm_cmd_callback(self, msg):
        self.get_logger().info(f"📥 收到LLM控制指令：{msg.data}")
        try:
            cmd_data = json.loads(msg.data)
            action = cmd_data["action"]
            value = float(cmd_data["value"])
            speed = float(cmd_data["speed"])
            angular_speed = float(cmd_data["angular_speed"])

            twist = Twist()

            if action == "前进":
                if value <= 0 or speed <= 0:
                    self.get_logger().warn("⚠️ 前进参数无效，跳过执行")
                    return
                duration = value / speed
                twist.linear.x = speed
                self._execute_movement(twist, duration)
                self.get_logger().info(f"✅ 完成前进{value}米（速度{speed}m/s，耗时{duration:.2f}秒）")

            elif action == "后退":
                if value <= 0 or speed <= 0:
                    self.get_logger().warn("⚠️ 后退参数无效，跳过执行")
                    return
                duration = value / speed
                twist.linear.x = -speed
                self._execute_movement(twist, duration)
                self.get_logger().info(f"✅ 完成后退{value}米（速度{speed}m/s，耗时{duration:.2f}秒）")

            elif action == "左转":
                if value <= 0 or angular_speed <= 0:
                    self.get_logger().warn("⚠️ 左转参数无效，跳过执行")
                    return
                rad = value * self.rad_per_degree
                duration = rad / angular_speed
                twist.angular.z = angular_speed
                self._execute_movement(twist, duration)
                self.get_logger().info(f"✅ 完成左转{value}度（角速度{angular_speed}rad/s，耗时{duration:.2f}秒）")

            elif action == "右转":
                if value <= 0 or angular_speed <= 0:
                    self.get_logger().warn("⚠️ 右转参数无效，跳过执行")
                    return
                rad = value * self.rad_per_degree
                duration = rad / angular_speed
                twist.angular.z = -angular_speed
                self._execute_movement(twist, duration)
                self.get_logger().info(f"✅ 完成右转{value}度（角速度{angular_speed}rad/s，耗时{duration:.2f}秒）")

            elif action == "停止":
                self.pub_cmd_vel.publish(Twist())
                self.get_logger().info("✅ 小车已停止")

            else:
                self.get_logger().error(f"❌ 不支持的动作：{action}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ JSON解析失败：{str(e)}")
        except Exception as e:
            self.get_logger().error(f"❌ 控制节点执行失败：{str(e)}")

    def _execute_movement(self, twist, duration):
        """执行持续运动，ROS2时钟精准计时，50Hz高频发布"""
        if duration <= 0:
            return
        self.get_logger().info(f"🚗 开始运动：持续{duration:.2f}秒")
        rate = self.create_rate(50)
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)
        while self.get_clock().now() < end_time:
            self.pub_cmd_vel.publish(twist)
            rate.sleep()
        # 强制停止
        self.pub_cmd_vel.publish(Twist())
        self.get_logger().info("🛑 运动结束，小车停止")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
