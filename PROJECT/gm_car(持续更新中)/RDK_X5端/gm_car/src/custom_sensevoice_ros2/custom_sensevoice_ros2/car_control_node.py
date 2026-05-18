#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time

# ====================== 安全上限（防止大模型输出异常值）======================
MAX_LINEAR_SPEED = 0.3   # 最大线速度 m/s
MAX_ANGULAR_SPEED = 0.8 # 最大角速度 rad/s

class CarControlNode(Node):
    def __init__(self):
        super().__init__("car_control_node")
        
        # 1. 订阅总控发来的「参数化运动指令」（JSON格式）
        self.control_sub = self.create_subscription(
            String, "/control_cmd", self.control_callback, 10
        )
        
        # 2. 发布速度指令给小车
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.get_logger().info("✅ 灵活版小车控制节点启动：支持参数化运动控制")

    # ====================== 核心：参数化运动执行（完全灵活）======================
    def execute_motion(self, linear_x, angular_z, duration):
        """
        按参数执行运动：
        linear_x: 线速度 m/s（正=前进，负=后退）
        angular_z: 角速度 rad/s（正=左转，负=右转）
        duration: 执行时长 秒
        """
        # 安全截断，防止大模型输出超范围值
        linear_x = max(min(linear_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
        angular_z = max(min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

        # 1. 发布速度指令
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(
            f"🚗 执行运动：线速度={linear_x:.2f}m/s, 角速度={angular_z:.2f}rad/s, 时长={duration:.1f}s"
        )

        # 2. 按指定时长运行
        time.sleep(duration)

        # 3. 自动停止
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("🛑 运动执行完成，已自动停止")

    # ====================== 回调：接收总控发来的参数化指令 ======================
    def control_callback(self, msg):
        """
        接收格式：JSON字符串，例如：
        {"linear_x": 0.3, "angular_z": 0.0, "duration": 2.0}
        由智谱总控节点生成并发送
        """
        import json
        try:
            # 解析JSON参数
            cmd_data = json.loads(msg.data)
            linear_x = cmd_data.get("linear_x", 0.0)
            angular_z = cmd_data.get("angular_z", 0.0)
            duration = cmd_data.get("duration", 2.0)

            # 用线程执行，不阻塞ROS主循环
            threading.Thread(
                target=self.execute_motion,
                args=(linear_x, angular_z, duration),
                daemon=True
            ).start()

        except Exception as e:
            self.get_logger().error(f"❌ 指令解析失败: {e}, 原始数据: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CarControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()