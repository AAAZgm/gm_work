#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # 订阅关节角度
        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/arm_joint_angles', 
            self.angle_callback, 10)
        
        # 订阅温湿度
        self.temp_sub = self.create_subscription(
            Float32MultiArray, '/sensor_temp_humidity', 
            self.temp_callback, 10)
        
        # 发布关节命令
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/arm_joint_commands', 10)
        
        # 复位服务客户端
        self.home_client = self.create_client(Trigger, '/arm_home')
        
        self.get_logger().info('Robot Arm Controller started!')
    
    def angle_callback(self, msg):
        angles = msg.data
        self.get_logger().info(f'关节角度: {list(angles)}')
    
    def temp_callback(self, msg):
        temp, humid = msg.data
        self.get_logger().info(f'温度: {temp:.1f}°C, 湿度: {humid:.1f}%')
    
    def set_angles(self, angles: list):
        """设置关节角度 [angle0, angle1, angle2, angle3]"""
        msg = Float32MultiArray()
        msg.data = angles
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'发送命令: {angles}')
    
    def go_home(self):
        """复位机械臂"""
        if not self.home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('服务 /arm_home 不可用')
            return
        future = self.home_client.call_async(Trigger.Request())
        future.add_done_callback(self.home_callback)
    
    def home_callback(self, future):
        result = future.result()
        self.get_logger().info(f'复位结果: success={result.success}, msg={result.message}')

def main():
    rclpy.init()
    node = RobotArmController()
    
    # 测试：设置关节角度
    import time
    time.sleep(2)  # 等待连接
    node.set_angles([90, 90, 45, 135])  # 设置 4 个关节角度
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
