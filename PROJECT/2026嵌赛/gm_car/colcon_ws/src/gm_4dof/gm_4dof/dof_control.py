#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gm_4dof_interfaces.srv import Catch, Loosen, Home
# from gm_4dof.arm_vision import ArmVisionGuide


from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')

        # micro-ROS 用 BEST_EFFORT，订阅端也必须用 BEST_EFFORT
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 订阅关节角度（micro-ROS 发布，用 BEST_EFFORT）
        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/arm_joint_angles',
            self.angle_callback, qos_best_effort)

        # 订阅温湿度（micro-ROS 发布，用 BEST_EFFORT）
        self.temp_sub = self.create_subscription(
            Float32MultiArray, '/sensor_temp_humidity',
            self.temp_callback, qos_best_effort)

        # 发布关节命令
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/arm_joint_commands', 10)
        
        self.ready_angles = [90.0, 90.0, 90.0, 90.0]
        # 当前关节角度（从话题反馈中更新）
        self.current_angles = [90.0, 90.0, 90.0, 90.0]
        # 机械臂家位置（home）
        self.home_angles = [90.0, 90.0, 90.0, 90.0]
        # 夹爪角度：第4轴，135.0=松开，45.0=夹紧（根据实际调整）
        self.gripper_open = 135.0
        self.gripper_close = 45.0

        # 视觉引导模块
        # self.vision_guide = ArmVisionGuide(self)

        # 创建服务
        # self.find_service = self.create_service(Find, '/find_object', self.find_callback)
        self.catch_service = self.create_service(Catch, '/catch_object', self.catch_callback)
        self.loosen_service = self.create_service(Loosen, '/loosen_object', self.loosen_callback)
        self.home_service = self.create_service(Home, '/home_object', self.home_callback)

        self.get_logger().info('Robot Arm Controller started!')

    def angle_callback(self, msg):
        data = list(msg.data)
        # ESP32 micro-ROS 在 index 0 附加了时间戳，跳过它
        self.current_angles = data[1:] if len(data) > 4 else data
        self.get_logger().info(f'关节角度: {self.current_angles}')

    def temp_callback(self, msg):
        data = list(msg.data)
        # ESP32 micro-ROS 在 index 0 附加了时间戳，跳过它
        temp, humid = (data[1], data[2]) if len(data) >= 3 else (data[0], data[1])
        self.get_logger().info(f'温度: {temp:.1f}°C, 湿度: {humid:.1f}%')

    def set_angles(self, angles: list):
        """设置关节角度 [angle0, angle1, angle2, angle3]"""
        msg = Float32MultiArray()
        msg.data = angles
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'发送关节命令: {angles}')

    def set_gripper(self, angle: float):
        """控制夹爪（第4轴），前3轴保持不变"""
        cmd = [self.current_angles[0], self.current_angles[1], self.current_angles[2], angle]
        self.set_angles(cmd)

    # def find_callback(self, request, response):
    #     """
    #     先到拍照待命位，然后检测物体，用基准角度+偏移量靠近
        
    #     原理：
    #       机械臂先到 base_angles（拍照待命位，和物体在同一平面高度）
    #       摄像头检测物体像素位置 → 算出相对中心的mm偏移(dx,dy)
    #       base_angles + 偏移对应的角度补偿 → 发送
    #     """
    #     color = request.color
    #     shape = request.shape

    #     self.get_logger().info(f'开始寻找物体: 颜色={color}, 形状={shape}')

    #     # 先到拍照待命位（和物体在同一平面高度）
    #     self.set_angles(self.ready_angles)
    #     time.sleep(2.0)
    #     self.get_logger().info('到达待命位')

    #     # 视觉检测 → 获取mm偏移量
    #     success, dx_mm, dy_mm = self.vision_guide.find(
    #         color=color,
    #         shape=shape,
    #         max_retries=5
    #     )

    #     if success:
    #         # 基准角度：物体所在平面的固定角度（Z轴高度已锁定）
    #         base_angles = [110.0, 120.0, 111.0, 120.0]
    #         # TODO: mm偏移→角度偏移的比例系数，需实际校准
    #         # 比如系数=0.3，偏移10mm就转3度
    #         mm_to_deg = 0.3

    #         # X偏移 → 关节0（底座左右转）
    #         # Y偏移 → 关节1/2（前后弯）
    #         target_angles = [
    #             base_angles[0] + dx_mm * mm_to_deg,
    #             base_angles[1] + dy_mm * mm_to_deg,
    #             base_angles[2] - dy_mm * mm_to_deg * 0.5,
    #             base_angles[3],
    #         ]
    #         self.get_logger().info(f'基准角度={base_angles}, mm偏移=({dx_mm:.1f},{dy_mm:.1f})')
    #         self.get_logger().info(f'目标角度={target_angles}')
    #         self.set_angles(target_angles)
    #         self.get_logger().info('已移动到物体位置')
    #     else:
    #         self.get_logger().warn('未能检测到物体')

    #     response.result = success
    #     return response

    def home_callback(self, request, response):
        """机械臂回到初始位置"""
        delay_time = request.delay_time

        self.get_logger().info(f'机械臂复位, 等待={delay_time}秒')
        self.set_angles(self.home_angles)
        time.sleep(delay_time)

        response.result = True
        self.get_logger().info('机械臂复位完成')
        return response

    def catch_callback(self, request, response):
        """夹住物体"""
        delay_time = request.delay_time

        self.get_logger().info(f'夹爪夹紧(第4轴={self.gripper_close}), 等待={delay_time}秒')
        self.set_gripper(self.gripper_close)
        time.sleep(delay_time)

        response.result = True
        self.get_logger().info('夹爪夹紧完成')
        return response

    def loosen_callback(self, request, response):
        """松开物体"""
        delay_time = request.delay_time

        self.get_logger().info(f'夹爪松开(第4轴={self.gripper_open}), 等待={delay_time}秒')
        self.set_gripper(self.gripper_open)
        time.sleep(delay_time)

        response.result = True
        self.get_logger().info('夹爪松开完成')
        return response


def main():
    rclpy.init()
    node = RobotArmController()

    # 测试：设置关节角度
    time.sleep(2)  # 等待连接
    node.set_angles([90.0, 90.0, 90.0, 90.0])  # 工作位：肘弯曲留活动空间

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
