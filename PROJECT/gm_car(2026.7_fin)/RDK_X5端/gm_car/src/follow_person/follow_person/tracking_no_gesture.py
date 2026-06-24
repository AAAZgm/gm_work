#!/usr/bin/env python3
"""
人体跟随节点
参考 tROS bodyTracking C++ 源码的 ProcessSmart / DoRotateMove / TrackingSwitchWithVision 逻辑实现
消息格式链：PerceptionTargets → Target[] → Roi[] → RegionOfInterest(x_offset, y_offset, w, h)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist           # ROS2 标准速度指令消息（线速度 + 角速度）
from ai_msgs.msg import PerceptionTargets     # D-Robotics AI 感知结果消息


class FollowPersonNode(Node):
    """人体跟随节点：接收 AI 检测结果 → 计算控制量 → 发布 cmd_vel 控制机器人运动"""

    def __init__(self):
        super().__init__('follow_person_node')

        # ========== 可调参数（对应 C++ 版 TrackCfg 中的阈值） ==========
        self.declare_parameter('image_width', 640)        # 图像宽度像素（需匹配实际摄像头分辨率！）
        self.declare_parameter('image_height', 480)       # 图像高度像素
        self.declare_parameter('linear_velocity', 0.25)    # 最大前进线速度 m/s（参考官方默认值）
        self.declare_parameter('angular_velocity', 0.5)   # 最大旋转角速度 rad/s（参考官方默认值）
        self.declare_parameter('stop_move_ratio', 0.45)   # 太近停止阈值：目标框宽度占画面宽度的比例
        self.declare_parameter('activate_move_thr', 5)    # 激活阈值：连续检测到 N 帧后才开始移动（防误触发）
        self.declare_parameter('ai_msg_topic','/hobot_mono2d_body_detection')   
        # 读取参数值到实例变量
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.stop_ratio = self.get_parameter('stop_move_ratio').value
        self.activate_thr = self.get_parameter('activate_move_thr').value
        self.ai_msg_topic = self.get_parameter('ai_msg_topic').value

        # ========== 发布者：向机器人底盘发送速度指令 ==========
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ========== 订阅者：接收 AI 检测节点的感知结果 ==========
        # 对应 C++ 版 FeedSmart() 的数据来源
        self.detection_sub = self.create_subscription(
            PerceptionTargets,
            self.ai_msg_topic,   # AI 检测话题名（人体检测节点发布的话题）
            self.detection_callback,           # 回调函数：每收到一帧就调用一次
            10                                 # QoS 队列深度
        )

        # ========== 状态变量（对应 C++ 版 TrackInfo 结构体）==========
        self.twist_msg = Twist()              # 复用的 Twist 消息对象（避免每次 new）
        self.frame_count = 0                  # 总帧计数器（用于周期性打印日志）
        self.track_id = None                  # 当前跟踪目标的 track_id（C++ 版 track_info_.track_id）
        self.present_rect = None              # 当前目标的检测框 [x1, y1, x2, y2]（C++ 版 present_rect）
        self.serial_lost_num = 0              # 连续丢失帧数（C++ 版 serial_lost_num）
        self.activate_count = 0               # 连续检测到目标的激活计数器（类似 C++ 的丢帧反向逻辑）
        self.is_moving = False                # 机器人是否正在移动（用于判断是否需要发停车指令）

        self.get_logger().info('🚶 人体跟随节点已启动')
        self.get_logger().info(f'   图像尺寸: {self.img_w}x{self.img_h}')
        self.get_logger().info(f'   线速度: {self.linear_vel} m/s, 角速度: {self.angular_vel} rad/s')

    def detection_callback(self, msg):
        """
        AI 检测结果回调函数（核心处理逻辑）
        对应 C++ 版的以下函数组合：
          - ProcessSmart(): 解析感知数据、更新追踪状态
          - UpdateTrackAngle(): 计算角度偏差
          - DoRotateMove(): 执行旋转和前进
          - TrackingSwitchWithVision(): 判断是否前进
        """
        self.frame_count += 1

        # ========== 调试日志（每 30 帧打印一次，避免刷屏）==========
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'📦 收到 {len(msg.targets)} 个目标, fps={msg.fps}')

        # --- 场景 1：没有检测到任何目标 ---
        if len(msg.targets) == 0:
            self.serial_lost_num += 1                          # 丢帧计数 +1（对应 C++ 第 671 行）
            if self.serial_lost_num > 30:                      # 丢失超过 30 帧（C++ 用 track_serial_lost_num_thr 控制）
                self.stop_robot()                               # 发送零速度停车
                self.track_id = None                            # 清空跟踪 ID（对应 C++ 切换到 LOST 状态）
            return

        # ========== 目标选择策略：找最大 person 的 body 框 ==========
        # 对应 C++ 版 ProcessSmart() 中 "不需要激活手势" 分支（第 837-872 行）：
        #   直接选择宽度最大的人体（最近的目标）开始追踪
        best_rect = None                                       # 最佳检测框
        best_area = 0                                          # 最佳框面积（用于比较大小）
        best_track_id = None                                   # 最佳目标的 track_id

        for target in msg.targets:                             # 遍历所有检测目标（对应 C++ 第 491 行 for 循环）

            for roi in target.rois:                             # 遍历目标的所有 ROI 区域（body/face/hand/head 等）
                if roi.type != 'body':                          # 只关心 body 类型（对应 C++ 第 498 行）
                    continue
                # 提取检测框坐标：从 (x_offset, y_offset, width, height) 转为 (x1, y1, x2, y2)
                x1 = roi.rect.x_offset                         # 左上角 X（对应 C++ 第 501 行）
                y1 = roi.rect.y_offset                         # 左上角 Y（对应 C++ 第 502 行）
                x2 = roi.rect.x_offset + roi.rect.width        # 右下角 X（对应 C++ 第 503 行）
                y2 = roi.rect.y_offset + roi.rect.height       # 右下角 Y（对应 C++ 第 504 行）
                area = roi.rect.width * roi.rect.height         # 计算面积（用于选最大目标）

                if area > best_area:                           # 选面积最大的（= 画面中最近的目标）
                    best_area = area
                    best_rect = [x1, y1, x2, y2]
                    best_track_id = target.track_id

        # --- 场景 2：有目标但没有 body 框（可能只有 face/hand）---
        if best_rect is None:#没有
            self.serial_lost_num += 1                          # 视为丢帧
            if self.serial_lost_num > 30:
                self.stop_robot()
            return

        # ========== 更新跟踪状态（状态机转换） ==========
        # 对应 C++ 版第 659-663 行：重置丢帧计数、更新检测框
        if self.track_id is not None and self.track_id != best_track_id:
            # track_id 跳变：AI检测ID不稳定，同一个人可能换新ID
            # 不停车、不重置激活计数，因为画面里确实有人
            self.get_logger().debug(f'track_id 跳变: {self.track_id} → {best_track_id}')

        self.track_id = best_track_id                         # 更新当前跟踪 ID
        self.present_rect = best_rect                         # 更新当前检测框
        self.serial_lost_num = 0                              # 找到目标，清零丢帧计数（C++ 第 659 行）
        self.activate_count += 1                              # 激活计数 +1（连续检测到目标）

        # ========== 调试信息输出 ==========
        if self.frame_count % 30 == 0:
            x1, y1, x2, y2 = best_rect
            box_w = x2 - x1                                    # 检测框宽度
            cx = (x1 + x2) / 2                                 # 检测框中心点 X
            self.get_logger().info(
                f'🎯 track_id={best_track_id}, '
                f'框=({x1},{y1},{x2},{y2}), '
                f'宽={box_w}({box_w/self.img_w:.2f}), '        # 宽度及其占画面比例
                f'中心x={cx}/{self.img_w}, '
                f'激活帧数={self.activate_count}')

        # ========== 激活门控：连续检测不足 N 帧，不移动 ==========
        # 对应 C++ 版的 INITING → TRACKING 状态转换逻辑：
        #   需要持续检测到目标一定帧数后才认为"确实在跟踪"，防止一闪而过就乱动
        if self.activate_count < self.activate_thr:
            return                                             # 未达到激活阈值，不执行任何运动控制

        # ========== 计算控制量（核心算法）==========

        x1, y1, x2, y2 = best_rect
        box_cx = (x1 + x2) / 2.0                               # 目标框中心点 X 坐标
        box_w = x2 - x1                                        # 目标框宽度（用于距离判断）

        # ---- 角速度计算（对应 C++ UpdateTrackAngle + RotateSwitch + DoRotateMove）----
        error_angular = (box_cx - self.img_w / 2.0) / (self.img_w / 2.0)
        # 上面这行做了两件事：
        #   ① box_cx - img_w/2 : 目标中心偏离图像中心的像素距离（正=偏右，负=偏左）
        #   ② ÷ (img_w/2)      : 归一化到 [-1, 1] 范围
        # 结果含义：+1 = 目标在最右边（需要大幅左转），-1 = 目标在最左边（需要大幅右转）
        # 对应 C++ 版 angel_with_robot_ 的计算（第 112-181 行），这里用更简洁的归一化方式替代

        # ---- 线速度计算（对应 C++ TrackingSwitchWithVision 第 364-456 行）----
        # 条件 1：目标框宽度超过比例阈值 → 太近了，停止前进（对应 C++ 第 383-394 行）
        # ---- 线速度计算 ----
        # 三段式：远→全速 | 中→线性衰减到0 | 近→停止
        distance_ratio = box_w / self.img_w                # 框宽占比 [0, 1]

        # 硬停止条件（优先判断）
        too_close = (distance_ratio >= self.stop_ratio)    # 框太大 → 太近

        if too_close:
            linear_x = 0.0
            if self.frame_count % 30 == 0:
                self.get_logger().warn('⛔ 目标太近，停止前进')
        elif distance_ratio < self.stop_ratio * 0.7:
            linear_x = self.linear_vel                      # 较远 → 全速
        else:
            # 减速区：从全速线性衰减到 0
            t = (distance_ratio - self.stop_ratio * 0.7) / (self.stop_ratio * 0.3)
            linear_x = self.linear_vel * (1.0 - t)
            linear_x = max(0.05, min(self.linear_vel, linear_x))

        # ---- 角速度 P 控制输出（对应 C++ DoRotateMove 第 296 行 twist->angular.z）----
        angular_z = -self.angular_vel * error_angular          # 负号：偏差为正(偏右)时角速度为负(顺时针/右转)
        angular_z = max(-self.angular_vel, min(self.angular_vel, angular_z))  # 钳位到 ±max_angular

        # ========== 发布 cmd_vel 速度指令 ==========
        # 对应 C++ robot_cmdvel_node_->RobotCtl(*twist) （第 329 行）
        self.twist_msg.linear.x = linear_x                     # 前后线速度（正值前进）
        self.twist_msg.angular.z = angular_z                   # 偏航角速度（正值逆时针/左转，负值顺时针/右转）
        self.cmd_pub.publish(self.twist_msg)                   # 发布到 /cmd_vel 话题，底盘订阅后执行
        self.is_moving = True                                  # 标记为运动状态

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'✅ v={linear_x:.2f} m/s, w={angular_z:.2f} rad/s')

    def stop_robot(self):
        """
        停止机器人运动（发送零速度指令）
        对应 C++ 版 CancelMove() 函数（第 337-358 行）
        """
        if self.is_moving:                                     # 如果本来就在运动中，才需要发停车指令（对应 C++ 第 339 行的重复发送检查）
            self.twist_msg.linear.x = 0.0                       # 线速度归零（对应 C++ 第 344-349 行）
            self.twist_msg.angular.z = 0.0                     # 角速度归零
            self.cmd_pub.publish(self.twist_msg)               # 发布零速度（对应 C++ 第 351 行）
            self.is_moving = False                              # 标记已停止（对应 C++ 第 355 行 last_cmdvel_is_cancel_ = true）
            self.get_logger().warn('🛑 停车')


def main():
    """ROS2 节点入口函数"""
    rclpy.init()                                               # 初始化 ROS2 Python 客户端库
    node = FollowPersonNode()                                  # 创建跟随节点实例
    try:
        rclpy.spin(node)                                       # 进入事件循环，阻塞等待回调（对应 C++ executor.spin()）
    except KeyboardInterrupt:                                  # Ctrl+C 中断
        node.stop_robot()                                      # 确保停车后再退出
        node.get_logger().info('跟随节点已关闭')
    finally:
        node.destroy_node()                                    # 销毁节点资源
        rclpy.shutdown()                                       # 关闭 ROS2 客户端库


if __name__ == '__main__':
    main()
