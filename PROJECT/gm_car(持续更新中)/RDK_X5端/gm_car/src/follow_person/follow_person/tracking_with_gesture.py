#!/usr/bin/env python3
"""
简单手势控制节点
订阅手势检测结果，根据手势类型直接控制机器人运动
手势映射：
  其他/无检测  → 停止
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ai_msgs.msg import PerceptionTargets
# 手势枚举值（对应 C++ GestureCtrlType）
GESTURE_ThumbUp = 2
GESTURE_Palm = 5
GESTURE_THUMB_RIGHT = 12
GESTURE_THUMB_LEFT = 13
class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        # ========== 可调参数 ==========
        self.declare_parameter('move_step', 0.2)     # 前进/后退线速度 m/s
        self.declare_parameter('rotate_step', 1.0)   # 左转/右转角速度 rad/s
        self.declare_parameter('lost_timeout', 30)   # 连续丢失多少帧后停止跟踪
        self.declare_parameter('ai_msg_topic','/hobot_mono2d_body_detection')
        self.move_step = self.get_parameter('move_step').value
        self.rotate_step = self.get_parameter('rotate_step').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.ai_msg_topic = self.get_parameter('ai_msg_topic').value
        # ========== 发布/订阅 ==========
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gesture_sub = self.create_subscription(
            PerceptionTargets,
            self.ai_msg_topic,
            self.gesture_callback,
            10
        )
        # ========== 状态 ==========
        self.track_id = None          # 当前跟踪的手的 track_id
        self.lost_count = 0           # 连续丢失帧数
        self.is_moving = False        # 防止重复发零速度
        self.twist = Twist()          # 复用消息对象
        self.frame_count = 0
        self.get_logger().info('手势控制节点已启动')
        self.get_logger().info(f'  前进/后退速度: {self.move_step} m/s')
        self.get_logger().info(f'  左转/右转速度: {self.rotate_step} rad/s')
    def gesture_callback(self, msg: PerceptionTargets):
        self.frame_count += 1
        # ===== 1. 没有检测到任何目标 → 丢帧计数 =====
        if not msg.targets:
            self.lost_count += 1
            if self.lost_count > self.lost_timeout:
                self.stop()
            return
        # ===== 2. 找到有 hand ROI + gesture 属性的目标 =====
        best_target = None
        best_area = 0
        for target in msg.targets:#target里面找handroi再找对应属性attr
            # 找 hand 类型的 ROI
            hand_rect = None
            for roi in target.rois:
                if roi.type == 'hand':
                    hand_rect = roi.rect
                    break
            if hand_rect is None:
                continue
            # 找 gesture 类型的属性
            gesture_val = -1
            for attr in target.attributes:
                if attr.type == 'gesture':
                    gesture_val = attr.value
                    break
            if gesture_val == -1:
                continue
            # 选面积最大的手（最近的手）
            area = hand_rect.width * hand_rect.height
            if area > best_area:
                best_area = area
                best_target = (target.track_id, gesture_val)
        # ===== 3. 没找到有效的手势目标 → 丢帧 =====
        if best_target is None:
            self.lost_count += 1
            if self.lost_count > self.lost_timeout:
                self.stop()
            return
        # ===== 4. 找到了目标 → 更新跟踪状态 =====
        found_id, gesture = best_target
        if self.track_id is not None and self.track_id != found_id:
            # 目标切换了，先停车等稳定
            self.stop()
            self.track_id = found_id
            self.lost_count = 0
            return
        self.lost_count = 0
        # ===== 5. 根据手势执行动作 =====
        if gesture == GESTURE_ThumbUp:
            self.move_forward()
        elif gesture == GESTURE_Palm:
            self.move_backward()
        elif gesture == GESTURE_THUMB_RIGHT:
            self.rotate_right()
        elif gesture == GESTURE_THUMB_LEFT:
            self.rotate_left()
        else:
            self.stop()
        if self.frame_count % 30 == 0:
            gesture_names = {
                GESTURE_ThumbUp: '(前进)',
                GESTURE_Palm: '(后退)',
                GESTURE_THUMB_RIGHT: 'ThumbRight(右转)',
                GESTURE_THUMB_LEFT: 'ThumbLeft(左转)',
            }
            name = gesture_names.get(gesture, f'Unknown({gesture})')#如果找到了，就返回对应的名字如果没找到，就返回 Unknown(xxx)
            self.get_logger().info(f'手势: {name}')
    def move_forward(self):
        self.twist.linear.x = self.move_step
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        self.is_moving = True
    def move_backward(self):
        self.twist.linear.x = -self.move_step
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        self.is_moving = True
    def rotate_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.rotate_step
        self.cmd_pub.publish(self.twist)
        self.is_moving = True
    def rotate_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.rotate_step
        self.cmd_pub.publish(self.twist)
        self.is_moving = True
    def stop(self):
        if self.is_moving:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            self.is_moving = False
def main():
    rclpy.init()
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info('手势控制节点已关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()