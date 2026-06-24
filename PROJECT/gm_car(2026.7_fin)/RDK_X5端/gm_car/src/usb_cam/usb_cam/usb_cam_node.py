#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # 声明参数
        self.declare_parameter('camera_device', 0)  # /dev/video0,可用ffpaly测试
        self.declare_parameter('frame_id', 'camera_link')
        
        # 获取参数
        self.device = self.get_parameter('camera_device').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # 创建发布者
        # 原始图像
        self.raw_publisher = self.create_publisher(Image, 'camera/image_raw', 10)

#        self.vla_publisher = self.create_publisher(Image, 'camera/image_for_vla', 10)     
        # 初始化 CvBridge
        self.bridge = CvBridge()
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.device)#这是索引
        
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头设备 {self.device}')
            return
        
        # 设置摄像头参数（可选）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 创建定时器，20fps → 间隔0.05秒
        self.timer = self.create_timer(0.05, self.timer_callback)
#        self.timer_2 = self.create_timer(30, self.timer_callback_2)
        self.get_logger().info(f'USB 摄像头节点已启动，设备: {self.device}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('无法读取摄像头帧')
            return
        
        # ===== OpenCV 处理 =====
        #  这里添加你的图像处理代码
        # processed_frame = self.process_image(frame)
        
        # ===== 发布原始图像 =====
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.frame_id = self.frame_id
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_publisher.publish(raw_msg)
        
        # # ===== 发布处理后图像 =====
        # processed_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
        # processed_msg.header.frame_id = self.frame_id
        # processed_msg.header.stamp = self.get_clock().now().to_msg()
        # self.processed_publisher.publish(processed_msg)

    # def timer_callback_2(self):
    #     ret, frame = self.cap.read()
        
    #     if not ret:
    #         self.get_logger().warn('无法读取摄像头帧')
    #         return
        
    #     # ===== 发布原始图像 =====
    #     vla_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    #     vla_msg.header.frame_id = self.frame_id
    #     vla_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.vla_publisher.publish(vla_msg)

    # # def process_image(self, frame):
    # #     """图像处理函数 - 根据需要修改"""
        
    # #     # 示例 1：灰度转换后转回 BGR（便于显示）
    # #     # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # #     # processed = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
    # #     # 示例 2：高斯模糊
    # #     # processed = cv2.GaussianBlur(frame, (15, 15), 0)
        
    # #     # 示例 3：边缘检测
    # #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # #     edges = cv2.Canny(gray, 100, 200)#图像，低阈值，高阈值
    # #     processed = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
    # #     # 示例 4：画个框（测试用）
    # #     # cv2.rectangle(processed, (100, 100), (300, 300), (0, 255, 0), 2)
        
    # #     return processed

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
