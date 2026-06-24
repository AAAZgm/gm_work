#!/usr/bin/env python3
"""
摄像头物体轮廓标注显示节点
订阅 /camera/image_raw，按颜色检测物体，不同颜色物体用不同轮廓颜色标注，cv2.imshow 显示
"""
import signal
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# --- 颜色HSV检测范围 + 轮廓绘制用的BGR颜色 ---
COLOR_MAP = {
    'red':    {'hsv': ([0, 100, 100],   [10, 255, 255]),  'bgr': (0, 0, 255)},
    'orange': {'hsv': ([10, 100, 100],  [20, 255, 255]),  'bgr': (0, 165, 255)},
    'yellow': {'hsv': ([20, 100, 100],  [35, 255, 255]),  'bgr': (0, 255, 255)},
    'green':  {'hsv': ([35, 100, 100],  [85, 255, 255]),  'bgr': (0, 255, 0)},
    'blue':   {'hsv': ([100, 100, 100], [130, 255, 255]), 'bgr': (255, 0, 0)},
    'white':  {'hsv': ([0, 0, 200],     [180, 50, 255]),  'bgr': (255, 255, 255)},
}

# 红色 H 跨两端，额外一段
RED_HSV2 = ([170, 100, 100], [180, 255, 255])

MIN_AREA = 500  # 最小轮廓面积，过滤噪点

# 画面中心十字线
CROSS_COLOR = (128, 128, 128)


class VisionDisplay(Node):
    def __init__(self):
        super().__init__('vision_display')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10)
        self.get_logger().info('视觉显示节点已启动，按 q 退出窗口')

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        display = frame.copy()
        h, w = display.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 遍历每种颜色，检测并标注
        for name, cfg in COLOR_MAP.items():
            lower = np.array(cfg['hsv'][0], dtype=np.uint8)
            upper = np.array(cfg['hsv'][1], dtype=np.uint8)
            bgr_color = cfg['bgr']

            # 红色特殊处理：合并两段
            if name == 'red':
                mask1 = cv2.inRange(hsv, lower, upper)
                mask2 = cv2.inRange(hsv, np.array(RED_HSV2[0], dtype=np.uint8),
                                         np.array(RED_HSV2[1], dtype=np.uint8))
                mask = mask1 | mask2
            else:
                mask = cv2.inRange(hsv, lower, upper)

            # 形态学去噪
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)#先腐蚀后膨胀去鼓励
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)#反之去空洞

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#只取外并节省内存

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue
                # 画轮廓
                cv2.drawContours(display, [cnt], -1, bgr_color, 2)#cnt是物体边缘
                # 画中心点 + 标签
                M = cv2.moments(cnt)#空间矩
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(display, (cx, cy), 4, bgr_color, -1)
                    cv2.putText(display, f'{name}({area:.0f})',
                                (cx - 30, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

        # 画面中心十字线
        cv2.drawMarker(display, (w // 2, h // 2), CROSS_COLOR,
                        cv2.MARKER_CROSS, 20, 1)

        # 左上角提示
        cv2.putText(display, 'q: quit', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow('Object Detection', display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('用户关闭显示窗口')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = VisionDisplay()

    # Ctrl+C 时确保触发 KeyboardInterrupt，走到 finally 关掉 OpenCV 窗口
    signal.signal(signal.SIGINT, lambda sig, frame: (_ for _ in ()).throw(KeyboardInterrupt))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
