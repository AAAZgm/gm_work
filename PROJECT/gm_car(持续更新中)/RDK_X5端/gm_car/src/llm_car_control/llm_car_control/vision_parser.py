#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#from hobot_dnn_msgs.msg import DnnDetection  # 官方YOLO检测消息类型
from ai_msgs.msg import PerceptionTargets
import json

class VisionParserNode(Node):
    def __init__(self):
        super().__init__("vision_parser_node")
        
        # 从配置文件加载参数
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic.dnn_detection", "/hobot_dnn_detection"),
                ("topic.vision_desc", "/car/vision_desc"),
                ("topic.object_coords", "/car/object_coords"),
                ("model.yolo_conf", 0.5),
                ("car.img_width", 640),
                ("car.img_height", 640)
            ]
        )
        self.dnn_detection_topic = self.get_parameter("topic.dnn_detection").value
        self.vision_desc_topic = self.get_parameter("topic.vision_desc").value
        self.object_coords_topic = self.get_parameter("topic.object_coords").value
        self.yolo_conf = self.get_parameter("model.yolo_conf").value
        self.img_width = self.get_parameter("car.img_width").value
        self.img_height = self.get_parameter("car.img_height").value

        # 缓存最新检测结果
        self.latest_objects = []

        # ROS2订阅/发布器
        
        self.sub_dnn_detection = self.create_subscription(
            PerceptionTargets,  # 原来是 DnnDetection
            self.dnn_detection_topic, S
            self.dnn_callback, 
            10
        )
        self.pub_vision_desc = self.create_publisher(
            String, self.vision_desc_topic, 10
        )
        self.pub_object_coords = self.create_publisher(
            String, self.object_coords_topic, 10
        )

        self.get_logger().info("✅ 官方YOLOv8解析节点启动成功")
        # 回调函数需要重新适配消息结构
    def dnn_callback(self, msg):
        objects = []
        for target in msg.targets:
            # 过滤低置信度结果
            for roi in target.rois:
                if roi.confidence < self.yolo_conf:
                    continue
                
                # 解析检测框
                rect = roi.rect  # sensor_msgs/RegionOfInterest
                x1 = rect.x_offset
                y1 = rect.y_offset
                x2 = x1 + rect.width
                y2 = y1 + rect.height
                
                # ... 后续逻辑
    
            # 计算中心坐标和偏移
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            cx_offset = cx - self.img_width / 2
            cy_offset = cy - self.img_height / 2

            # 封装物体信息（和原方案格式一致，保证LLM兼容）
            obj_info = {
                "class": detection.label,  # 官方返回的类别名（如person, bottle）
                "confidence": round(detection.score, 2),
                "bbox": [x1, y1, x2, y2],
                "center": [cx, cy],
                "offset": [cx_offset, cy_offset],
                "size": {"width": x2 - x1, "height": y2 - y1}
            }
            objects.append(obj_info)

        # 更新缓存
        self.latest_objects = objects

        # 发布物体坐标
        self.pub_object_coords.publish(
            String(data=json.dumps(objects, ensure_ascii=False))
        )

        # 生成自然语言视觉描述（给LLM决策）
        if len(objects) == 0:
            vision_desc = "画面中未检测到任何物体"
        else:
            vision_desc = f"画面中检测到{len(objects)}个物体："
            for obj in objects:
                pos_desc = "中心" if abs(obj["offset"][0]) < 20 else \
                           "左侧" if obj["offset"][0] < 0 else "右侧"
                vision_desc += f"{obj['class']}（置信度{obj['confidence']}，位于画面{pos_desc}，偏移{obj['offset'][0]:.0f}像素）；"

        # 发布视觉描述
        self.pub_vision_desc.publish(String(data=vision_desc))
        self.get_logger().debug(f"📸 视觉描述：{vision_desc}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()