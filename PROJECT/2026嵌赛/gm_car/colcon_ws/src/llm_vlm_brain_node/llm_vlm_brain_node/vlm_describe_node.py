#!/usr/bin/env python3
"""
VLM 视觉描述节点

链路：
    收到 /vlm_describe 服务请求 → 按需打开摄像头拍帧 → 调用 VLM 大模型分析图片 → 返回描述 → 关闭摄像头

服务：
    /vlm_describe (llm_vlm_brain_interfaces/srv/VisionDescribe)
        请求: query (string) — 问 VLM 的问题，如 "你看到了什么"
        响应: description (string) — VLM 的文字描述, result (bool) — 是否成功

按需拍照：
    服务被调用时才打开摄像头（OpenCV VideoCapture），抓取 3 帧后取最后一帧（丢弃前几帧等自动曝光稳定），
    拍完立即释放摄像头，0 资源占用。

依赖：
    pip install opencv-python requests

启动：
    ros2 run llm_vlm_brain_node vlm_describe_node

测试：
    ros2 service call /vlm_describe llm_vlm_brain_interfaces/srv/VisionDescribe "{query: '你看到了什么'}"
"""

# ===== 标准库 =====
import base64
import os
import time
from concurrent.futures import ThreadPoolExecutor

# ===== 第三方 =====
import cv2
import requests

# ===== ROS2 =====
import rclpy
from rclpy.node import Node

# ===== 自定义接口 =====
from llm_vlm_brain_interfaces.srv import VisionDescribe


class VLMDescribeNode(Node):
    """VLM 视觉语言模型节点

    工作流程：
        1. 收到 /vlm_describe 服务调用时：
           a. 按需打开摄像头，抓取 3 帧（丢弃前几帧等自动曝光稳定），取最后一帧
           b. 立即释放摄像头
           c. 将图片转为 base64，发给 llama.cpp VLM 进行分析
           d. 可选：保存当前帧到磁盘（用于调试/记录）
        2. 返回描述结果
    """

    def __init__(self):
        """初始化：参数、服务"""
        super().__init__('vlm_describe_node')

        # ---- 线程池：VLM 推理在独立线程中执行，避免阻塞 ROS2 spin（最多180秒） ----
        self._executor = ThreadPoolExecutor(max_workers=1)

        # ---- 可配置参数 ----
        self.declare_parameter('vlm_url', 'http://localhost:8080')     # llama.cpp 服务地址
        self.declare_parameter('model_name', 'qwen3')                   # VLM 模型名
        self.declare_parameter('temperature', 0.3)                      # 温度（低=更确定性）
        self.declare_parameter('max_tokens', 512)                      # 最大生成长度
        self.declare_parameter('camera_device', 0)                      # 摄像头设备号（0=/dev/video0）
        self.declare_parameter('warmup_frames', 3)                      # 预热帧数（丢弃前N帧等曝光稳定）
        self.declare_parameter('image_save_path', '/home/sunrise/gm_car/colcon_ws/src/llm_vlm_brain_node/photo')  # 图片保存路径（空=不保存）

        self.vlm_url = self.get_parameter('vlm_url').value
        self.model_name = self.get_parameter('model_name').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.camera_device = self.get_parameter('camera_device').value
        self.warmup_frames = self.get_parameter('warmup_frames').value
        self.image_save_path = self.get_parameter('image_save_path').value
        # 拼接 OpenAI 兼容 API 地址
        self.chat_api = f"{self.vlm_url.rstrip('/')}/v1/chat/completions"

        # ---- VLM 系统提示词 ----
        self.system_prompt = (
            "你是一辆智能小车的眼睛，负责描述摄像头拍到的画面。"
            "请用简洁的中文描述你看到的内容，重点描述障碍物、道路、行人、标志物等。"
            "回答不要超过4句话。"
        )

        # ---- ROS2 接口 ----
        self.create_service(VisionDescribe, 'vlm_describe', self.describe_callback)

        self.get_logger().info(
            f'VLM 视觉节点已启动 | VLM: {self.vlm_url} | '
            f'服务: /vlm_describe | 摄像头设备: {self.camera_device}'
        )

    # ==================== 按需拍照 ====================

    def _capture_image(self):
        """按需打开摄像头拍照，拍完立即关闭

        Returns:
            成功: (cv2_image, None)  — BGR 格式的 numpy 数组
            失败: (None, error_msg)  — 错误信息字符串
        """
        cap = None
        try:
            self.get_logger().info(f'正在打开摄像头 /dev/video{self.camera_device} ...')
            cap = cv2.VideoCapture(self.camera_device)
            if not cap.isOpened():
                return None, f'无法打开摄像头 /dev/video{self.camera_device}'

            # 丢弃前 warmup_frames 帧，等待自动曝光/白平衡稳定
            for i in range(self.warmup_frames):
                ret, frame = cap.read()
                if not ret:
                    return None, f'摄像头读帧失败（第 {i+1} 帧）'

            # 取最后一帧作为有效帧
            ret, frame = cap.read()
            if not ret or frame is None:
                return None, '摄像头读帧失败'

            self.get_logger().info(
                f'拍照成功 {frame.shape[1]}x{frame.shape[0]}，摄像头已释放'
            )
            return frame, None
        except Exception as e:
            return None, f'拍照异常: {e}'
        finally:
            if cap is not None:
                cap.release()

    # ==================== 服务回调 ====================

    def describe_callback(self, request, response):
        """/vlm_describe 服务回调：拍照 + 问题 → VLM 分析 → 返回描述

        使用线程池异步执行 VLM 推理，避免阻塞 ROS2 spin。
        """
        query = request.query
        self.get_logger().info(f'收到视觉描述请求: "{query}"')

        # 按需打开摄像头拍照
        cv_image, err = self._capture_image()
        if cv_image is None:
            self.get_logger().warn(f'拍照失败: {err}')
            response.description = f'摄像头不可用: {err}'
            response.result = False
            return response

        # 在线程池中执行 VLM 推理 + 图片保存
        b64_img = self._cv_image_to_base64(cv_image)
        future = self._executor.submit(self._process_vlm_request, b64_img, cv_image, query)

        try:
            desc = future.result(timeout=180)
        except Exception as e:
            self.get_logger().error(f'VLM 线程执行异常: {e}')
            desc = f"抱歉，视觉处理出错: {e}"

        response.description = desc
        response.result = bool(desc and not desc.startswith('抱歉'))
        return response

    def _process_vlm_request(self, base64_image: str, cv_image, query: str) -> str:
        """在线程池中执行：调用 VLM → 保存图片"""
        desc = self._call_vlm(base64_image, query)
        self.get_logger().info(f'VLM 描述: "{desc}"')
        self._save_cv_image(cv_image)
        return desc

    # ==================== 内部方法 ====================

    def _cv_image_to_base64(self, cv_image) -> str:
        """将 OpenCV BGR 图像转换为 base64 编码的 JPEG 字符串"""
        success, buffer = cv2.imencode('.jpg', cv_image)
        if not success:
            raise ValueError("图片编码失败")
        return base64.b64encode(buffer).decode('utf-8')

    def _call_vlm(self, base64_image: str, question: str) -> str:
        """调用 llama.cpp VLM 多模态大模型分析图片

        使用 OpenAI 兼容的多模态 API 格式（图片作为 image_url 内嵌）。

        Args:
            base64_image: base64 编码的 JPEG 图片
            question: 问 VLM 的问题（如 "你看到什么"、"有红色物体吗"）

        Returns:
            VLM 生成的描述文字（出错时返回以"抱歉"开头的提示语）
        """
        # 构造多模态消息：文本 + 图片 URL
        image_url = f"data:image/jpeg;base64,{base64_image}"
        messages = [
            {"role": "system", "content": self.system_prompt},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": question},
                    {"type": "image_url", "image_url": {"url": image_url}},
                ],
            },
        ]
        try:
            resp = requests.post(
                self.chat_api,
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": False,
                    "temperature": self.temperature,
                    "max_tokens": self.max_tokens,
                    "chat_template_kwargs": {"thinking": False},
                },
                timeout=180,   # VLM 图片推理可能较慢，给 3 分钟超时
            )
            resp.raise_for_status()
            msg = resp.json()['choices'][0]['message']
            content = msg.get('content', '').strip()
            reasoning = msg.get('reasoning_content', '').strip()
            # Qwen3 思考模式：content 为空时，用 reasoning_content 作为回复
            return reasoning if not content and reasoning else content
        except requests.exceptions.Timeout:
            return "抱歉，视觉模型思考太久了，请再试一次。"
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'无法连接 VLM 服务: {self.vlm_url}')
            return "抱歉，无法连接视觉模型服务。"
        except Exception as e:
            self.get_logger().error(f'VLM 调用出错: {e}')
            return f"抱歉，调用视觉模型时出错: {e}"

    def _save_cv_image(self, cv_image):
        """将 OpenCV 图像保存为 PNG 文件（用于调试/记录）"""
        if not self.image_save_path:
            return
        try:
            os.makedirs(self.image_save_path, exist_ok=True)
            current_time = time.strftime("%Y%m%d_%H%M%S")
            save_filename = os.path.join(self.image_save_path, f'image_{current_time}.png')
            cv2.imwrite(save_filename, cv_image)
        except Exception as e:
            self.get_logger().warn(f'保存图片失败: {e}')


# ==================== 主入口 ====================

def main(args=None):
    """ROS2 节点标准入口"""
    rclpy.init(args=args)
    node = VLMDescribeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
