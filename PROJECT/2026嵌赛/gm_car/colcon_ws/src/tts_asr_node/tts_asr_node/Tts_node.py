#!/usr/bin/env python3
"""
TTS 语音合成节点 (edge-tts)

功能：
    接收文字 → 调用微软 edge-tts 引擎合成语音 → 通过 mpg123 播放

服务：
    /tts (tts_asr_interfaces/srv/Tts)
        请求: text (string) — 要合成的文字
        响应: result (bool) — 是否成功播报

话题发布：
    /tts_status (std_msgs/String) — 播报状态通知
        'speaking' — 正在说话（ASR 收到后暂停录音防回声）
        'idle'     — 说完空闲（ASR 收到后恢复录音）

依赖：
    pip install edge-tts          # 微软免费 TTS 引擎（Azure 同源）
    sudo apt install mpg123       # 命令行 MP3 播放器
"""

# ===== 标准库 =====
import asyncio
import os
import socket
import subprocess
import threading
import uuid

# ===== 强制 edge-tts 走 IPv4（解决 IPv6 SSL 被墙问题） =====
import aiohttp
_orig_tcp = aiohttp.TCPConnector.__init__
def _patched_tcp_init(self, *args, **kwargs):
    kwargs.setdefault('family', socket.AF_INET)
    _orig_tcp(self, *args, **kwargs)
aiohttp.TCPConnector.__init__ = _patched_tcp_init

# ===== ROS2 =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tts_asr_interfaces.srv import Tts

# ===== 第三方 =====
import edge_tts


class TtsNode(Node):
    """TTS 语音合成节点（支持并发安全）

    工作流程：
        1. 收到 /tts 服务请求（文字内容）
        2. 加入队列，等待前一个TTS完成（互斥锁保证串行执行）
        3. 发布 'speaking' 到 /tts_status → ASR 暂停录音
        4. edge-tts 将文字合成 MP3 音频文件（使用唯一文件名避免冲突）
        5. mpg123 播放 MP3
        6. 发布 'idle' 到 /tts_status → ASR 恢复录音
        7. 返回成功/失败
    """

    def __init__(self):
        """初始化：创建 TTS 服务 + 状态发布者 + 互斥锁"""
        super().__init__('tts')

        # 提供语音合成服务（ASR/LLM/VLM 节点都通过调用此服务让小车"说话"）
        self.srv = self.create_service(Tts, 'tts', self.tts_callback)
        # 播报状态发布者：通知 ASR 当前是否在说话（用于回声抑制）
        self.status_pub = self.create_publisher(String, '/tts_status', 10)

        # ===== 并发控制：互斥锁 =====
        self._lock = threading.Lock()       # 保证TTS串行执行
        self._queue = []                    # 等待队列（用于日志）
        self._is_speaking = False           # 当前是否正在播放

        self.get_logger().info('语音服务已启动 (edge-tts) [支持并发安全]')

    # ==================== 服务回调 ====================

    def tts_callback(self, request, response):
        """/tts 服务回调：文字 → 合成语音 → 播放 → 返回结果

        特性：
            - 使用互斥锁确保同一时间只有一个TTS在执行
            - 使用唯一临时文件名避免文件冲突
            - 快速连续调用会自动排队串行执行

        Args:
            request: Tts.Request，request.text 为要朗读的文字
            response: Tts.Response，response.result 表示是否成功
        """
        text = request.text.strip()
        if not text:
            response.result = False
            return response

        self.get_logger().info(f"接收到文字：{text}")

        # ===== 获取锁（如果正在播放则等待） =====
        acquired = self._lock.acquire(timeout=0.1)
        if not acquired:
            self.get_logger().warn(f'TTS忙，排队等待: "{text[:20]}..."')
            # 使用更长超时等待锁
            acquired = self._lock.acquire(timeout=30.0)
            if not acquired:
                self.get_logger().error(f'TTS等待超时，放弃: "{text[:20]}..."')
                response.result = False
                return response

        try:
            # ① 通知 ASR："我要开始说话了，你先别录音"
            self._is_speaking = True
            self._publish_status('speaking')

            # ② edge-tts 文字→MP3，然后 mpg123 播放（使用唯一文件名）
            tmp_file = f"/tmp/edge_tts_{uuid.uuid4().hex[:8]}.mp3"
            try:
                asyncio.run(self._speak(text, tmp_file))
                response.result = True
                self.get_logger().info(f'✓ TTS播报成功: "{text[:30]}"')
            except Exception as e:
                self.get_logger().error(f'语音播报失败: {e}')
                response.result = False
        finally:
            # ③ 通知 ASR："我说完了，你可以恢复录音了"
            self._is_speaking = False
            self._publish_status('idle')
            # ④ 释放锁，让下一个等待的请求可以执行
            self._lock.release()

        return response

    # ==================== 内部方法 ====================

    async def _speak(self, text: str, tmp_file: str):
        """调用 edge-tts 合成语音并用 mpg123 播放

        流程：
            1. edge-tts 将 text 合成为 MP3，保存到临时文件（唯一文件名）
            2. mpg123 -q 安静模式播放该 MP3
            3. 播完后删除临时文件

        Args:
            text: 要朗读的文字内容
            tmp_file: 唯一的临时文件路径（避免并发冲突）
        """
        voice = "zh-CN-XiaoxiaoNeural"   # 微软中文女声（晓晓），自然流畅

        try:
            communicate = edge_tts.Communicate(text, voice)
            await communicate.save(tmp_file)                       # 合成 MP3
            subprocess.run(["mpg123", "-q", tmp_file], check=True) # -q = 安静模式
            self.get_logger().info('TTS合成+播放成功')
        finally:
            # 清理临时文件（即使播放失败也要清理）
            if os.path.exists(tmp_file):
                os.remove(tmp_file)

    def _publish_status(self, status: str):
        """向 /tts_status 话题发布当前播报状态

        Args:
            status: 'speaking'（正在说话）或 'idle'（空闲）
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


# ==================== 主入口 ====================

def main(args=None):
    """ROS2 节点标准入口"""
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
