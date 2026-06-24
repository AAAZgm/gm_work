#!/usr/bin/env python3
"""
ASR 语音识别节点 (Vosk 流式 + PyAudio + VAD/RMS 版)

功能：
    PyAudio 流式录音 → RMS+VAD 语音活动检测 → Vosk 流式识别 → 发布结果

核心改进：
    1. 替换 arecord 为 PyAudio 流式录音（零文件IO，内存直送）
    2. RMS 能量 + 静音超时双阈值 VAD（自动检测说话起止，不再傻等）
    3. 结合 /tts_status 的回声抑制 + RMS 回声门控双重防护
    4. Vosk AcceptWaveform 流式识别（边录边识）

工作模式：
    模式1 — /asr 服务：外部调用触发（兼容旧接口，也改为流式）
    模式2 — 唤醒词循环：后台线程监听 → 唤醒词命中 → 进入连续对话
    模式3 — 手动模式：按空格键触发ASR（通过 /asr_manual_trigger 话题）

服务：
    /asr (tts_asr_interfaces/srv/Asr)
        请求: listen_time (int16) — 最大录音时长(秒)
        响应: text, result

话题：
    /asr_result            (std_msgs/String) — 识别结果
    /wake_word_detected    (std_msgs/String) — 唤醒事件
    /asr_manual_trigger    (std_msgs/String) [订阅] — 手动模式触发（按空格键时由外部发布）
    /tts_status            (std_msgs/String) [订阅] — speaking/idle

依赖：
    pip install vosk pyaudio
    sudo apt install portaudio19-dev   # pyaudio 编译依赖
"""

# ===== 标准库 =====
import json
import math
import os
import struct
import sys
import threading
import time

# ===== ROS2 =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tts_asr_interfaces.srv import Asr
from tts_asr_interfaces.srv import Tts

# ===== 第三方 =====
import pyaudio
try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    Model = None
    KaldiRecognizer = None


class VAD:
    """基于 RMS 能量的轻量级语音活动检测器

    原理：
        对每个音频帧计算均方根(RMS)能量值，与动态阈值比较判断是否有人声。
        支持静音超时自动截断（说完整句后立刻返回，不用等固定时长）。
    """

    def __init__(self,
                 sample_rate: int = 16000,
                 frame_ms: int = 30,
                 rms_threshold: float = 300,
                 silence_timeout: float = 0.6,
                 min_speech_frames: int = 10,
                 pre_speech_frames: int = 5):
        """
        Args:
            sample_rate:      采样率 Hz
            frame_ms:         每帧长度 ms（越小响应越快，但 CPU 开销略增）
            rms_threshold:    RMS 触发阈值（低于此值视为静默，需根据设备调整）
            silence_timeout:  连续静默多少秒后判定说话结束
            min_speech_frames: 最少连续语音帧数才确认开始说话（防瞬态噪音误触发）
            pre_speech_frames: 检测到说话后往前保留的帧数（不丢开头几个字）
        """
        self.sample_rate = sample_rate
        self.frame_size = int(sample_rate * frame_ms / 1000)  # 每帧采样数
        self.rms_threshold = rms_threshold
        self.silence_timeout = silence_timeout
        self.min_speech_frames = min_speech_frames
        self.pre_speech_frames = pre_speech_frames

        self.reset()

    def reset(self):
        """重置状态，准备新一轮检测"""
        self.state = 'idle'          # idle | listening | speech_detected
        self.ring_buffer = []        # 环形缓冲区，存储最近 N 帧（用于保留话前音频）
        self.speech_buffer = []      # 已确认的语音帧列表
        self.silence_frame_count = 0 # 连续静默帧计数
        self.total_speech_samples = 0

    def compute_rms(self, audio_bytes: bytes) -> float:
        """计算音频数据的 RMS（均方根）能量值

        Args:
            audio_bytes: PCM S16_LE 格式的原始音频字节

        Returns:
            RMS 浮点数值（范围 0~32767）
        """
        if len(audio_bytes) == 0:
            return 0.0
        count = len(audio_bytes) // 2
        samples = struct.unpack(f'<{count}h', audio_bytes)
        sum_sq = sum(s * s for s in samples)
        return math.sqrt(sum_sq / count) if count > 0 else 0.0

    def process_frame(self, audio_bytes: bytes) -> str:
        """处理一帧音频数据，返回状态事件

        Args:
            audio_bytes: 一帧 PCM S16_LE 音频数据（长度 = frame_size * 2 字节）

        Returns:
            事件字符串：
                ''           — 无事件，继续录音
                'speech_start'  — 检测到语音开始
                'speech_end'    — 检测到语音结束（收集到的语音数据可通过 get_speech() 获取）
        """
        rms = self.compute_rms(audio_bytes)

        # 将帧存入环形缓冲区
        self.ring_buffer.append(audio_bytes)
        if len(self.ring_buffer) > self.pre_speech_frames + 5:
            self.ring_buffer.pop(0)

        if self.state == 'idle':
            # --- 等待语音 ---
            if rms > self.rms_threshold:
                # 可能是语音，进入 listening 状态验证
                self.state = 'listening'
                self.silence_frame_count = 0
                # 把环形缓冲区中的前几帧也加入语音缓冲（保留开头）
                start_idx = max(0, len(self.ring_buffer) - self.pre_speech_frames)
                for f in self.ring_buffer[start_idx:]:
                    self.speech_buffer.append(f)
                    self.total_speech_samples += len(f) // 2
                self.speech_buffer.append(audio_bytes)
                self.total_speech_samples += len(audio_bytes) // 2
            return ''

        elif self.state == 'listening':
            frame_duration = len(audio_bytes) / 2 / self.sample_rate
            self.speech_buffer.append(audio_bytes)
            self.total_speech_samples += len(audio_bytes) // 2

            if rms > self.rms_threshold:
                # 仍在说话
                self.silence_frame_count = 0
                if len(self.speech_buffer) >= self.min_speech_frames:
                    self.state = 'speech_detected'
                    return 'speech_start'
                return ''
            else:
                # 静默 — 可能是短暂的停顿
                self.silence_frame_count += 1
                if self.silence_frame_count * frame_duration >= self.silence_timeout:
                    # 静默太久了，还没达到最小语音长度，算作噪音，重置
                    self.reset()
                    return ''
                return ''

        elif self.state == 'speech_detected':
            frame_duration = len(audio_bytes) / 2 / self.sample_rate
            self.speech_buffer.append(audio_bytes)
            self.total_speech_samples += len(audio_bytes) // 2

            if rms > self.rms_threshold:
                self.silence_frame_count = 0
                return ''
            else:
                self.silence_frame_count += 1
                # 连续静默超过阈值，判定说话结束
                if self.silence_frame_count * frame_duration >= self.silence_timeout:
                    self.state = 'idle'
                    return 'speech_end'
                return ''

        return ''

    def get_speech(self) -> bytes:
        """获取已收集的所有语音音频数据（PCM S16_LE）"""
        result = b''.join(self.speech_buffer)
        self.reset()
        return result

    def is_idle(self) -> bool:
        return self.state == 'idle'


class StreamRecognizer:
    """PyAudio 流式录音 + Vosk 流式识别 + VAD 组合引擎

    将三个组件串联：
        PyAudio InputStream → 逐帧回调 → [TTS回声过滤] → [VAD] → [Vosk AcceptWaveform]
    """

    def __init__(self, node: 'AsrNode', manual_mode: bool = False):
        self.node = node
        self._manual_mode = manual_mode

        # ---- 音频参数 ----
        self.sample_rate = node.sample_rate       # 16000
        self.channels = node.channels             # 1 (单声道)
        self.format = pyaudio.paInt16             # 16位有符号整数
        self.frame_ms = 30                        # 每帧 30ms（平衡延迟与CPU）
        self.chunk = int(self.sample_rate * self.frame_ms / 1000)  # 每帧采样数

        # ---- VAD 参数 ----
        self.vad_rms_threshold = node.get_parameter('vad_rms_threshold').value
        self.vad_silence_timeout = node.get_parameter('vad_silence_timeout').value
        self.max_record_sec = node.get_parameter('max_record_sec').value

        # ---- PyAudio ----
        self.pa = None
        self.stream = None

    def _create_vad(self) -> VAD:
        return VAD(
            sample_rate=self.sample_rate,
            frame_ms=self.frame_ms,
            rms_threshold=self.vad_rms_threshold,
            silence_timeout=self.vad_silence_timeout,
            min_speech_frames=8,      # 至少 240ms 语音才算开始
            pre_speech_frames=6,      # 保留前 180ms
        )

    def _is_tts_echo(self) -> bool:
        """检查当前是否应该跳过录音（TTS 正在说话或刚结束的缓冲期内）"""
        if self.node._tts_speaking:
            return True
        # TTS 刚结束后留缓冲期让余音消散
        if (self.node._tts_speaking_time > 0 and
            time.time() - self.node._tts_speaking_time < 0.5):
            return True
        return False

    def record_and_recognize(self, timeout_sec: float = None) -> str:
        """流式录音 + VAD 截断 + Vosk 流式识别，一次性完成

        Args:
            timeout_sec: 最大总录音时长秒数（None 则使用参数默认值）

        Returns:
            识别出的文本字符串
        """
        if KaldiRecognizer is None:
            self.node.get_logger().error('[流式录音] Vosk 未安装或模型未加载')
            return ''
        if self.node.vosk_model is None:
            self.node.get_logger().warn('[流式录音] Vosk 模型未加载，跳过识别')
            return ''

        rec = KaldiRecognizer(self.node.vosk_model, self.sample_rate)
        vad = self._create_vad()
        result_text = ""
        speech_started = False
        event = ""

        max_timeout = timeout_sec or self.max_record_sec
        start_time = time.time()

        try:
            self.pa = pyaudio.PyAudio()
            self.stream = self.pa.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk,
            )

            self.node.get_logger().info(
                f'[流式录音] 开始 | VAD阈值={self.vad_rms_threshold} '
                f'静默超时={self.vad_silence_timeout}s 最大时长={max_timeout}s')

            while True:
                # 超时检查
                elapsed = time.time() - start_time
                if elapsed > max_timeout:
                    self.node.get_logger().info(
                        f'[流式录音] 达到最大时长 {max_timeout:.1f}s')
                    break

                # PTT 检测：仅手动模式下生效，松开空格时停止录音
                if self._manual_mode and not self.node._manual_trigger_event.is_set():
                    self.node.get_logger().info(
                        '[流式录音] 检测到松开空格，停止录音开始识别')
                    break

                try:
                    data = self.stream.read(self.chunk, exception_on_overflow=False)
                except OSError as e:
                    self.node.get_logger().warn(f'[流式录音] 读音频异常: {e}')
                    continue

                # ===== 回声抑制门控：TTS 说话时丢弃音频 =====
                if self._is_tts_echo():
                    # 处于 TTS 回声期，不送入 VAD 也不送入 Vosk，
                    # 同时重置 VAD 状态避免残留噪音累积触发
                    if not vad.is_idle():
                        vad.reset()
                    continue

                # ===== VAD 处理 =====
                event = vad.process_frame(data)

                if event == 'speech_start':
                    speech_started = True
                    self.node.get_logger().debug('[VAD] 语音开始')

                elif event == 'speech_end':
                    self.node.get_logger().info(
                        f'[VAD] 语音结束 ({elapsed:.1f}s)，开始最终识别')
                    # 将收集到的所有语音数据一次性送入 Vosk 做最终识别
                    speech_data = vad.get_speech()
                    result_text = self._recognize_once(rec, speech_data)
                    break

                # ===== 如果没有 VAD 或 VAD 已检测到语音，实时送入 Vosk =====
                if speech_started and event != 'speech_end':
                    if rec.AcceptWaveform(data):
                        res = json.loads(rec.Result())
                        partial = res.get('text', '')
                        if partial:
                            result_text += partial
                            self.node.get_logger().debug(f'[Vosk中间结果] "{partial}"')

            # 如果是超时退出且有中间结果，取 FinalResult
            if not result_text or (speech_started and event != 'speech_end'):
                res = json.loads(rec.FinalResult())
                final = res.get('text', '')
                if final:
                    result_text = final

        except Exception as e:
            self.node.get_logger().error(f'[流式录音] 异常: {e}')
        finally:
            self._close_stream()

        return result_text.strip()

    def _recognize_once(self, recognizer, audio_data: bytes) -> str:
        """将一段完整的 PCM 数据送入新的 Vosk 识别器做一次性识别"""
        rec_new = KaldiRecognizer(self.node.vosk_model, self.sample_rate)
        text = ""
        offset = 0
        frame_size = 4000 * 2  # 每次 4000 样本 (约250ms)
        while offset < len(audio_data):
            chunk = audio_data[offset:offset + frame_size]
            if rec_new.AcceptWaveform(chunk):
                res = json.loads(rec_new.Result())
                text += res.get('text', '')
            offset += frame_size
        res = json.loads(rec_new.FinalResult())
        text += res.get('text', '')
        return text.strip()

    def _close_stream(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        if self.pa:
            self.pa.terminate()
            self.pa = None


class AsrNode(Node):
    """ASR 语音识别节点 (Vosk 流式 + PyAudio + VAD 版本)

    提供两种使用方式：
      1. 同步服务调用：ros2 service call /asr ...
      2. 唤醒词对话：后台线程循环流式录音，检测唤醒词后进入连续对话模式
    """

    def __init__(self):
        """初始化节点"""
        super().__init__('asr_node')

        # ---- 录音参数 ----
        self.sample_rate = 16000
        self.channels = 1

        # ---- ROS2 通信接口 ----
        self.srv = self.create_service(Asr, 'asr', self.asr_callback)
        self.result_pub = self.create_publisher(String, '/asr_result', 10)
        self.wake_pub = self.create_publisher(String, '/wake_word_detected', 10)
        self.tts_client = self.create_client(Tts, 'tts')
        self.create_subscription(String, '/tts_status', self._tts_status_callback, 10)

        # ---- 包根目录（绝对路径）----
        self._pkg_dir = '/home/gm_ubuntu/Desktop/gm_car/colcon_ws/src/tts_asr_node'

        # ---- 临时文件目录（兼容旧接口的 fallback）----
        self.audio_dir = os.path.join(self._pkg_dir, 'temp_audio')
        os.makedirs(self.audio_dir, exist_ok=True)

        # ---- Vosk 模型加载 ----
        self.vosk_model = None
        self._init_vosk_model()

        # ---- VAD / 流式参数 ----
        self.declare_parameter('mode', 'auto')  # 'manual' | 'auto'
        self.declare_parameter('wake_word', '地瓜 地瓜')
        self.declare_parameter('wake_listen_time', 5)
        self.declare_parameter('chat_max_silence', 15)
        self.declare_parameter('vad_rms_threshold', 250)
        self.declare_parameter('vad_silence_timeout', 0.8)
        self.declare_parameter('max_record_sec', 10)

        # ---- 运行状态标志 ----
        self.listening = True
        self._tts_speaking = False
        self._tts_speaking_time = 0.0
        self._manual_trigger_event = threading.Event()  # 手动模式触发事件

        # ---- 模式判断 ----
        mode = self.get_parameter('mode').value

        # ---- 手动模式：订阅键盘节点发布的触发话题 ----
        if mode == 'manual':
            self.create_subscription(
                String, '/asr_manual_trigger',
                self._manual_trigger_callback, 10)
            self.get_logger().info(
                '[手动模式] 已订阅 /asr_manual_trigger，'
                '请另开终端运行 manual_trigger 节点')

        # ---- 启动唤醒词后台线程 ----
        self.wake_thread = threading.Thread(target=self._wake_word_loop, daemon=True)
        self.wake_thread.start()

        self.get_logger().info('=' * 50)
        self.get_logger().info('ASR 语音识别服务已启动 [Vosk 流式+PyAudio+VAD]')
        self.get_logger().info(f'  采样率: {self.sample_rate} Hz')
        self.get_logger().info(f'  VAD RMS阈值: {self.get_parameter("vad_rms_threshold").value}')
        self.get_logger().info(f'  VAD 静默超时: {self.get_parameter("vad_silence_timeout").value}s')
        self.get_logger().info(f'  最大录音时长: {self.get_parameter("max_record_sec").value}s')
        self.get_logger().info(f'  模型状态: {"已加载" if self.vosk_model else "未加载"}')

        if mode == 'manual':
            self.get_logger().info(f'  模式: 手动模式 (PTT对讲机，需另开键盘终端)')
            self.get_logger().info(f'  [操作] ros2 run tts_asr_node manual_trigger')
            self.get_logger().info(f'         按住 [空格] 录音，松开即识别 | [q] 退出')
        else:
            self.get_logger().info(f'  模式: 自动模式 (需要唤醒词唤醒)')
            self.get_logger().info(f'  唤醒词: "{self.get_parameter("wake_word").value}"')

        self.get_logger().info('=' * 50)

    # ==================== 模型初始化 ====================

    def _init_vosk_model(self):
        if Model is None:
            self.get_logger().error('请先安装 Vosk: pip install vosk')
            return

        try:
            model_path = os.path.join(self._pkg_dir, 'model', 'vosk-model-small-cn-0.22')
            if not os.path.exists(model_path):
                self.get_logger().error(f'模型目录不存在: {model_path}')
                return

            self.vosk_model = Model(model_path)
            self.get_logger().info(f'Vosk 模型已加载: {model_path}')
        except Exception as e:
            self.get_logger().error(f'模型加载失败: {e}')

    # ==================== /asr 服务（兼容旧接口，改用流式）====================

    def asr_callback(self, request, response):
        """/asr 服务回调：流式录音+VAD+Vosk 识别"""
        max_sec = request.listen_time
        if max_sec <= 0 or max_sec > 60:
            max_sec = 15

        sr = StreamRecognizer(self)
        text = sr.record_and_recognize(timeout_sec=max_sec)

        if text:
            msg = String()
            msg.data = text
            self.result_pub.publish(msg)
            self.get_logger().info(f'[服务] 识别结果: "{text}"')
            response.result = True
            response.text = text
        else:
            response.result = False
            response.text = ""
            self.get_logger().warn('[服务] 未识别到有效内容')

        return response

    # ==================== TTS 回声抑制 ====================

    def _tts_status_callback(self, msg: String):
        if msg.data == 'speaking':
            self._tts_speaking = True
            self._tts_speaking_time = time.time()
        elif msg.data == 'idle':
            self._tts_speaking = False

    # ==================== 手动模式：键盘节点话题回调 ====================

    def _manual_trigger_callback(self, msg: String):
        """/asr_manual_trigger 话题回调（由独立键盘节点 manual_trigger 发布）

        消息格式：
            'press'  — 按下空格键，开始录音
            'release' — 松开空格键，停止录音并识别
        """
        if msg.data == 'press':
            self._manual_trigger_event.set()
            self.get_logger().info('[手动触发] >>> 按住空格，开始录音...')
        elif msg.data == 'release':
            self._manual_trigger_event.clear()
            self.get_logger().info('[手动触发] <<< 松开空格，停止录音，开始识别')
        elif msg.data == 'quit':
            self.get_logger().info('[手动触发] 收到退出指令')
            self.listening = False

    # ==================== 唤醒词 & 对话主循环 ====================

    def _speak_wake_response(self):
        if not self.tts_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('TTS 未就绪，跳过唤醒播报')
            return
        req = Tts.Request()
        req.text = '我在哦，有什么事吗'
        future = self.tts_client.call_async(req)
        future.add_done_callback(self._tts_done)

    def _tts_done(self, future):
        try:
            result = future.result()
            if result.result:
                self.get_logger().info('唤醒提示语播报完成')
            else:
                self.get_logger().warn('唤醒提示语播报失败')
        except Exception as e:
            self.get_logger().error(f'TTS 调用异常: {e}')

    def _wake_word_loop(self):
        """唤醒词 + 对话主循环 / 手动模式主循环（全部使用流式录音+VAD）

        自动模式流程：
            ┌──────────────────────────────────────────┐
            │ 阶段1: 等待唤醒词                          │
            │   流式录音 → Vosk 识别 → 文本匹配唤醒词?   │
            │   匹配成功 → TTS播报 → 进阶段2             │
            ├──────────────────────────────────────────┤
            │ 阶段2: 持续对话                           │
            │   等 TTS 说完                             │
            │   流式录音 + VAD 自动截断 → Vosk 识别     │
            │   发布结果 → 等 LLM/TTS 回复              │
            │   循环直到连续静默超时                     │
            │   → 回到阶段1                              │
            └──────────────────────────────────────────┘

        手动模式流程（PTT 对讲机模式）：
            ┌──────────────────────────────────────────┐
            │ 等待按住空格键                             │
            │   按住后 → 录音（持续采集音频）            │
            │   松开后 → 停止录音 → Vosk 识别           │
            │   有结果? → 发布唤醒事件+TTS提示+结果      │
            │   无结果? → 静默忽略                      │
            └──────────────────────────────────────────┘
        """
        mode = self.get_parameter('mode').value
        wake_word = self.get_parameter('wake_word').value
        wake_listen_time = self.get_parameter('wake_listen_time').value
        chat_max_silence = self.get_parameter('chat_max_silence').value

        if mode == 'manual':
            # ==================== 手动模式（PTT 对讲机）====================
            self.get_logger().info(
                '[手动模式] 已启动 (PTT对讲机)，按住空格录音，松开识别...')
            while self.listening and rclpy.ok():
                try:
                    # 等待按下空格键
                    if not self._manual_trigger_event.wait(timeout=0.1):
                        continue

                    # 按下了！不 clear，保持 set 状态让录音线程知道还在按住
                    self.get_logger().info('[手动模式] >>> 按住空格，开始录音...')

                    # 每次触发新建实例，避免状态累积
                    sr = StreamRecognizer(self, manual_mode=True)
                    # 录音中：record_and_recognize 会检测松开(event clear)后停止
                    cmd_text = sr.record_and_recognize()

                    if cmd_text.strip():
                        # 有识别结果 → 发布唤醒事件 + TTS提示 + 识别结果
                        wake_msg = String()
                        wake_msg.data = '__manual__'
                        self.wake_pub.publish(wake_msg)
                        self._speak_wake_response()

                        msg = String()
                        msg.data = cmd_text.strip()
                        self.result_pub.publish(msg)
                        self.get_logger().info(f'[手动模式] 识别结果: "{msg.data}"')
                    else:
                        self.get_logger().warn('[手动模式] 未识别到有效内容，已静默忽略')

                    # 继续等待下一次触发
                    self.get_logger().info('[手动模式] 录音结束，等待下次触发...')

                except Exception as e:
                    self.get_logger().error(f'[手动模式] 出错: {e}')
                    time.sleep(0.5)

        else:
            # ==================== 自动模式（仅唤醒词触发）====================
            while self.listening and rclpy.ok():
                try:
                    # 每轮循环新建实例，避免状态累积
                    sr = StreamRecognizer(self)

                    # ========== 阶段1: 等待唤醒词 ==========
                    self.get_logger().info('--- 等待唤醒词 ---')

                    # 正常唤醒词监听流程
                    text = sr.record_and_recognize(timeout_sec=wake_listen_time)
                    self.get_logger().info(f'唤醒词检测结果: "{text}"')

                    if wake_word in text:
                        self.get_logger().info(f'>>> 唤醒词命中！进入对话模式')

                        wake_msg = String()
                        wake_msg.data = wake_word
                        self.wake_pub.publish(wake_msg)
                        self._speak_wake_response()

                        # ========== 阶段2: 持续对话（VAD自动截断）==========
                        silence_rounds = 0
                        while self.listening and rclpy.ok():
                            # --- 等待 TTS 说话完毕 ---
                            if self._tts_speaking:
                                time.sleep(0.3)
                                continue
                            if (self._tts_speaking_time > 0 and
                                    time.time() - self._tts_speaking_time < 0.5):
                                time.sleep(0.2)
                                continue

                            # --- 流式录音 + VAD 自动截断 + 识别 ---
                            cmd_text = sr.record_and_recognize()

                            if cmd_text.strip():
                                msg = String()
                                msg.data = cmd_text.strip()
                                self.result_pub.publish(msg)
                                self.get_logger().info(f'[对话] 识别并发布: "{msg.data}"')
                                silence_rounds = 0
                            else:
                                silence_rounds += 1
                                self.get_logger().debug(
                                    f'[对话] 未识别到内容 (静默轮次={silence_rounds})')

                            # 连续多轮静默则退出对话回到唤醒词监听
                            if silence_rounds >= chat_max_silence:
                                self.get_logger().info(
                                    f'连续 {silence_rounds} 轮无语音，退出对话模式')
                                break

                            # 短暂等待 LLM/TTS 处理
                            time.sleep(0.5)

                    # 唤醒词未匹配，继续下一轮监听
                    time.sleep(0.2)

                except Exception as e:
                    self.get_logger().error(f'唤醒词循环出错: {e}')
                    time.sleep(0.5)

        # 自动模式循环结束


# ==================== 主入口 ====================

def main(args=None):
    rclpy.init(args=args)
    node = AsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listening = False
        if node.wake_thread.is_alive():
            node.wake_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
