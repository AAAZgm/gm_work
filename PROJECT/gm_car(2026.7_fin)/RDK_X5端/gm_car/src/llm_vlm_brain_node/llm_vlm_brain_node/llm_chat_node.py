#!/usr/bin/env python3
"""
LLM 对话节点 (多模态语义控制版 - 重构版)

链路：
    ASR识别 -> /asr_result -> LLM理解语义 -> 执行命令(移动/VLM/传感器/导航/Launch/TTS) + TTS播报

新增功能：支持通过 <launch> 标签启动 ROS2 launch 文件
"""
import re
import math
import json
import os
import time
import threading
import subprocess
import signal
import requests
import rclpy
from std_msgs.msg import String, Float32MultiArray
from gm_car_interfaces.msg import RobotStatus
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from llm_vlm_brain_interfaces.srv import Chat, VisionDescribe
from tts_asr_interfaces.srv import Tts
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class LLMChatNode(BasicNavigator):
    """LLM 对话节点 - 继承 BasicNavigator 实现导航+对话功能"""

    # 支持的标签类型（正则匹配）
    _TAG_RE = re.compile(r'<(move|vlm|sensor|nav|weather|launch)>(.*?)</\1>', re.DOTALL)

    # Launch 任务映射表：关键词 -> (package, launch_file)
    LAUNCH_COMMANDS = {
        'follow_without_gesture': ('follow_person', 'follow_without_gesture_launch_min.py'),
        'follow_with_gesture': ('follow_person', 'follow_with_gesture_launch_min.py'),
        'exploration': ('gm_exploration', 'exploration_launch_min.py'),
    }

    def __init__(self):
        super().__init__('llm_chat_node')

        # ---- 参数 ----
        self.declare_parameter('llm_url', 'http://localhost:8080')
        self.declare_parameter('model_name', 'qwen3')
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_tokens', 512)
        self.llm_url = self.get_parameter('llm_url').value
        self.model_name = self.get_parameter('model_name').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.chat_api = f"{self.llm_url.rstrip('/')}/v1/chat/completions"

        # ---- 运动参数 ----
        self.MAX_LINEAR = 0.2
        self.MAX_ANGULAR = 3.0
        self.linear_speed = 0.2
        self.angular_speed = 2.0

        # ---- 定时器句柄 ----
        self._stop_timer = None

        # ---- ASR 防抖 ----
        self._last_asr_time = 0
        self._asr_debounce = 3.0

        # ---- TF ----
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        # ---- 传感器数据缓存 ----
        self.battery_voltage = 0.0
        self.temperature = 0.0
        self.humidity = 0.0
        self._latest_odom = None

        # ---- 天气查询参数 ----
        self.declare_parameter('weather_city', '芜湖')
        self.weather_city = self.get_parameter('weather_city').value

        # ---- 导航相关 ----
        self._named_locations = {}
        self._locations_file = os.path.expanduser(
            '/home/sunrise/gm_car/colcon_ws/src/llm_vlm_brain_node/location/nav_locations.json'
        )
        self._nav_busy = False
        self._nav_lock = threading.Lock()
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self._load_locations()

        # ---- Launch 进程管理 ----
        self._launch_processes = {}  # {name: subprocess.Popen}
        self._launch_lock = threading.Lock()

        # ---- 订阅话题 ----
        self.create_subscription(RobotStatus, '/robot_state', self.robot_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/sensor_temp_humidity',
                                 self.sensor_temp_humidity_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(String, '/asr_result', self.asr_result_callback, 10)

        # ---- System Prompt ----
        self.system_prompt = self._build_system_prompt()

        # ---- 服务客户端 ----
        self.tts_client = self.create_client(Tts, 'tts')
        self.vlm_client = self.create_client(VisionDescribe, 'vlm_describe')

        # ---- 服务/发布者 ----
        self.create_service(Chat, 'llm_chat', self.chat_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.llm_response_pub = self.create_publisher(String, '/llm_response', 10)

        # ---- 启动日志 ----
        self.get_logger().info(
            f'LLM对话节点已启动 | 线速≤{self.MAX_LINEAR}m/s | 角速≤{self.MAX_ANGULAR}rad/s'
        )
        self.get_logger().info(
            f'支持标签: <move>运动 | <vlm>视觉 | <sensor>传感器 | <nav>导航 | '
            f'<weather>天气 | <launch>启动任务'
        )
        self.get_logger().info(f'默认城市: {self.weather_city}')
        self.get_logger().info(f'已加载地点: {self._named_locations or "无"}')

        # ---- 连通性检查（延迟3秒） ----
        self._conn_timer = self.create_timer(3.0, self._check_connectivity)

    def _build_system_prompt(self):
        return """\
你是"地瓜"，智能小车的语音助手。说话简洁口语化，像朋友聊天。

规则：
1. 中文数字转阿拉伯数字（两米→2，九十度→90）
2. 有意图就在回复末尾加功能标签，纯聊天不加
3. 先说人话再放标签，标签放最后
4. 传感器数据不许编造，用<sensor>实时查
5. <move>stop</move>是停车，<launch>stop</launch>是停止跟随/探索，别搞混
6. 多任务按执行顺序排列标签

=== 功能标签 ===

【运动】<move>方向 [数值]</move>
  forward  米数  前进     例<move>forward 1</move>前进1米
  backward 米数  后退     例<move>backward 0.5</move>后退0.5米
  left     度数  左转     例<move>left 90</move>左转90度
  right    度数  右转     例<move>right 360</move>右转一圈
  stop           停车     例<move>stop</move>
不带数值时默认：前进/后退0.5米，左转/右转90度

【视觉】<vlm>问题</vlm>
  例<vlm>前面有红色物体吗</vlm>检测特定目标
  例<vlm>红色物体在哪里</vlm>定位目标位置

【传感器】<sensor>类型</sensor>
  voltage    电压   例<sensor>voltage</sensor>
  temperature 温度  例<sensor>temperature</sensor>
  humidity   湿度   例<sensor>humidity</sensor>
  all        全部   例<sensor>all</sensor>

【导航】<nav>命令</nav>
  goto 坐标  导航到坐标    例<nav>goto 3 4</nav>
  goto 地点  导航到地点    例<nav>goto 家</nav>
  init       初始化位姿    例<nav>init</nav>
  save 名称 x y  保存地点  例<nav>save 卧室 2 8</nav>
  waypoints 地点;地点  多点 例<nav>waypoints 厨房;卧室</nav>

【天气】<weather>城市名</weather>
  例<weather>北京</weather>查指定城市天气
不带地点时默认芜湖

【任务】<launch>名称</launch>
  follow_without_gesture 视觉跟随   例<launch>follow_without_gesture</launch>
  follow_with_gesture   手势跟随    例<launch>follow_with_gesture</launch>
  exploration           自主探索    例<launch>exploration</launch>
  stop                  停止高级任务 例<launch>stop</launch>

已知地点：
{{LOCATIONS}}

=== 示例 ===
往前走 → 好的，往前走。<move>forward 0.5</move>
前进一米 → 收到，前进1米。<move>forward 1</move>
后退两米 → 收到，后退2米。<move>backward 2</move>
左转 → 好的，左转。<move>left 90</move>
左转45度 → 好的，左转。<move>left 45</move>
右转一圈 → 好嘞，右转一圈。<move>right 360</move>
停下 → 好的，停车了。<move>stop</move>
你看到了什么 → 让我看看。<vlm>你看到了什么</vlm>
前面有红色的东西吗 → 我看看。<vlm>有红色物体吗</vlm>
电压多少 → 帮你查一下。<sensor>voltage</sensor>
现在几度 → 查一下温度。<sensor>temperature</sensor>
湿度多少 → 查一下湿度。<sensor>humidity</sensor>
查所有传感器 → 全面检测中。<sensor>all</sensor>
去3 4 → 好的，出发。<nav>goto 3 4</nav>
回家 → 走，回家。<nav>goto 家</nav>
初始化位置 → 好的。<nav>init</nav>
把卧室定为2 8 → 记好了。<nav>save 卧室 2 8</nav>
先去厨房再去卧室 → <nav>waypoints 厨房;卧室</nav>
今天天气怎么样 → 查一下。<weather>默认</weather>
北京天气 → <weather>北京</weather>
跟着我 → 好的，跟着你。<launch>follow_without_gesture</launch>
手势控制 → 进入手势模式。<launch>follow_with_gesture</launch>
自主探索 → 去探索一下。<launch>exploration</launch>
别跟了 → 好的，停了。<launch>stop</launch>
去客厅查温湿度 → <nav>goto 客厅</nav><sensor>all</sensor>
去厨房看看有什么 → <nav>goto 厨房</nav><vlm>你看到了什么</vlm>
查询当前天气并初始化位置，并将门口定为三一然后前往门口描述环境 → <weather>默认</weather><nav>init</nav><nav>save 门口 3 1</nav><nav>goto 门口</nav><vlm>你看到了什么</vlm>
你好 → 你好呀！
你叫什么名字 → 我叫地瓜，是你的小车助手！
"""

    # ======================== 连通性检查 ========================

    def _check_connectivity(self):
        """启动时验证与下位机的通信连接"""
        self._conn_timer.cancel()
        self.get_logger().info('====== 正在验证与下位机的通信连接 ======')

        def check_topic(topic_name, device_desc):
            try:
                publishers = self.get_publishers_info_by_topic(topic_name)
                if publishers:
                    self.get_logger().info(f'[OK] {topic_name} → {device_desc}在线')
                else:
                    self.get_logger().warning(f'[FAIL] {topic_name} → {device_desc}未连接')
            except Exception as e:
                self.get_logger().warning(f'[ERROR] 检查 {topic_name}: {e}')

        check_topic('/asr_result', '麦克风')
        check_topic('/sensor_temp_humidity', '机械臂')
        self.get_logger().info('====== 连通性验证完成 ======')

    # ======================== 中文数字转换 ========================

    @staticmethod
    def _normalize_chinese_numbers(text: str) -> str:
        """将 ASR 输出中的中文数字转为阿拉伯数字"""
        CN = {'零': 0, '一': 1, '二': 2, '两': 2, '三': 3, '四': 4, '五': 5,
              '六': 6, '七': 7, '八': 8, '九': 9}
        UNIT = {'十': 10, '百': 100, '千': 1000, '万': 10000}

        def parse_cn_number(s):
            if not s:
                return None
            if s[0] == '十':
                s = '一' + s
            result = 0
            current = 0
            for ch in s:
                if ch in CN:
                    current = CN[ch]
                elif ch in UNIT:
                    if current == 0:
                        current = 1
                    result += current * UNIT[ch]
                    current = 0
                else:
                    return None
            result += current
            return result if result > 0 else None

        all_chars = set(CN.keys()) | set(UNIT.keys())
        pattern = '[' + ''.join(re.escape(c) for c in all_chars) + ']+'

        def replace_match(m):
            original = m.group(0)
            num = parse_cn_number(original)
            if num is not None:
                return str(num)
            return original

        return re.sub(pattern, replace_match, text)

    # ======================== ASR 回调 ========================

    def asr_result_callback(self, msg: String):
        """ASR 话题回调：收到语音识别结果后自动处理"""
        now = time.time()
        if now - self._last_asr_time < self._asr_debounce:
            self.get_logger().info(f'ASR防抖: 距上次{now - self._last_asr_time:.1f}s < {self._asr_debounce}s，跳过')
            return
        self._last_asr_time = now

        text = msg.data
        if not text or not text.strip():
            return
        text = self._normalize_chinese_numbers(text)
        self.get_logger().info(f'ASR识别: "{text}"')

        # 异步处理 LLM 调用
        threading.Thread(target=self._handle_asr_llm, args=(text,), daemon=True).start()

    # ======================== 任务执行引擎 ========================

    def _execute_cmds(self, reply, task_list):
        """串行执行所有任务"""
        # 有任务时不播报回复文本（任务内部会自己TTS，避免重复）
        if task_list:
            self.get_logger().info(f'LLM回复(有任务，跳过TTS): "{reply[:50] if reply else ""}"')
        elif reply and reply != '（无内容）':
            self._call_tts_sync(reply)
            self.get_logger().info(f'LLM回复(无任务): "{reply}"')
            return
        else:
            return

        self.get_logger().info(f'共 {len(task_list)} 个任务，开始串行执行')

        for i, task in enumerate(task_list):
            self.get_logger().info(f'--- 执行任务 [{i + 1}/{len(task_list)}]: <{task["type"]}> ---')
            try:
                self._execute_single_task(task)
            except Exception as e:
                self.get_logger().error(f'任务[{i + 1}]<{task["type"]}> 执行异常: {e}')
                self._call_tts_sync(f'抱歉，第{i + 1}个任务出错了')
            if i < len(task_list) - 1:
                time.sleep(0.5)

        self.get_logger().info(f'全部 {len(task_list)} 个任务执行完毕')

    def _execute_single_task(self, task):
        """同步执行单个任务"""
        task_type = task['type']
        data = task['data']

        handlers = {
            'nav': lambda d: self._execute_nav_sync(d),
            'vlm': lambda d: self._call_vlm_sync(d),
            'sensor': lambda d: self._execute_sensor_query(d),
            'move': lambda d: self._execute_move(d),
            'weather': lambda d: self._execute_weather_sync(d),
            'launch': lambda d: self._execute_launch_sync(d),
        }

        handler = handlers.get(task_type)
        if handler:
            handler(data)
        else:
            self.get_logger().warning(f'未知任务类型: {task_type}')

    # ======================== Launch 启动管理 ========================

    def _execute_launch_sync(self, launch_type: str):
        """同步执行 Launch 命令（启动或停止 ROS2 launch 文件）

        Args:
            launch_type: 任务类型名称
                        - follow_without_gesture / follow_with_gesture / exploration → 启动任务
                        - stop → 停止所有正在运行的任务
        """
        # ===== 停止命令 =====
        if launch_type == 'stop':
            with self._launch_lock:
                if not self._launch_processes:
                    self.get_logger().info('没有正在运行的任务')
                    self._call_tts_sync('好的，当前没有正在运行的任务')
                    return

                running_tasks = list(self._launch_processes.keys())
                #friendly_names = [self._get_launch_friendly_name(t) for t in running_tasks]
                self.get_logger().info(f'正在停止任务: {running_tasks}')

            self.stop_all_launches()
            return

        # ===== 启动命令 =====
        if launch_type not in self.LAUNCH_COMMANDS:
            self.get_logger().warning(f'未知的 launch 类型: {launch_type}')
            self._call_tts_sync(f'抱歉，不知道如何启动 {launch_type}')
            return

        package, launch_file = self.LAUNCH_COMMANDS[launch_type]

        with self._launch_lock:
            # 如果有正在运行的进程，先停止它
            if self._launch_processes:
                old_name = list(self._launch_processes.keys())[0]
                self.get_logger().info(f'停止旧任务: {old_name}')
                self._stop_all_launches_internal()

        try:
            self._call_tts_sync(f'好的，正在启动{self._get_launch_friendly_name(launch_type)}')

            cmd = ['ros2', 'launch', package, launch_file]

            self.get_logger().info(f'启动 Launch: {" ".join(cmd)}')

            # 使用 subprocess.Popen 启动进程（非阻塞）
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )

            with self._launch_lock:
                self._launch_processes[launch_type] = process

            self.get_logger().info(f'Launch 进程已启动 [PID={process.pid}]')

            # 在后台线程监控输出
            monitor_thread = threading.Thread(
                target=self._monitor_launch_process,
                args=(launch_type, process,),
                daemon=True
            )
            monitor_thread.start()

        except Exception as e:
            self.get_logger().error(f'启动 Launch 失败: {e}')
            self._call_tts_sync(f'抱歉，启动失败：{e}')

    def _monitor_launch_process(self, name: str, process: subprocess.Popen):
        """后台监控 Launch 进程输出和状态"""

        def read_stream(stream, prefix):
            for line in iter(stream.readline, ''):
                if line:
                    self.get_logger().info(f'[Launch-{name}] {prefix} {line.strip()}')

        # 并行读取 stdout 和 stderr
        threads = []
        if process.stdout:
            t = threading.Thread(target=read_stream, args=(process.stdout, 'OUT'), daemon=True)
            t.start()
            threads.append(t)
        if process.stderr:
            t = threading.Thread(target=read_stream, args=(process.stderr, 'ERR'), daemon=True)
            t.start()
            threads.append(t)

        # 等待进程结束
        return_code = process.wait()

        with self._launch_lock:
            if name in self._launch_processes and self._launch_processes[name].pid == process.pid:
                del self._launch_processes[name]

        status = '正常退出' if return_code == 0 else f'异常退出(code={return_code})'
        self.get_logger().info(f'Launch [{name}] 已{status}')

    def _stop_all_launches_internal(self):
        """内部方法：停止所有 launch 进程（需要在 _lock 内调用）"""
        for name, proc in list(self._launch_processes.items()):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                self.get_logger().info(f'已发送停止信号给 [{name}] PID={proc.pid}')
            except Exception as e:
                self.get_logger().warning(f'停止 [{name}] 时出错: {e}')
        self._launch_processes.clear()

    def stop_all_launches(self):
        """公共接口：停止所有正在运行的 launch 任务"""
        with self._launch_lock:
            if self._launch_processes:
                self._stop_all_launches_internal()
                self._call_tts_sync('好的，已停止当前任务')

    @staticmethod
    def _get_launch_friendly_name(launch_type: str) -> str:
        """获取 launch 类型的中文友好名称"""
        names = {
            'follow_without_gesture': '跟随模式',
            'follow_with_gesture': '手势跟随模式',
            'exploration': '自主探索',
        }
        return names.get(launch_type, launch_type)

    # ======================== 导航执行 ========================

    def _execute_nav_sync(self, cmd):
        """同步执行导航命令"""
        sub = cmd.get('sub', '')
        try:
            nav_handlers = {
                'goto': lambda: self._nav_goto(cmd),
                'init': lambda: self._nav_init(),
                'save': lambda: self._nav_save_location(cmd),
                'waypoints': lambda: self._nav_waypoints(cmd),
            }
            handler = nav_handlers.get(sub)
            if handler:
                handler()
            else:
                self.get_logger().warning(f'未知导航子命令: {sub}')
                self._call_tts_sync('抱歉，未知的导航命令')
        except Exception as e:
            self.get_logger().error(f'导航执行异常: {e}')
            self._call_tts_sync(f'抱歉，导航出错：{e}')

    # ======================== VLM 调用 ========================

    def _call_vlm_sync(self, query):
        """同步调用 VLM 服务"""
        self.get_logger().info(f'正在调用 VLM: {query[:30]}...')

        if not self.vlm_client.service_is_ready():
            self.get_logger().warning('VLM 服务(/vlm_describe)未就绪')
            self._call_tts_sync('抱歉，视觉模块还没准备好')
            return

        req = VisionDescribe.Request()
        req.query = query

        future = self.vlm_client.call_async(req)
        deadline = time.time() + 30

        while not future.done():
            if time.time() > deadline:
                self.get_logger().error('VLM 调用超时(30s)')
                self._call_tts_sync('抱歉，视觉服务超时了')
                return
            time.sleep(0.2)

        try:
            result = future.result()
            desc = result.description
            if result.result and desc:
                self.get_logger().info(f'VLM返回: "{desc}"')
                self._call_tts_sync(desc)
            else:
                self.get_logger().warning('VLM 描述失败或返回空')
                self._call_tts_sync('抱歉，视觉分析没有成功')
        except Exception as e:
            self.get_logger().error(f'VLM 调用异常: {e}')
            self._call_tts_sync('抱歉，视觉服务出错了')

    # ======================== 天气查询 ========================

    def _execute_weather_sync(self, city):
        """同步查询天气"""
        try:
            url = f"https://wttr.in/{city}?format=j1&lang=zh"
            resp = requests.get(url, timeout=10)
            resp.raise_for_status()
            data = resp.json()

            current = data['current_condition'][0]
            temp = current['temp_C']
            feels_like = current['FeelsLikeC']
            humidity = current['humidity']

            if 'lang_zh' in current and current['lang_zh']:
                desc = current['lang_zh'][0]['value']
            elif 'weatherDesc' in current and current['weatherDesc']:
                desc = current['weatherDesc'][0]['value']
            else:
                desc = '未知'

            wind_speed = current['windspeedKmph']
            reply = (
                f"{city}现在{desc}，气温{temp}度，体感{feels_like}度，"
                f"湿度{humidity}%，风速{wind_speed}公里每小时"
            )

            self.get_logger().info(f'天气查询: {reply}')
            self._call_tts_sync(reply)

        except requests.exceptions.Timeout:
            self._call_tts_sync('抱歉，天气查询超时了')
        except requests.exceptions.ConnectionError:
            self._call_tts_sync('抱歉，无法连接天气服务')
        except Exception as e:
            self.get_logger().error(f'天气查询出错: {e}')
            self._call_tts_sync('抱歉，查天气出了点问题')

    # ======================== TTS ========================

    def _call_tts_sync(self, text):
        """同步调用 TTS 并等待播报完成"""
        if not text or not text.strip():
            return

        # 发布 LLM 回复文本，供网页等外部订阅
        self.llm_response_pub.publish(String(data=text))

        if not self.tts_client.service_is_ready():
            self.get_logger().warning('TTS 未就绪，跳过')
            return

        req = Tts.Request()
        req.text = text
        future = self.tts_client.call_async(req)

        deadline = time.time() + 60
        while not future.done():
            if time.time() > deadline:
                self.get_logger().warning('TTS 播报超时(60s)')
                return
            time.sleep(0.1)

        try:
            result = future.result()
            if result.result:
                self.get_logger().info(f'TTS 播报成功: "{text[:30]}..."')
            else:
                self.get_logger().warning(f'TTS 播报失败: "{text[:30]}"')
        except Exception as e:
            self.get_logger().error(f'TTS 异常: {e}')

    # ======================== LLM 调用 ========================

    def _handle_asr_llm(self, text: str):
        """ASR -> LLM -> 执行任务的完整流程"""
        reply, task_list = self._call_llm(text)
        self._execute_cmds(reply, task_list)

    def chat_callback(self, request, response):
        """/llm_chat 服务回调"""
        reply, task_list = self._call_llm(request.message)
        threading.Thread(target=self._execute_cmds, args=(reply, task_list), daemon=True).start()
        response.response = reply
        response.result = bool(reply and not reply.startswith('抱歉'))
        return response

    def _parse_tag(self, tag_type: str, tag_content: str):
        """解析单个标签内容"""
        if tag_type == 'move':
            parts = tag_content.split()
            action = parts[0].lower()
            param = float(parts[1]) if len(parts) > 1 else None
            valid_actions = {'forward', 'backward', 'left', 'right', 'stop'}
            if action in valid_actions:
                return {'action': action, 'param': param}
            return None

        elif tag_type == 'vlm':
            return tag_content if tag_content else None

        elif tag_type == 'sensor':
            sensor_type = tag_content.lower()
            valid_types = {'voltage', 'temperature', 'humidity', 'all'}
            if sensor_type in valid_types:
                return sensor_type
            return None

        elif tag_type == 'nav':
            return self._parse_nav_command(tag_content)

        elif tag_type == 'weather':
            city = tag_content.strip()
            if city.lower() in ('默认', 'default', ''):
                city = self.weather_city
            return city

        elif tag_type == 'launch':
            launch_type = tag_content.strip().lower()

            # 停止命令
            if launch_type in ('stop', '停止', 'cancel'):
                return 'stop'

            # 启动命令：精确匹配或模糊匹配
            if launch_type in self.LAUNCH_COMMANDS:
                return launch_type

            for key in self.LAUNCH_COMMANDS:
                if key in launch_type or launch_type in key:
                    return key

            return None

        return None

    def _call_llm(self, user_message):
        """
        调用 LLM API，从输出中解析所有功能标签。

        返回: (纯文本回复, 任务列表)
        """
        dynamic_prompt = self.system_prompt.replace('{{LOCATIONS}}', self._format_locations_for_prompt())

        messages = [
            {"role": "system", "content": dynamic_prompt},
            {"role": "user", "content": user_message},
        ]

        self.get_logger().info(f'正在调用 LLM: {user_message[:30]}...')

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
                timeout=(5, 30),
            )
            resp.raise_for_status()

            msg = resp.json()['choices'][0]['message']
            content = msg.get('content', '').strip()
            reasoning = msg.get('reasoning_content', '').strip()

            if not content and reasoning:
                content = reasoning
            if not content:
                return ('（无内容）', [])

            # 解析所有标签
            reply_text = content
            task_list = []

            for match in self._TAG_RE.finditer(content):
                tag_type = match.group(1)
                tag_content = match.group(2).strip()
                parsed = self._parse_tag(tag_type, tag_content)
                if parsed is not None:
                    task_list.append({'type': tag_type, 'data': parsed})
                    self.get_logger().info(f'解析到任务 [{tag_type}]: {parsed}')

            reply_text = self._TAG_RE.sub('', content).strip()

            return (reply_text or '', task_list)

        except requests.exceptions.Timeout:
            return ("抱歉，思考太久了，请再试一次。", [])
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'无法连接 LLM: {self.llm_url}')
            return ("抱歉，无法连接语言模型服务。", [])
        except Exception as e:
            self.get_logger().error(f'LLM 出错: {e}')
            return (f"抱歉，出错: {e}", [])

    # ======================== 运动执行 ========================

    def _execute_move(self, cmd):
        """解析并执行运动命令"""
        action = cmd['action']
        param = cmd['param']

        # 取消之前的定时器
        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None

        twist = Twist()
        duration = None

        move_actions = {
            'forward': lambda: (
                param if param else 0.5,
                min(self.linear_speed, self.MAX_LINEAR),
                True,
            ),
            'backward': lambda: (
                param if param else 0.5,
                -min(self.linear_speed, self.MAX_LINEAR),
                True,
            ),
            'left': lambda: (
                90 if not param else param,
                min(self.angular_speed, self.MAX_ANGULAR),
                False,
            ),
            'right': lambda: (
                90 if not param else param,
                -min(self.angular_speed, self.MAX_ANGULAR),
                False,
            ),
        }

        if action == 'stop':
            pass  # twist 保持全零
        elif action in move_actions:
            val, speed, is_linear = move_actions[action]()
            if is_linear:
                twist.linear.x = speed
                duration = val / abs(speed) if speed != 0 else None
            else:
                twist.angular.z = speed
                duration = (val / 360) * (2 * math.pi / abs(speed)) if speed != 0 else None
        else:
            self.get_logger().warning(f'未知动作: {action}')
            return

        self.cmd_vel_pub.publish(twist)

        info_parts = []
        if twist.linear.x != 0:
            info_parts.append(f'线速={twist.linear.x:.2f}m/s')
        if twist.angular.z != 0:
            info_parts.append(f'角速={twist.angular.z:.2f}rad/s')
        if duration:
            info_parts.append(f'{duration:.1f}秒后自停')

        self.get_logger().info(
            f'/cmd_vel -> {action}' + (f' param={param}' if param else '') +
            f' | {" ".join(info_parts)}'
        )

        if duration and duration > 0:
            time.sleep(duration)
            self.cmd_vel_pub.publish(Twist())
            self._stop_timer = self.create_timer(duration, self._auto_stop)

    def _auto_stop(self):
        """自动停止回调"""
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('/cmd_vel -> 自动停止')
        if self._stop_timer is not None:
            self._stop_timer.destroy()
            self._stop_timer = None

    # ======================== 传感器查询 ========================

    def _execute_sensor_query(self, sensor_type):
        """查询传感器数据并通过 TTS 播报"""
        parts = []

        if sensor_type in ('voltage', 'all'):
            parts.append(f"当前电压{self.battery_voltage:.1f}伏")
        if sensor_type in ('temperature', 'all'):
            parts.append(f"当前温度{self.temperature:.1f}度")
        if sensor_type in ('humidity', 'all'):
            parts.append(f"当前湿度{self.humidity:.1f}%")

        if parts:
            reply_text = "，".join(parts)
            self.get_logger().info(f'传感器查询({sensor_type}): {reply_text}')
            self._call_tts_sync(reply_text)
        else:
            self.get_logger().warning(f'未知传感器类型: {sensor_type}')
            self._call_tts_sync('抱歉，不知道要查什么传感器')

    def robot_state_callback(self, msg):
        """机器人状态回调"""
        try:
            self.battery_voltage = float(msg.battery_voltage)
        except (ValueError, AttributeError) as e:
            self.get_logger().warning(f'解析 robot_state 消息失败: {e}')

    def sensor_temp_humidity_callback(self, msg: Float32MultiArray):
        """温湿度传感器回调"""
        if len(msg.data) >= 3:
            self.temperature = float(msg.data[1])
            self.humidity = float(msg.data[2])
        else:
            self.get_logger().warning(f'sensor_temp_humidity 数据长度不足: {len(msg.data)}')

    def _odom_callback(self, msg: Odometry):
        """里程计回调"""
        self._latest_odom = msg

    # ======================== 导航控制 ========================

    def _parse_nav_command(self, raw: str):
        """解析 <nav> 标签内容"""
        parts = raw.strip().split()
        if not parts:
            return None

        sub_cmd = parts[0].lower()

        if sub_cmd == 'init':
            return {'sub': 'init'}

        elif sub_cmd == 'goto':
            if len(parts) < 2:
                return None
            try:
                x = float(parts[1])
                y = float(parts[2]) if len(parts) > 2 else 0.0
                return {'sub': 'goto', 'x': x, 'y': y}
            except ValueError:
                return {'sub': 'goto', 'name': parts[1]}

        elif sub_cmd == 'save':
            if len(parts) < 3:
                return None
            try:
                x = float(parts[-2])
                y = float(parts[-1])
                actual_name = ''.join(parts[1:-2])
                if not actual_name:
                    return None
                return {'sub': 'save', 'name': actual_name, 'x': x, 'y': y}
            except ValueError:
                return None

        elif sub_cmd == 'waypoints':
            waypoints_str = raw[len('waypoints'):].strip()
            if not waypoints_str:
                return None

            points = []
            for segment in waypoints_str.split(';'):
                segment = segment.strip()
                if not segment:
                    continue
                seg_parts = segment.split()
                if len(seg_parts) >= 2:
                    try:
                        px, py = float(seg_parts[0]), float(seg_parts[1])
                        points.append({'type': 'coord', 'x': px, 'y': py})
                        continue
                    except ValueError:
                        pass
                loc_name = ''.join(seg_parts)
                points.append({'type': 'location', 'name': loc_name})

            return {'sub': 'waypoints', 'points': points} if points else None

        return None

    def _nav_goto(self, cmd):
        """执行 goto 导航"""
        with self._nav_lock:
            if self._nav_busy:
                self._call_tts_sync('正在导航中，请先等一下')
                return
            self._nav_busy = True

        try:
            if 'name' in cmd:
                name = cmd['name']
                if name not in self._named_locations:
                    self._call_tts_sync(f'抱歉，没找到{name}的位置')
                    return
                x, y = self._named_locations[name]
                self.get_logger().info(f'地点 "{name}" -> ({x}, {y})')
            else:
                x, y = cmd['x'], cmd['y']

            self._call_tts_sync('好的，前往目标点')

            current = self.get_current_pose()
            if current is not None:
                rotation_euler = euler_from_quaternion([
                    current.rotation.x, current.rotation.y,
                    current.rotation.z, current.rotation.w])
                target_yaw = rotation_euler[2]
            else:
                target_yaw = 0.0

            target_pose = self.get_pose_by_xyyaw(x, y, target_yaw)
            self.nav_to_pose(target_pose)
        finally:
            with self._nav_lock:
                self._nav_busy = False

    def _nav_init(self):
        """执行初始化位姿"""
        if self._latest_odom is None:
            self._call_tts_sync('抱歉，尚未收到里程计数据，初始化失败')
            return

        odom = self._latest_odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        yaw = euler_from_quaternion([
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w
        ])[2]

        self._call_tts_sync(f'好的，已将位置初始化为({x:.1f}, {y:.1f})')

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose = self.get_pose_by_xyyaw(x, y, yaw).pose
        self._initialpose_pub.publish(pose_msg)
        self.get_logger().info(f'位姿已初始化为 ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

    def _nav_save_location(self, cmd):
        """保存命名地点"""
        name = cmd['name']
        x, y = cmd['x'], cmd['y']
        is_new = name not in self._named_locations
        self._named_locations[name] = (x, y)
        self._save_locations_to_file()
        action_str = '已记录' if is_new else '已更新'
        self._call_tts_sync(f'好的，{action_str}{name}位置为({int(x)}, {int(y)})')
        self.get_logger().info(f'地点{action_str}: {name} -> ({x}, {y})')

    def _nav_waypoints(self, cmd):
        """多点巡航"""
        with self._nav_lock:
            if self._nav_busy:
                self._call_tts_sync('正在导航中，请先等一下')
                return
            self._nav_busy = True

        try:
            targets = []
            for p in cmd['points']:
                if p['type'] == 'coord':
                    targets.append((p['x'], p['y']))
                elif p['type'] == 'location':
                    name = p['name']
                    if name in self._named_locations:
                        targets.append(self._named_locations[name])
                    else:
                        self._call_tts_sync(f'抱歉，没找到{name}的位置')
                        return

            if not targets:
                self._call_tts_sync('没有有效的目标点')
                return

            total = len(targets)
            self._call_tts_sync(f'好的，开始巡航，共{total}个目标点')

            for i, (x, y) in enumerate(targets, 1):
                self.get_logger().info(f'巡航 [{i}/{total}] -> ({x}, {y})')
                self._call_tts_sync(f'前往第{i}个点')
                target_pose = self.get_pose_by_xyyaw(x, y, 0.0)
                self.nav_to_pose(target_pose)

            self._call_tts_sync('巡航完成，全部到达')
        finally:
            with self._nav_lock:
                self._nav_busy = False

    # ======================== 地点管理 ========================

    def _format_locations_for_prompt(self):
        """格式化地点信息用于 prompt"""
        if not self._named_locations:
            return '（暂无保存的地点）'
        lines = [f'  {name}: ({x}, {y})' for name, (x, y) in self._named_locations.items()]
        return '\n'.join(lines)

    def _load_locations(self):
        """从 JSON 文件加载地点"""
        try:
            if os.path.exists(self._locations_file):
                with open(self._locations_file, 'r') as f:
                    data = json.load(f)
                    for k, v in data.items():
                        if isinstance(v, (list, tuple)):
                            self._named_locations[k] = (float(v[0]), float(v[1]))
                        elif isinstance(v, dict):
                            self._named_locations[k] = (float(v['x']), float(v['y']))
                self.get_logger().info(f'从文件加载了 {len(self._named_locations)} 个地点')
        except Exception as e:
            self.get_logger().warning(f'加载地点文件失败: {e}')
            self._named_locations = {}

    def _save_locations_to_file(self):
        """保存地点到 JSON 文件"""
        try:
            os.makedirs(os.path.dirname(self._locations_file), exist_ok=True)
            with open(self._locations_file, 'w') as f:
                json.dump(
                    {name: list(coords) for name, coords in self._named_locations.items()},
                    f,
                    ensure_ascii=False,
                    indent=2
                )
        except Exception as e:
            self.get_logger().error(f'保存地点文件失败: {e}')

    # ======================== Nav2 导航核心 ========================

    def nav_to_pose(self, target_pose):
        """导航到指定位姿（线程安全版）"""
        if not hasattr(self, '_nav_action_client') or self._nav_action_client is None:
            self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        if not self._nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 导航服务不可用')
            self._call_tts_sync('抱歉，导航服务未就绪')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)

        deadline = time.time() + 15
        while not send_goal_future.done():
            if time.time() > deadline:
                self.get_logger().error('导航目标接受超时')
                self._call_tts_sync('抱歉，导航响应超时')
                return False
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('导航目标被拒绝')
            self._call_tts_sync('抱歉，导航目标无法接受')
            return False

        self.get_logger().info('导航目标已接受，前往目标点...')

        result_future = goal_handle.get_result_async()
        nav_deadline = time.time() + 300

        while not result_future.done():
            if time.time() > nav_deadline:
                self.get_logger().error('导航执行超时')
                self._call_tts_sync('抱歉，导航执行太久了')
                return False
            time.sleep(0.5)

        result = result_future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('导航成功')
            return True
        elif status == 5:  # CANCELED
            self._call_tts_sync('导航已被取消')
            return False
        elif status == 6:  # ABORTED
            self._call_tts_sync('抱歉，导航失败')
            return False
        else:
            self._call_tts_sync(f'抱歉，导航异常状态{status}')
            return False

    def get_current_pose(self):
        """获取当前位姿"""
        for retry in range(3):
            if not rclpy.ok():
                return None
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint',
                    rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1)
                )
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移:{transform.translation}, 旋转欧拉角:{rotation_euler}'
                )
                return transform
            except Exception as e:
                self.get_logger().warning(f'获取坐标变换失败({retry + 1}/3): {e}')

        self.get_logger().error('TF 获取位姿失败：已重试3次')
        return None

    @staticmethod
    def get_pose_by_xyyaw(x, y, yaw):
        """通过 x, y, yaw 创建 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose


# ======================== 主函数入口 ========================

def main(args=None):
    """ROS2 节点标准入口"""
    rclpy.init(args=args)
    node = LLMChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
