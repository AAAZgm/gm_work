#!/usr/bin/env python3
"""远程启动节点 - 通过SSH控制下位机运行ROS2功能

支持功能:
    手势跟随 / 开启手势跟随 -> ros2 launch follow_person follow_with_gesture_launch.py
    人体跟随 / 开启人体跟随 -> ros2 launch follow_person follow_without_gesture_launch.py
    自主探索 / 开启自主探索 -> ros2 launch gm_exploration exploration_launch.py
    多点巡航 / 开启多点巡航 -> ros2 launch patrol_robot autopatrol.launch.py config:=...
    保存地图 / 开启建图     -> sh gm_save_slamtool.sh
    退出功能 / 停止         -> Ctrl+C 终止当前运行的功能

用法:
    ros2 run tts_asr_node remote_start --ros-args -p ssh_host:=192.168.x.x -p ssh_user:=sunrise -p ssh_password:=xxx
"""

import os
import signal
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RemoteStartNode(Node):
    """订阅语音识别结果, 通过SSH在下位机上启动对应功能"""

    # 功能名称 -> 远程命令映射
    COMMAND_MAP = {
        "手势跟随": (
            "cd ~/gm_car/colcon_ws && source install/setup.bash && "
            "ros2 launch follow_person follow_with_gesture_launch.py"
        ),
        "人体跟随": (
            "cd ~/gm_car/colcon_ws && source install/setup.bash && "
            "ros2 launch follow_person follow_without_gesture_launch.py"
        ),
        "自主探索": (
            "cd ~/gm_car/colcon_ws && source install/setup.bash && "
            "ros2 launch gm_exploration exploration_launch.py"
        ),
        "多点巡航": (
            "cd ~/gm_car/colcon_ws && source install/setup.bash && "
            "ros2 launch patrol_robot autopatrol.launch.py "
            "config:=/home/sunrise/gm_car/colcon_ws/src/patrol_robot/config/patrol_config.yaml"
        ),
        "保存地图": (
            "cd ~/gm_car/colcon_ws && bash gm_save_slamtool.sh"
        ),
    }

    # 关键词别名 (都映射到上面的标准名称)
    ALIASES = {
        "手势跟随": ["手势跟随", "开启手势跟随", "打开手势跟随", "启动手势跟随"],
        "人体跟随": ["人体跟随", "开启人体跟随", "打开人体跟随", "启动人体跟随", "人跟"],
        "自主探索": ["自主探索", "开启自主探索", "打开自主探索", "启动自主探索", "探索"],
        "多点巡航": ["多点巡航", "开启多点巡航", "打开多点巡航", "启动多点巡航", "巡航", "巡逻"],
        "保存地图": ["保存地图", "开启建图", "打开建图", "启动建图", "建图", "slam"],
    }
    STOP_KEYWORDS = {"退出功能", "停止", "关闭功能", "退出", "停止功能"}

    def __init__(self):
        super().__init__("remote_start_node")

        # ---- SSH 连接参数 ----
        self.declare_parameter("ssh_host", "192.168.1.100")
        self.declare_parameter("ssh_user", "sunrise")
        self.declare_parameter("ssh_password", "")
        self.ssh_host = self.get_parameter("ssh_host").value
        self.ssh_user = self.get_parameter("ssh_user").value
        self.ssh_password = self.get_parameter("ssh_password").value

        # ---- 状态管理 ----
        self._current_process: subprocess.Popen | None = None
        self._current_func_name: str = ""
        self._lock = threading.Lock()

        # ---- 订阅语音识别结果 ----
        self.create_subscription(String, "/asr_result", self._on_asr_result, 10)

        # 调用 TTS 服务播报
        self._tts_client = self.create_client("/tts", "tts_asr_interfaces/srv/Tts")

        self.get_logger().info(
            f"RemoteStartNode 已启动 | SSH: {self.ssh_user}@{self.ssh_host}"
        )
        self.get_logger().info(
            f"可用指令: {', '.join(self.COMMAND_MAP.keys())} / 退出功能"
        )

    def _speak(self, text: str):
        """异步调用TTS播报, 不阻塞主流程"""
        if not self._tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("TTS服务不可用, 跳过播报")
            return
        req = type(self._tts_client.srv_type).Request()
        req.text = text
        future = self._tts_client.call_async(req)

        def _done(fut):
            try:
                fut.result()
            except Exception:
                pass
        future.add_done_callback(_done)

    def _match_keyword(self, text: str) -> str | None:
        """
        将语音文本匹配为功能名或 'STOP'。
        返回 COMMAND_MAP 的 key, 或字符串 'STOP', 或 None(未命中)。
        """
        text = text.strip()

        # 先检查停止指令
        for kw in self.STOP_KEYWORDS:
            if kw in text:
                return "STOP"

        # 再检查功能指令 (最长匹配优先)
        best_match = None
        best_len = 0
        for func_name, aliases in self.ALIASES.items():
            for alias in aliases:
                if alias in text and len(alias) > best_len:
                    best_match = func_name
                    best_len = len(alias)
        return best_match

    def _stop_current(self):
        """停止当前正在运行的SSH进程"""
        with self._lock:
            if self._current_process is None:
                return False
            proc = self._current_process
            self._current_process = None
            old_name = self._current_func_name
            self._current_func_name = ""

        self.get_logger().info(f"正在停止 [{old_name}] ...")

        # 发送 Ctrl+C (SIGINT) 给远程进程组
        try:
            # 先尝试 SIGINT (Ctrl+C)
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # SIGINT 无效则 SIGTERM
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        except Exception as e:
            self.get_logger().warn(f"停止进程异常: {e}")

        self.get_logger().info(f"[{old_name}] 已停止")
        self._speak(f"已停止{old_name}")
        return True

    def _start_function(self, func_name: str):
        """通过SSH启动指定功能"""
        cmd = self.COMMAND_MAP[func_name]

        # 如果已有进程在跑,先停掉
        with self._lock:
            if self._current_process is not None:
                old_name = self._current_func_name
                self.get_logger().warn(
                    f"当前正在运行 [{old_name}], 请先说'退出功能'"
                )
                self._speak(f"当前正在运行{old_name}，请先说退出功能")
                return

        self.get_logger().info(f"正在启动 [{func_name}] ...")
        self._speak(f"好的, 正在启动{func_name}")

        # 构造 SSH 命令
        ssh_cmd = [
            "ssh",
            "-o", "StrictHostKeyChecking=no",
            "-o", "ServerAliveInterval=30",
            "-o", "ServerAliveCountMax=3",
            f"{self.ssh_user}@{self.ssh_host}",
            cmd,
        ]

        self.get_logger().info(f"SSH命令: {' '.join(ssh_cmd)}")

        try:
            proc = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                preexec_fn=os.setsid,          # 新建进程组, 方便统一杀掉子进程
                start_new_session=True,
            )

            with self._lock:
                self._current_process = proc
                self._current_func_name = func_name

            self.get_logger().info(f"[{func_name}] 启动成功 (PID={proc.pid})")

            # 异步读取输出日志 + 监控进程退出
            threading.Thread(
                target=self._monitor_process,
                args=(proc, func_name),
                daemon=True,
            ).start()

        except Exception as e:
            self.get_logger().error(f"启动失败: {e}")
            self._speak(f"启动失败, {str(e)}")

    def _monitor_process(self, proc: subprocess.Popen, func_name: str):
        """后台线程: 监控远程进程输出与退出状态"""
        import select

        # 持续读取 stdout/stderr 并打日志
        while True:
            if proc.poll() is not None:
                break
            readable, _, _ = select.select(
                [proc.stdout, proc.stderr], [], [], 1.0
            )
            for stream in readable:
                line = stream.readline()
                if line:
                    tag = "STDOUT" if stream is proc.stdout else "STDERR"
                    self.get_logger().debug(f"[{func_name}] {tag}: {line.strip()}")

        # 进程已退出
        ret_code = proc.returncode
        self.get_logger().info(
            f"[{func_name}] 进程已退出, 返回码={ret_code}"
        )
        self._speak(f"{func_name}已结束")

        with self._lock:
            if self._current_process is proc:
                self._current_process = None
                self._current_func_name = ""

    def _on_asr_result(self, msg: String):
        """收到语音识别结果回调"""
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"收到语音: '{text}'")

        match = self._match_keyword(text)
        if match == "STOP":
            self._stop_current()
        elif match and match in self.COMMAND_MAP:
            self._start_function(match)


def main():
    rclpy.init()
    node = RemoteStartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_current()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
