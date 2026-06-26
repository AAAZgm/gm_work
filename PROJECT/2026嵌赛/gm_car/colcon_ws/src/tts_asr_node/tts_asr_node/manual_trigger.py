#!/usr/bin/env python3
"""
ASR 手动模式 — 键盘控制节点 (PTT 对讲机风格)

类似 ROS teleop_twist_keyboard，在独立终端中运行，
按住空格键录音，松开后自动识别。

用法：
    ros2 run tts_asr_node manual_trigger

操作：
    按住 [空格]  —— 录音中...
    松开 [空格]  —— 停止录音，开始识别
    [q]          —— 退出

依赖：仅需 rclpy 和 std_msgs（无 xterm/gui 依赖）
"""

import os
import select
import sys
import termios
import time
import tty


def get_terminal_size():
    """获取终端尺寸"""
    try:
        size = os.get_terminal_size()
        return size.columns, size.lines
    except Exception:
        return 80, 24


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node('asr_keyboard')
    pub = node.create_publisher(String, '/asr_manual_trigger', 10)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # ---- 状态变量 ----
    is_pressing = False
    release_cooldown = 0.0  # 松开后的冷却时间戳

    def publish(msg_data: str):
        msg = String()
        msg.data = msg_data
        pub.publish(msg)

    def draw_ui(status_text: str = ''):
        """绘制界面（类似 teleop_twist_keyboard 的风格）"""
        cols, _ = get_terminal_size()
        line = '=' * min(cols, 50)
        # 用 \033[J 清除从光标到屏幕末尾，实现刷新效果
        output = f'''
\033[2J\033[H{line}
   ASR 手动模式 - 键盘控制节点 (PTT 对讲机)
{line}

   按住 [空格]  录音中...   松开即识别
   [q]         退出

   当前状态: {status_text}
{line}
   (此节点通过 /asr_manual_trigger 话题控制 asr_node)
{line}
'''
        sys.stdout.write(output)
        sys.stdout.flush()

    try:
        tty.setraw(fd)

        draw_ui('等待按键... (按住空格录音)')

        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                ch = sys.stdin.read(1)

                if ch == ' ' and not is_pressing:
                    # ---- 按下空格 ----
                    # 冷却保护：松开后 300ms 内忽略误触
                    if time.time() - release_cooldown < 0.3:
                        pass
                    else:
                        is_pressing = True
                        publish('press')
                        draw_ui('\033[1;32m>>> 录音中... (松开空格停止)\033[0m')

                elif ch == ' ' and is_pressing:
                    # 空格重复键（还在按住），忽略
                    pass

            else:
                # select 超时（无新输入）= 松开了
                if is_pressing:
                    is_pressing = False
                    release_cooldown = time.time()  # 记录松开时间
                    publish('release')
                    draw_ui('\033[1;33m<<< 识别中...\033[0m')

            # 让 ROS 回调有机会执行
            rclpy.spin_once(node, timeout_sec=0.001)

            # 检查 q 键（非阻塞方式已经在上面处理了）
            # 在 raw 模式下 q 会出现在 select 中

        # 如果循环退出是因为 rclpy.ok() 返回 False
        draw_ui('\033[1;31m已退出\033[0m')

    except KeyboardInterrupt:
        draw_ui('\033[1;31mCtrl+C 退出\033[0m')
        if is_pressing:
            publish('release')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
