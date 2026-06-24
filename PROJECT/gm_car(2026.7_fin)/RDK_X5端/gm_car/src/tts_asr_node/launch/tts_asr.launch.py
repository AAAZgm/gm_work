"""TTS & ASR 节点启动文件

启动语音合成(TTS)和语音识别(ASR)节点（流式+VAD版）。

用法:
    ros2 launch tts_asr_node tts_asr.launch.py
    ros2 launch tts_asr_node tts_asr.launch.py wake_word:=你好 vad_rms_threshold:=400
    ros2 launch tts_asr_node tts_asr.launch.py mode:=manual  # 手动模式(自动启动键盘终端)
    ros2 launch tts_asr_node tts_asr.launch.py mode:=auto    # 自动模式(需要唤醒词，默认)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---- 模式参数 ----
        DeclareLaunchArgument(
            'mode', default_value='auto',
            description='ASR工作模式: manual(手动/键盘触发) | auto(自动/需要唤醒词)'),
        
        # ---- 唤醒词参数 ----
        DeclareLaunchArgument(
            'wake_word', default_value='地瓜 地瓜',
            description='唤醒关键词'),
        DeclareLaunchArgument(
            'wake_listen_time', default_value='10',
            description='等待唤醒词的最大时长(秒)'),

        # ---- VAD 语音活动检测参数 ----
        DeclareLaunchArgument(
            'vad_rms_threshold', default_value='350',
            description='VAD RMS能量阈值(低于此值=静默)，麦克风灵敏调大/小'),
        DeclareLaunchArgument(
            'vad_silence_timeout', default_value='0.6',
            description='VAD连续静默多少秒后判定说话结束(秒)'),
        DeclareLaunchArgument(
            'max_record_sec', default_value='15',
            description='单次流式录音最大时长(秒)'),
        DeclareLaunchArgument(
            'chat_max_silence', default_value='15',
            description='对话模式下连续N轮无语音后退出回到唤醒词监听'),

        # ASR 语音识别节点 (流式 + VAD)
        Node(
            package='tts_asr_node',
            executable='Asr_node',
            name='asr_node',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'wake_word': LaunchConfiguration('wake_word'),
                'wake_listen_time': LaunchConfiguration('wake_listen_time'),
                'vad_rms_threshold': LaunchConfiguration('vad_rms_threshold'),
                'vad_silence_timeout': LaunchConfiguration('vad_silence_timeout'),
                'max_record_sec': LaunchConfiguration('max_record_sec'),
                'chat_max_silence': LaunchConfiguration('chat_max_silence'),
            }],
        ),

        # TTS 语音合成节点
        Node(
            package='tts_asr_node',
            executable='Tts_node',
            name='tts',
            output='screen',
        ),

        # ===== 手动模式：自动弹出键盘控制终端 =====
        # 仅在 mode=manual 时启动，通过 xterm 弹出新终端窗口
        Node(
            package='tts_asr_node',
            executable='manual_trigger',
            name='asr_keyboard',
            output='screen',
            prefix='xterm -e ',
            condition=LaunchConfigurationEquals('mode', 'manual'),
        ),
    ])
