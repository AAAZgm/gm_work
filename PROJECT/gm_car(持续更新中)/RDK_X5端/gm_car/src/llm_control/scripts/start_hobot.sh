#!/bin/bash
# 绝对路径启动hobot_llamacpp（完全对齐你的命令）
cd /home/sunrise/gm_car
source /opt/ros/humble/setup.bash

# 拼接模型绝对路径
VIT_MODEL="${HOME}/gm_car/models/vit_model_int16_v2.bin"
LLM_MODEL="${HOME}/gm_car/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf"

# 启动hobot_llamacpp（你的原始命令+绝对路径）
ros2 run hobot_llamacpp hobot_llamacpp --ros-args     -p feed_type:=1     -p model_type:=1     -p image_type:=0     -p user_prompt:="图片里面有啥"     -p model_file_name:="${VIT_MODEL}"     -p llm_model_name:="${LLM_MODEL}"     -p ros_img_sub_topic_name:=/camera/image_for_vla

