#!/bin/bash

# ===================== 核心配置：修改为你想要的固定路径 =====================
FIXED_MAP_DIR="/home/sunrise/gm_car/colcon_ws/src/gm_navigation/maps"
MAP_NAME="my_slam_tool_map"
# ==========================================================================

# 创建目录（如果不存在）
# mkdir -p ${FIXED_MAP_DIR}

# 拼接完整的地图保存路径
FULL_MAP_PATH="${FIXED_MAP_DIR}/${MAP_NAME}"

# 使用 map_saver_cli 保存地图
ros2 run nav2_map_server map_saver_cli -f ${FULL_MAP_PATH}

# 提示保存完成
echo "地图已保存到：${FULL_MAP_PATH}.pgm 和 ${FULL_MAP_PATH}.yaml"
