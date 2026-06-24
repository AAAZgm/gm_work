#!/bin/bash

# 定义根目录
ROOT_DIR="$HOME/gm_car/colcon_ws/src/llm_car_control"

# 1. 创建核心目录结构
mkdir -p "${ROOT_DIR}/config"
mkdir -p "${ROOT_DIR}/launch"
mkdir -p "${ROOT_DIR}/llm_car_control"

# 2. 创建空文件
touch "${ROOT_DIR}/config/car_config.yaml"
touch "${ROOT_DIR}/launch/car_core.launch.py"
touch "${ROOT_DIR}/launch/yolo8_launch.py"
touch "${ROOT_DIR}/llm_car_control/__init__.py"
touch "${ROOT_DIR}/llm_car_control/vision_parser.py"
touch "${ROOT_DIR}/llm_car_control/llm_node.py"
touch "${ROOT_DIR}/llm_car_control/control_node.py"
touch "${ROOT_DIR}/package.xml"
touch "${ROOT_DIR}/setup.py"

# 3. 赋予执行权限
chmod +x "${ROOT_DIR}/../create_dirs_new.sh"

# 4. 输出结果
echo "✅ 适配官方YOLOv8的目录结构创建完成！"
ls -R "${ROOT_DIR}"
