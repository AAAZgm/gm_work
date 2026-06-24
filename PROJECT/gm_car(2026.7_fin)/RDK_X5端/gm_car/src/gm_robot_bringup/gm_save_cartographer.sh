#!/bin/bash

FIXED_MAP_DIR="/home/sunrise/gm_car/colcon_ws/src/gm_navigation/maps"
MAP_NAME="my_cartographer_map"

# 1. 结束轨迹
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 2. 保存内部状态到 .pbstream 文件
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${FIXED_MAP_DIR}/${MAP_NAME}.pbstream'}"

# 3. 将 .pbstream 转换为 .pgm + .yaml 地图文件
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    --map_filestem=${FIXED_MAP_DIR}/${MAP_NAME} \
    --pbstream_filename=${FIXED_MAP_DIR}/${MAP_NAME}.pbstream \
    --resolution=0.05
    
echo "地图已保存到: ${FIXED_MAP_DIR}/${MAP_NAME}.pgm 和 ${FIXED_MAP_DIR}/${MAP_NAME}.yaml"
