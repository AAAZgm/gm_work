# GM Car — 智能小车

基于 RDK X5 的 ROS 2 智能小车，集成激光雷达 SLAM、YOLO 视觉、语音控制与本地 LLM 推理。

> ⚠ **持续更新中**

## 功能

- 激光雷达 SLAM 建图与导航（Cartographer / SLAM Toolbox）
- YOLO BPU 加速目标检测 (地平线 BPU)
- 4 自由度机械臂抓取
- 语音控制（SenseVoice + 语音合成）
- 本地 LLM 大模型控制（llama.cpp）
- micro-ROS 底盘通信
- 自动巡检与路径规划
- 人体跟随
- 摄像头实时画面

## 技术栈

- **主控**: RDK X5 (地平线旭日 5)
- **底盘**: STM32F407 (lhl_car_2diff, 两轮差速)
- **框架**: ROS 2 (Humble)
- **雷达**: 镭神 LSLiDAR X 系列
- **视觉**: YOLO + 相机 (BPU 加速)
- **语音**: SenseVoice + TTS
- **AI**: llama.cpp (本地大模型推理)
- **建模**: SolidWorks 3D 打印外壳

## 目录结构

```
RDK_X5端/gm_car/     — ROS 2 工作空间（功能包集合）
├── src/
│   ├── gm_navigation/           — 导航与路径规划
│   ├── gm_exploration/          — 自主探索
│   ├── gm_yolo_bpu/             — YOLO 目标检测
│   ├── gm_car_vision_glm_tts/   — 视觉 + 语音合成
│   ├── voice_control_car/       — 语音控制
│   ├── llm_car_control/         — LLM 决策控制
│   ├── follow_person/           — 人体跟随
│   ├── patrol_robot/            — 自动巡检
│   ├── serial2ros2/             — 串口转发
│   ├── imu_ros2_device/         — IMU 驱动
│   ├── usb_cam/                 — 摄像头驱动
│   ├── micro-ROS-Agent/         — micro-ROS 代理
│   └── LSLIDAR_X_ROS2-20240228/ — 雷达驱动

底盘模型与固件/      — STM32F407 底盘固件 (STM32CubeMX + Keil)
小车模型/            — SolidWorks 3D 模型文件 (.SLDPRT / .SLDASM)
电脑端/              — llama.cpp Windows 端推理环境
```

## 硬件清单

- RDK X5 开发板
- STM32F407 底盘驱动板
- 镭神 LSLiDAR X 激光雷达
- USB 摄像头
- 520 编码电机 × 2
- 65mm 轮胎
- IMU 传感器
- 4 自由度机械臂
- 3D 打印外壳
- 锂电池供电系统

## 使用方法

小车底盘固件: 用 Keil 打开 `底盘模型与固件/lhl_car_2diff/MDK-ARM` 编译下载。

ROS 2 工作空间:
```bash
cd RDK_X5端/gm_car
colcon build
source install/setup.bash
```
