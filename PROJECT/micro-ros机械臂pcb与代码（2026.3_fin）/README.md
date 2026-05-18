# micro-ROS 4DOF 机械臂

基于 ESP32-S3 的 4 自由度机械臂控制系统，支持 Web 页面控制与 micro-ROS 通信。

## 功能

- 4 路舵机独立 PWM 控制（底座、肩部、肘部、夹爪）
- 手机浏览器 Web 页面控制（4 个滑块实时调角度）
- micro-ROS 通信，可接入 ROS 2 系统
- AHT30 温湿度检测，Web 与 ROS 话题实时发布
- 预设姿态保存/加载
- 上电自动回到中立位置 (90°)

## 技术栈

- **主控**: ESP32-S3 (Xtensa LX7)
- **开发环境**: PlatformIO (Arduino 框架)
- **控制方式**: LEDC 硬件 PWM (50Hz, 14bit)
- **通信协议**: WiFi (STA 模式) + HTTP (WebServer) + micro-ROS
- **传感器**: AHT30 (I2C)

## 目录结构

```
ESP32端/my_robot/
├── src/main.cpp          — 主程序
├── platformio.ini        — PlatformIO 配置
├── include/              — 头文件
├── lib/                  — 第三方库
├── test/                 — 测试代码
└── .vscode/              — VS Code 配置
```

## 硬件清单

- ESP32-S3 开发板
- SG90/MG90S 舵机 × 4
- AHT30 温湿度传感器
- 5V 舵机独立供电电源
- PCB 文件: `Gerber_PCB3_2026-01-12.zip`

## 使用

1. 修改 `src/main.cpp` 中的 WiFi SSID 和密码
2. 用 PlatformIO 编译下载
3. 串口查看分配的 IP 地址，浏览器打开即可控制
