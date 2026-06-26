# gm_web_dashboard

地瓜小车 Web 控制面板，基于浏览器实现对 ROS2 机器人的远程监控与控制。

## 架构

启动后运行三个服务：

| 服务 | 端口 | 说明 |
|------|------|------|
| HTTP 静态服务器 | 8000 | 托管 `index.html` 和 `roslib.min.js` |
| rosbridge_websocket | 9090 | ROS2 ↔ WebSocket 桥接，前端通过它订阅/发布话题 |
| web_video_server | 8888 | 将 ROS2 图像话题转为 MJPEG 流 |

```
浏览器
  │
  ├── HTTP :8000 ──→ 静态网页 (index.html)
  ├── WS   :9090 ──→ rosbridge ──→ ROS2 话题订阅/发布
  └── HTTP :8888 ──→ web_video_server ──→ /camera/image_raw (MJPEG)
```

## 功能模块

### 地图与位姿
- 订阅 `/map` (nav_msgs/OccupancyGrid) 实时绘制栅格地图
- 订阅 `/odom` (nav_msgs/Odometry) 在地图上标注机器人位姿（红色三角形）

### 摄像头
- 通过 web_video_server 获取 `/camera/image_raw` 的 MJPEG 流
- 点击画面可重新加载

### 传感器状态
- 订阅 `/robot_state` (gm_car_interfaces/RobotStatus)：电池电压、线速度/角速度
- 订阅 `/sensor_temp_humidity` (std_msgs/Float32MultiArray)：温度、湿度

### 速度控制
- 发布 `/cmd_vel` (geometry_msgs/Twist)：前进/后退/左转/右转/停止

### 语音/文字指令
- 发布 `/asr_result` (std_msgs/String)：发送自然语言指令
- 支持浏览器 Web Speech API 麦克风语音输入（需 Chrome）
- 内置快捷指令按钮

## 依赖话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/map` | nav_msgs/OccupancyGrid | 订阅 | SLAM 地图 |
| `/odom` | nav_msgs/Odometry | 订阅 | 里程计位姿 |
| `/robot_state` | gm_car_interfaces/RobotStatus | 订阅 | 电池/速度状态 |
| `/sensor_temp_humidity` | std_msgs/Float32MultiArray | 订阅 | 温湿度 |
| `/camera/image_raw` | sensor_msgs/Image | 订阅(MJPEG) | 摄像头图像 |
| `/cmd_vel` | geometry_msgs/Twist | 发布 | 速度控制 |
| `/asr_result` | std_msgs/String | 发布 | 语音指令 |

## 使用方法

### 构建
```bash
cd ~/gm_car/colcon_ws
colcon build --packages-select gm_web_dashboard
source install/setup.bash
```

### 启动
```bash
ros2 launch gm_web_dashboard web_dashboard.launch.py
```

### 访问
在浏览器中打开（使用小车实际 IP）：
```
http://<小车IP>:8000
```

> **注意**：请使用 IP 地址访问，不要用 `localhost`，否则摄像头流无法加载。

## 依赖

- rosbridge_server
- web_video_server
- gm_car_interfaces
