/*
 * =================================================================
 *  项目名称: 6DOF 机械臂舵机控制器
 *  硬件平台: ESP32-S3 开发板
 *  功能说明:
 *    1. ESP32-S3 上电后自动连接手机 WiFi 热点 (STA 模式)
 *    2. 连接成功后, 手机浏览器打开串口打印的 IP 地址即可控制
 *    3. 网页上有 6 个滑块, 分别控制机械臂的 6 个关节舵机
 *    4. 上电时所有舵机自动回到 90° (机械臂竖直中立位置)
 *    5. 支持保存/加载预设姿态, 方便重复执行动作
 *  接线说明:
 *    - 6 个舵机的信号线分别接到 GPIO 6~11 (需根据实际修改)
 *    - 舵机供电建议使用独立 5V 电源, 不要从 ESP32 取电
 *    - 舵机地线 (GND) 需与 ESP32 GND 共地
 *  舵机驱动: 直接使用 ESP32-S3 的 LEDC 硬件 PWM
 *    - 每个 GPIO 独占一个 LEDC 通道, 互不干扰
 *    - 不依赖第三方舵机库, 更加稳定可靠
 * =================================================================
 */

// ---------- 引入所需的库 ----------

#include <WiFi.h>          // 引入 ESP32 的 WiFi 库, 提供 WiFi 连接功能
#include <WebServer.h>     // 引入 ESP32 的 Web 服务器库, 提供创建 HTTP 服务器的功能
#include <driver/ledc.h>   // 引入 ESP32 的 LEDC 驱动库, 提供硬件 PWM 信号输出功能
// ========== micro-ROS 头文件 ==========
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_srvs/srv/trigger.h>

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// --- ROS 2 话题 & 服务 全局变量 ---
rcl_publisher_t angle_publisher;                          // 角度话题发布者
std_msgs__msg__Float32MultiArray angle_pub_msg;           // 发布消息: 当前 6 个关节角度

rcl_subscription_t angle_subscriber;                      // 角度命令订阅者
std_msgs__msg__Float32MultiArray angle_sub_msg;           // 订阅消息缓冲

rcl_service_t home_service;                               // 复位服务
std_srvs__srv__Trigger_Request home_req;                  // 服务请求 (空)
std_srvs__srv__Trigger_Response home_res;                 // 服务响应

// =================================================================
//  一、WiFi 连接配置 (STA 模式)
//  ESP32-S3 上电后会连接手机的热点, 不再自己创建热点
//  【重要】请修改为你手机热点的名称和密码!
// =================================================================

const char *WIFI_SSID     = "gm_win11";   // WiFi 名称, 需改成你手机热点的名称
const char *WIFI_PASSWORD = "123456789";   // WiFi 密码, 需改成你手机热点的密码

// =================================================================
//  二、舵机硬件配置
//  6DOF = 6 个自由度, 即 6 个舵机分别控制机械臂的不同关节
//  使用 ESP32-S3 的 LEDC 硬件 PWM 直接驱动, 每路完全独立
// =================================================================

#define NUM_SERVOS 6  // 宏定义: 舵机总数量, 6 个自由度 = 6 个舵机

// 舵机信号线连接的 GPIO 引脚号
// 格式: {底座, 肩部, 肘部, 腕部俯仰, 腕部旋转, 夹爪}
// 注意: 不要用 GPIO 0~3, 这些是启动引脚/串口引脚
const int SERVO_PINS[NUM_SERVOS] = {6, 7, 8, 9, 10, 11};
// 第0个舵机(底座)接 GPIO6, 第1个(肩部)接 GPIO7, 依次类推

// 每个舵机对应一个独立的 LEDC 通道号 (0~5, 每个通道控制一个舵机)
const int SERVO_CHANNELS[NUM_SERVOS] = {0, 1, 2, 3, 4, 5};
// LEDC 通道 0~5, ESP32-S3 有 8 个低速通道可用

// 每个舵机的最小角度限制 (单位: 度), 防止舵机转到机械极限损坏
const int SERVO_MIN[NUM_SERVOS] = {0,   0,   0,   0,   0,   0};
// 6 个舵机的最小角度都是 0°

// 每个舵机的最大角度限制 (单位: 度)
const int SERVO_MAX[NUM_SERVOS] = {180, 180, 180, 180, 180, 180};
// 6 个舵机的最大角度都是 180°

// 每个舵机的名称, 用于网页显示和串口调试输出
const char *SERVO_NAMES[NUM_SERVOS] = {
  "Base",         // [0] 底座旋转舵机 (控制整个机械臂左右转动)
  "Shoulder",     // [1] 肩部舵机 (控制大臂上下摆动)
  "Elbow",        // [2] 肘部舵机 (控制小臂上下弯曲)
  "Wrist Pitch",  // [3] 腕部俯仰舵机 (控制手腕上下翘起)
  "Wrist Roll",   // [4] 腕部旋转舵机 (控制手腕左右翻转)
  "Gripper"       // [5] 夹爪舵机 (控制夹爪开合)
};

// 记录每个舵机当前的角度值, 初始值全部 90° (机械臂竖直中立位置)
int servoAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
// 这个数组会在网页控制时实时更新



// 创建 Web 服务器对象, 监听 TCP 80 端口 (HTTP 默认端口)
WebServer server(80);
// 浏览器访问 http://IP地址:80 即可打开控制网页

// =================================================================
//  三、LEDC PWM 参数
//  ESP32-S3 的 LEDC 可以生成精确的 PWM 信号来驱动舵机
//  - 频率 50Hz (标准舵机频率, 周期 20ms)
//  - 14 位分辨率 (16384 级精度)
//  - 脉宽范围 500~2500 微秒 (对应 0°~180°)
// =================================================================

#define LEDC_MODE       LEDC_LOW_SPEED_MODE  // 使用低速模式, 50Hz 不需要高速模式
#define LEDC_TIMER_NUM  LEDC_TIMER_0         // 使用 LEDC 定时器 0
#define LEDC_FREQ_HZ    50                   // PWM 频率 50Hz, 周期 20ms (标准舵机频率)
#define LEDC_DUTY_RES   LEDC_TIMER_14_BIT    // 14 位分辨率, 范围 0~16383 (ESP32-S3 最大 14 位)
#define SERVO_PULSE_MIN 500                  // 0° 对应的 PWM 脉宽 500 微秒 (0.5ms)
#define SERVO_PULSE_MAX 2500                 // 180° 对应的 PWM 脉宽 2500 微秒 (2.5ms)
#define LEDC_MAX_DUTY   16384                // 14 位最大值 = 2^14 = 16384

// 前向声明 (函数定义在后面, 但 micro_ros_setup 需要引用)
void setServoAngle(int channel, int angle);
void angle_cmd_callback(const void *msg_in);
void home_service_callback(const void *req_in, void *res_out);


bool micro_ros_setup() {
    // micro-ROS WiFi UDP 传输配置
    IPAddress agent_ip(192, 168, 137, 199);  // ★ 改成你上位机的 IP
    size_t agent_port = 8888;

    char ssid[] = "gm_win11";
    char psk[]  = "123456789";

    // set_microros_wifi_transports 会自动连接 WiFi 并设置 UDP 传输
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    // 等待网络稳定
    delay(2000);

    // 初始化 allocator内存分配器
    allocator = rcl_get_default_allocator();

    // 创建初始化support用来存储传输和内存分配器
    rclc_support_init(&support, 0, NULL, &allocator);

    // 创建节点
    rclc_node_init_default(&node, "esp32_servo_controller", "", &support);

    // --- 话题发布者: /arm_joint_angles (std_msgs/Float32MultiArray) ---
    // 上位机订阅此话题获取机械臂实时角度 (每 100ms 发布一次)
    memset(&angle_pub_msg, 0, sizeof(angle_pub_msg));
    angle_pub_msg.data.size = NUM_SERVOS;
    angle_pub_msg.data.capacity = NUM_SERVOS;
    angle_pub_msg.data.data = (float*)allocator.allocate(NUM_SERVOS * sizeof(float), allocator.state);
    rclc_publisher_init_default(&angle_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/arm_joint_angles");

    // --- 话题订阅者: /arm_joint_commands (std_msgs/Float32MultiArray) ---
    // 上位机发布 6 个 float 到此话题即可控制机械臂角度
    memset(&angle_sub_msg, 0, sizeof(angle_sub_msg));
    angle_sub_msg.data.size = NUM_SERVOS;
    angle_sub_msg.data.capacity = NUM_SERVOS;
    angle_sub_msg.data.data = (float*)allocator.allocate(NUM_SERVOS * sizeof(float), allocator.state);
    rclc_subscription_init_default(&angle_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/arm_joint_commands");

    // --- 服务: /arm_home (std_srvs/Trigger) ---
    // 上位机调用此服务将所有关节复位到 90°
    memset(&home_req, 0, sizeof(home_req));
    memset(&home_res, 0, sizeof(home_res));
    rclc_service_init_default(&home_service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/arm_home");

    // 初始化执行器 (2 个句柄: 1 订阅 + 1 服务)
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &angle_subscriber, &angle_sub_msg,
        &angle_cmd_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &home_service, &home_req, &home_res,
        &home_service_callback);

    return true;
}


// ---------- ROS 2 订阅回调: 上位机通过 /arm_joint_commands 发送目标角度 ----------
// 消息类型: std_msgs/Float32MultiArray, 6 个 float 分别对应 6 个关节
void angle_cmd_callback(const void *msg_in) {
  const std_msgs__msg__Float32MultiArray *cmd = (const std_msgs__msg__Float32MultiArray *)msg_in;
  int target[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    target[i] = (i < (int)cmd->data.size) ? constrain((int)cmd->data.data[i], SERVO_MIN[i], SERVO_MAX[i]) : servoAngles[i];
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoAngles[i] = target[i];
    setServoAngle(SERVO_CHANNELS[i], target[i]);
  }
}

// ---------- ROS 2 服务回调: 上位机调用 /arm_home 复位所有关节到 90° ----------
void home_service_callback(const void *req_in, void *res_out) {
  int home[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoAngles[i] = home[i];
    setServoAngle(SERVO_CHANNELS[i], home[i]);
  }
  std_srvs__srv__Trigger_Response *res = (std_srvs__srv__Trigger_Response *)res_out;
  res->success = true;
  static char ok_msg[] = "All joints reset to 90 degrees";
  res->message.data = ok_msg;
  res->message.size = strlen(ok_msg);
  res->message.capacity = sizeof(ok_msg);
}

// ---------- 角度转占空比函数 ----------
// 将角度 (0~180) 转换为 LEDC 占空比值 (0~16383)
// 原理: 占空比 = (脉宽 / 周期) * 最大值
//      例: 角度 90° → 脉宽 1500us → 占空比 = (1500/20000) * 16384 = 1228
int angleToDuty(int angle) {
  // 先用 Arduino 的 map() 函数把角度映射到脉宽 (500~2500 微秒)
  // map(value, fromLow, fromHigh, toLow, toHigh) 是线性映射
  long pulseUs = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  // 再把脉宽转换为 14 位占空比值
  // PWM 周期 = 1000000us / 50Hz = 20000us
  // 占空比 = (脉宽 / 20000) * 16384
  long duty = (pulseUs * LEDC_MAX_DUTY) / 20000L;  // L 后缀表示 long 类型, 防止溢出
  return (int)duty;  // 将 long 转为 int 返回
}

// ---------- 设置舵机角度函数 ----------
// 参数 channel: LEDC 通道号 (0~5)
// 参数 angle: 目标角度 (0~180)
void setServoAngle(int channel, int angle) {
  int duty=0;
  if(channel==3||channel==4)  {duty = angleToDuty(180-angle);}
  else {duty = angleToDuty(angle);  }                             // 先把角度转为 LEDC 占空比值
  ledc_set_duty(LEDC_MODE, (ledc_channel_t)channel, duty);    // 设置该通道的占空比 (还没生效)
  ledc_update_duty(LEDC_MODE, (ledc_channel_t)channel);       // 更新占空比, 使新值生效输出到舵机
}




// =================================================================
//  四、网页 HTML 代码 (存储在 Flash 中, 节省 RAM)
//  PROGMEM 关键字将字符串存放到 Flash 而非 RAM
//  R"rawliteral(...)rawliteral" 是 C++ 原始字符串, 避免转义字符问题
// =================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>6DOF 舵机控制</title>
<style>
  /* 全局样式: 去掉默认边距和内边距 */
  * { margin:0; padding:0; box-sizing:border-box; }
  /* 页面背景: 深蓝紫渐变, 字体浅灰色 */
  body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    background: linear-gradient(135deg, #0f0c29, #302b63, #24243e);
    color: #e0e0e0; min-height: 100vh; padding: 16px;
  }
  /* 标题样式: 居中, 天蓝色 */
  h1 { text-align:center; font-size:1.4em; margin-bottom:4px; color:#7ecfff; }
  /* 连接状态文字: 居中, 绿色 */
  .status { text-align:center; font-size:0.85em; color:#8a8; margin-bottom:16px; }
  /* 每个舵机的卡片样式: 半透明白色背景, 圆角 */
  .servo-card {
    background: rgba(255,255,255,0.07); border-radius:12px;
    padding:14px; margin-bottom:12px; backdrop-filter:blur(4px);
  }
  /* 卡片头部: 名称和角度值左右对齐 */
  .servo-header {
    display:flex; justify-content:space-between; align-items:center; margin-bottom:8px;
  }
  /* 舵机名称: 浅蓝色, 加粗 */
  .servo-name { font-size:1em; font-weight:600; color:#a0d4ff; }
  /* 角度数值: 黄色, 右对齐 */
  .servo-angle { font-size:1.1em; font-weight:700; color:#ffcc00; min-width:48px; text-align:right; }
  /* 滑块样式: 蓝色渐变轨道 */
  input[type=range] {
    -webkit-appearance:none; width:100%; height:8px; border-radius:4px;
    background: linear-gradient(to right, #4a90d9, #7ecfff); outline:none;
  }
  /* 滑块拖动圆点: 白色圆形, 带阴影 */
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance:none; width:28px; height:28px; border-radius:50%;
    background:#fff; cursor:pointer; box-shadow:0 2px 6px rgba(0,0,0,0.4);
  }
  /* 按钮行: 弹性布局, 居中 */
  .btn-row { display:flex; gap:8px; margin-top:12px; flex-wrap:wrap; justify-content:center; }
  /* 通用按钮样式: 白色文字, 无边框, 圆角 */
  .btn {
    flex:1; min-width:80px; padding:12px 8px; border:none; border-radius:10px;
    font-size:0.95em; font-weight:600; cursor:pointer; color:#fff; transition:all .15s;
  }
  /* 按钮按下时缩小效果 */
  .btn:active { transform:scale(0.95); }
  /* 复位按钮: 蓝色 */
  .btn-home  { background:#3498db; }
  /* 保存按钮: 绿色 */
  .btn-save  { background:#27ae60; }
  /* 预设姿态区域 */
  .presets { margin-top:16px; }
  .presets h3 { font-size:1em; color:#a0d4ff; margin-bottom:8px; }
  /* 预设按钮: 半透明边框, 小号字体 */
  .preset-btn {
    padding:8px 14px; margin:4px; border:1px solid rgba(126,207,255,0.4);
    border-radius:8px; background:rgba(255,255,255,0.08); color:#cde;
    cursor:pointer; font-size:0.85em; transition:all .15s;
  }
  .preset-btn:active { background:rgba(126,207,255,0.3); }
</style>
</head>
<body>
<h1>6DOF 机械臂控制</h1>
<div class="status" id="status">已连接</div>
<div id="sliders"></div>
<div class="btn-row">
  <button class="btn btn-home" onclick="goHome()">复位 (90°)</button>
  <button class="btn btn-save" onclick="savePose()">保存姿态</button>
</div>
<div class="presets">
  <h3>预设姿态</h3>
  <div id="presetList"></div>
</div>
<script>
// === JavaScript 部分 ===
// 6 个舵机的名称数组, 和 C++ 端一一对应
const NAMES = ["Base","Shoulder","Elbow","Wrist Pitch","Wrist Roll","Gripper"];
// 每个舵机的最小角度
const MIN = [0,0,0,0,0,0];
// 每个舵机的最大角度
const MAX = [180,180,180,180,180,180];
// 记录当前 6 个舵机的角度, 初始值 90°
let angles = [90,90,90,90,90,90];
// 记录当前正在拖动哪个滑块, -1 表示没有拖动
let dragging = -1;

// 获取页面上的滑块容器 div
const box = document.getElementById('sliders');
// 遍历 6 个舵机, 为每个舵机生成一个滑块卡片
NAMES.forEach((name, i) => {
  box.innerHTML += `
    <div class="servo-card">
      <div class="servo-header">
        <span class="servo-name">${name}</span>
        <span class="servo-angle" id="val${i}">${angles[i]}°</span>
      </div>
      <input type="range" id="s${i}" min="${MIN[i]}" max="${MAX[i]}" value="${angles[i]}"
        oninput="onSlider(${i},this.value)" onpointerdown="dragging=${i}" onpointerup="dragging=-1">
    </div>`;
});

// 滑块值改变时的回调函数
function onSlider(i, v) {
  angles[i] = parseInt(v);                     // 将新值转为整数存入角度数组
  document.getElementById('val'+i).textContent = v + '°';  // 更新页面上显示的角度数字
  sendAngles();                                 // 把新角度发送给 ESP32
}

// 上次发送时间戳, 用于节流 (防止发送太频繁)
let lastSend = 0;
// 发送当前角度到 ESP32
function sendAngles() {
  const now = Date.now();                       // 获取当前时间 (毫秒)
  if (now - lastSend < 50) return;              // 距离上次发送不到 50ms 则跳过 (节流)
  lastSend = now;                               // 更新上次发送时间
  const p = angles.join(',');                   // 把 6 个角度用逗号拼成字符串, 如 "90,45,120,60,30,0"
  fetch('/set?angles=' + p).catch(()=>{});      // 发送 HTTP GET 请求到 ESP32, catch 忽略错误
}

// 从 ESP32 获取当前舵机状态 (定时每 500ms 调用一次)
function fetchState() {
  fetch('/state')                               // 向 ESP32 发送 GET /state 请求
    .then(r=>r.json())                          // 把返回的 JSON 字符串解析为 JavaScript 对象
    .then(d=>{                                  // d = {"ip":"192.168.x.x","angles":[90,45,...]}
      if(d.angles) {                            // 如果返回数据中包含 angles 数组
        d.angles.forEach((a,i) => {             // 遍历 6 个角度值
          angles[i] = a;                        // 更新本地的角度数组
          const sl = document.getElementById('s'+i);   // 找到第 i 个滑块元素
          if(sl && dragging !== i) { sl.value = a; }  // 如果用户没在拖这个滑块, 更新滑块位置
          const vl = document.getElementById('val'+i); // 找到第 i 个角度显示元素
          if(vl) vl.textContent = a + '°';            // 更新角度文字
        });
      }
      // 更新页面顶部的连接状态文字, 显示 IP 地址
      document.getElementById('status').textContent = '已连接 - IP: ' + (d.ip||'');
    }).catch(()=>{                              // 如果请求失败 (网络断开等)
      document.getElementById('status').textContent = '正在连接...';  // 显示"正在连接"
    });
}

// 复位函数: 把所有舵机回到 90° 中立位置
function goHome() {
  const home = [90,90,90,90,90,90];             // 中立位置的角度值
  angles = [...home];                           // 更新本地角度数组 (展开运算符拷贝)
  home.forEach((v,i) => {                       // 遍历 6 个角度
    document.getElementById('s'+i).value = v;    // 重置滑块位置到 90°
    document.getElementById('val'+i).textContent = v + '°';  // 重置角度显示文字
  });
  sendAngles();                                 // 把复位角度发送给 ESP32
}

// 保存当前姿态到浏览器本地存储
function savePose() {
  const name = prompt('请输入姿态名称:');       // 弹出输入框让用户命名
  if(!name) return;                             // 用户点了取消则返回
  const poses = JSON.parse(localStorage.getItem('poses')||'{}');  // 读取已保存的所有姿态
  poses[name] = [...angles];                    // 把当前角度数组存入 (用展开运算符拷贝)
  localStorage.setItem('poses', JSON.stringify(poses));            // 写回 localStorage
  renderPresets();                              // 刷新页面上的预设按钮列表
}

// 加载一个已保存的姿态
function loadPose(name) {
  const poses = JSON.parse(localStorage.getItem('poses')||'{}');  // 读取所有已保存姿态
  if(poses[name]) {                             // 如果找到了对应名称的姿态
    angles = [...poses[name]];                  // 用保存的角度覆盖当前角度
    angles.forEach((v,i) => {                   // 遍历更新滑块和显示
      document.getElementById('s'+i).value = v;
      document.getElementById('val'+i).textContent = v + '°';
    });
    sendAngles();                               // 发送给 ESP32 执行
  }
}

// 删除一个已保存的姿态
function deletePose(name) {
  const poses = JSON.parse(localStorage.getItem('poses')||'{}');  // 读取所有姿态
  delete poses[name];                           // 从对象中删除这个姿态
  localStorage.setItem('poses', JSON.stringify(poses));            // 写回 localStorage
  renderPresets();                              // 刷新预设按钮列表
}

// 渲染预设姿态按钮列表
function renderPresets() {
  const poses = JSON.parse(localStorage.getItem('poses')||'{}');  // 读取所有已保存姿态
  const el = document.getElementById('presetList');                 // 获取容器 div
  el.innerHTML = '';                                               // 先清空
  Object.keys(poses).forEach(name => {                             // 遍历每个姿态名称
    // 为每个姿态生成两个按钮: 一个"加载", 一个"删除"(红色✕)
    el.innerHTML += `<button class="preset-btn" onclick="loadPose('${name}')">${name}</button>
      <button class="preset-btn" onclick="deletePose('${name}')" style="color:#e74c3c;">✕</button>`;
  });
}

// === 页面加载完成后执行 ===
renderPresets();                                // 渲染预设姿态按钮
setInterval(fetchState, 500);                   // 每 500ms 从 ESP32 获取一次状态
fetchState();                                   // 立即获取一次状态
</script>
</body>
</html>
)rawliteral";

// =================================================================
//  五、HTTP 请求处理函数
//  当浏览器访问不同路径时, 调用对应的处理函数
// =================================================================

// GET /state → 返回当前舵机状态和 ESP32 的 IP 地址
void handleState() {
  // 手动拼接 JSON 字符串 (不用 ArduinoJson 库, 省空间)
  // 结果格式: {"ip":"192.168.1.105","angles":[90,45,120,60,30,0]}
  String json = "{\"ip\":\"" + WiFi.localIP().toString() + "\",\"angles\":[";  // JSON 开头, 写入 IP
  for (int i = 0; i < NUM_SERVOS; i++) {       // 遍历 6 个舵机
    json += String(servoAngles[i]);             // 把角度值追加到 JSON 字符串
    if (i < NUM_SERVOS - 1) json += ",";        // 前 5 个后面加逗号, 最后一个不加
  }
  json += "]}";                                 // 关闭 JSON 数组和对象
  server.sendHeader("Access-Control-Allow-Origin", "*");  // 添加 CORS 头, 允许跨域访问
  server.send(200, "application/json", json);   // 返回 HTTP 200 成功, 内容类型 JSON
}

// GET /set?angles=90,45,120,60,30,0 → 接收角度并驱动舵机
void handleSetAngles() {
  if (!server.hasArg("angles")) {               // 检查请求中是否包含 "angles" 参数
    server.send(400, "text/plain", "Missing angles");  // 没有参数则返回 400 错误
    return;                                     // 直接返回, 不继续执行
  }
  String val = server.arg("angles");            // 获取参数值, 如 "90,45,120,60,30,0"
  int target[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    target[i] = servoAngles[i];  // 默认保持当前角度
  }
  int idx = 0;
  char buf[val.length() + 1];
  val.toCharArray(buf, sizeof(buf));
  char *tok = strtok(buf, ",");
  while (tok != NULL && idx < NUM_SERVOS) {
    target[idx] = constrain(atoi(tok), SERVO_MIN[idx], SERVO_MAX[idx]);
    idx++;
    tok = strtok(NULL, ",");
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoAngles[i] = target[i];
    setServoAngle(SERVO_CHANNELS[i], target[i]);
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");  // CORS 头
  server.send(200, "text/plain", "OK");         // 返回成功
}

// GET / → 返回控制网页 HTML
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);    // 返回 200, HTML 类型, 内容为 INDEX_HTML
}

// =================================================================
//  六、setup() - 初始化函数 (上电只执行一次)
//  Arduino 程序的入口点, 按顺序执行以下初始化步骤
// =================================================================
void setup() {
  Serial.begin(115200);                         // 初始化串口, 波特率 115200 (用于调试输出)
  delay(500);                                   // 等待 500ms, 让串口稳定
  Serial.println("\n=== 6DOF Servo Controller (LEDC PWM) ===");  // 打印启动信息

  // --- 第1步: 配置 LEDC 定时器 ---
  // 所有舵机共用一个定时器 (50Hz, 14位分辨率)
  // 定时器就像一个节拍器, 决定 PWM 信号的频率和精度
  ledc_timer_config_t timer_conf = {            // 创建定时器配置结构体
    .speed_mode = LEDC_MODE,                    // 低速模式 (低速模式频率范围 0.005Hz~40MHz)
    .duty_resolution = LEDC_DUTY_RES,           // 14 位分辨率 (0~16383, 精度足够)
    .timer_num = LEDC_TIMER_NUM,                // 使用定时器 0 (ESP32-S3 有 4 个定时器)
    .freq_hz = LEDC_FREQ_HZ,                    // PWM 频率 50Hz (舵机标准频率)
    .clk_cfg = LEDC_AUTO_CLK                    // 自动选择时钟源 (APB 或 APLL)
  };
  ledc_timer_config(&timer_conf);               // 应用定时器配置, 定时器开始工作
  Serial.println("LEDC timer configured: 50Hz, 14-bit");  // 打印确认信息

  // --- 第2步: 为每个舵机配置独立的 LEDC 通道 ---
  // 每个 GPIO 绑定一个独立的通道, 6 个舵机互不干扰
  for (int i = 0; i < NUM_SERVOS; i++) {        // 循环 6 次, 为每个舵机配置通道
    ledc_channel_config_t channel_conf = {       // 创建通道配置结构体
      .gpio_num = SERVO_PINS[i],                // 舵机信号输出引脚 (GPIO 6~11)
      .speed_mode = LEDC_MODE,                  // 低速模式 (与定时器匹配)
      .channel = (ledc_channel_t)SERVO_CHANNELS[i],  // 通道号 0~5 (强制类型转换)
      .intr_type = LEDC_INTR_DISABLE,           // 禁用中断 (不需要中断处理)
      .timer_sel = LEDC_TIMER_NUM,              // 绑定到定时器 0
      .duty = 0,                                // 初始占空比为 0 (舵机不动作)
      .hpoint = 0                               // 高电平起始点, 0 表示从周期起始开始
    };
    ledc_channel_config(&channel_conf);         // 应用通道配置, 该引脚开始输出 PWM

    // 设置初始角度 90° (机械臂竖直中立)
    setServoAngle(SERVO_CHANNELS[i], servoAngles[i]);  // 让舵机转到 90°

    // 在串口打印每个舵机的配置信息, 方便调试
    Serial.printf("Servo %d (%s): GPIO=%d, Channel=%d, Angle=%d\n",
                  i, SERVO_NAMES[i], SERVO_PINS[i], SERVO_CHANNELS[i], servoAngles[i]);
  }

  // --- 第3步: 连接手机 WiFi 热点 ---
  WiFi.mode(WIFI_STA);                          // 设置为 STA 模式 (Station, 当客户端连接热点)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);         // 开始连接 WiFi (传入名称和密码)
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);  // 打印正在连接的热点名称

  int wifiTimeout = 60;                         // WiFi 连接超时计数器, 60 次 × 500ms = 30 秒
  while (WiFi.status() != WL_CONNECTED && wifiTimeout > 0) {  // 循环等待连接成功或超时
    delay(500);                                 // 每 500ms 检查一次
    Serial.print(".");                          // 打印一个点, 表示正在等待
    wifiTimeout--;                              // 超时计数器减 1
  }
  Serial.println();                             // 换行 (上面的点打印完了)

  if (WiFi.status() == WL_CONNECTED) {          // 检查是否连接成功
    Serial.println("WiFi connected!");          // 成功: 打印成功信息
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());  // 打印分配到的 IP 地址
  } else {                                      // 连接失败
    Serial.println("WiFi connection FAILED!");  // 打印失败信息
    Serial.println("Please check SSID and password.");  // 提示检查名称和密码
  }

  // --- 第4步: 启动 HTTP 服务器 ---
  server.on("/", handleRoot);                   // 注册路由: 访问根路径 "/" 返回网页
  server.on("/state", handleState);             // 注册路由: 访问 "/state" 返回舵机状态 JSON
  server.on("/set", handleSetAngles);           // 注册路由: 访问 "/set" 设置舵机角度
  server.begin();                               // 启动 Web 服务器, 开始监听 80 端口

  Serial.println("HTTP server started!");       // 打印服务器启动成功
  Serial.printf("Open http://%s on your phone\n", WiFi.localIP().toString().c_str());  // 提示访问地址

    // ★ 初始化 micro-ROS（在 WiFi 连接成功后）
    if (WiFi.status() == WL_CONNECTED) {
        micro_ros_setup();
        Serial.println("micro-ROS initialized over WiFi UDP");
    }
}

// =================================================================
//  七、loop() - 主循环 (上电后无限重复执行)
// =================================================================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  server.handleClient();                        // 处理来自浏览器的 HTTP 请求 (必须放在循环中)

  // 每 1000ms 发布一次当前关节角度到 /arm_joint_angles
  static unsigned long last_pub_ms = 0;
  if (millis() - last_pub_ms >= 1000) {
    last_pub_ms = millis();
    for (int i = 0; i < NUM_SERVOS; i++) {
      angle_pub_msg.data.data[i] = (float)servoAngles[i];
    }
    (void)rcl_publish(&angle_publisher, &angle_pub_msg, NULL);
  }

  delay(2);                                     // 短暂延时 2ms, 让系统有机会处理其他任务 (看门狗等)
}
