/**
 * ============================================================
 * ADS1299 EEG WiFi 版本 — 网页实时脑电图
 * ============================================================
 * 功能: ESP32-C3 建立 WiFi 热点，浏览器连接后查看 8 通道实时 EEG 波形
 * 用法:
 *   1. 烧录此固件到 ESP32-C3
 *   2. 手机/电脑连接 WiFi 热点 "BCI_EEG_Monitor" (密码: 12345678)
 *   3. 浏览器打开 http://192.168.4.1
 *
 * 注意: 此文件与 main.cpp (串口版) 互斥，不能同时编译。
 *       通过 platformio.ini 的 build_src_filter 切换:
 *         - airm2m_core_esp32c3       → 编译 main.cpp (串口版)
 *         - airm2m_core_esp32c3_wifi  → 编译 main_wifi.cpp (WiFi版)
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ================== 双串口调试宏（兼容两个 USB 口）==================
// ESP32-C3 板载两个 USB: 原生USB(GPIO18/19)=Serial, CH340(GPIO20/21)=Serial0
// 不知道用户插哪个 → 两个都发，保证至少一个能看到
#define D_PRINT(...)    do { Serial.print(__VA_ARGS__);    Serial0.print(__VA_ARGS__);    } while(0)
#define D_PRINTLN(...)  do { Serial.println(__VA_ARGS__);  Serial0.println(__VA_ARGS__);  } while(0)
#define D_PRINTF(...)   do { Serial.printf(__VA_ARGS__);   Serial0.printf(__VA_ARGS__);   } while(0)

// ================== LED 状态灯 ==================
#define LED_PIN 12  // Ai-M2M 板载 LED (GPIO12)
#define LED_BLINK_FAST  150   // 快速闪烁: 正在启动 / WiFi 初始化中
#define LED_BLINK_SLOW  800   // 慢速闪烁: WiFi 失败
#define LED_SOLID       0     // 常亮: 系统就绪
#define LED_OFF         9999  // 全灭（不推荐）

// ================== ADS1299 宏定义区（与 main.cpp 完全一致）==================
// ---------- 采样率配置 ----------
#define CONFIG1_REG  0x95
#define CONFIG2_REG  0xC0
#define CONFIG3_REG  0xEC

// ---------- 通道配置 ----------
#define CHnSET_REG   0x60
#define ENABLE_SRB1  0x20
#define BIAS_SENSP   0xFF
#define BIAS_SENSN   0xFF

// ---------- 阻抗测量配置 ----------
#define LOFF_CONFIG   0x06
#define LOFF_SENSP    0xFF
#define LOFF_SENSN    0xFF

// ---------- 模式 ----------
#define MODE_CONTINUOUS_READ   1
#define MODE_IMPEDANCE_MEASURE 2
#define MODE_SELF_TEST         3

// ================== 引脚定义==================
#define CS_PIN    7
#define SCLK_PIN  4
#define MOSI_PIN  6
#define MISO_PIN  5
#define DRDY_PIN  2
#define START_PIN 10
#define RESET_PIN 3

// ================== ADS1299 SPI 命令 ==================
#define WAKEUP  0x02
#define STANDBY 0x04
#define RESET   0x06
#define START   0x08
#define STOP    0x0A
#define RDATAC  0x10
#define SDATAC  0x11
#define RDATA   0x12
#define RREG    0x20
#define WREG    0x40

// ================== WiFi 配置 ==================
#define WIFI_SSID     "BCI_EEG_Monitor"
#define WIFI_PASSWORD "12345678"

// ================== 数据批处理配置 ==================
#define BATCH_SIZE    25    // 每批采样点数 (500SPS × 50ms = 25)
#define MAX_POINTS    500   // 网页端显示点数（与 plotter.py 一致）
uint8_t id;
// ================== 全局变量 ==================
volatile bool dataReady = false;
int currentMode = MODE_CONTINUOUS_READ;
double channelDataBuffer[9];       // [0]=STATUS, [1]~[8]=8通道电压(V)

// 批量缓冲: [8通道][25采样点]
double batchBuffer[8][BATCH_SIZE];
int batchCount = 0;
int wifiRetryCount = 0;             // WiFi AP 重试计数

// 网络服务
WebServer server(80);              // HTTP 网页服务器
WebSocketsServer webSocket(81);    // WebSocket 数据推送

wifi_power_t wifiPowerLevel = WIFI_POWER_8_5dBm;

// ================== 函数声明 ==================
void IRAM_ATTR onDRDYInterrupt();
void initADS1299();
void startContinuousReadMode();
void startImpedanceMeasurementMode();
void startSelfTestMode();
void readData();
void convertData(uint8_t *data, double *channelData);
void sendCommand(uint8_t cmd);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void getDeviceID();
String buildBatchJson();
void handleRoot();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// ================== 内嵌网页 (HTML + CSS + JS) ==================
// 使用 C++11 raw string literal，内容完全原样，包含双引号也不需转义
// 注意: ESP32 上不能用 PROGMEM，否则 server.send() 读取失败
const char INDEX_HTML[] = R"EOF(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>ADS1299 EEG Monitor</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0d0d0d;
    color: #ccc;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    display: flex;
    flex-direction: column;
    align-items: center;
    min-height: 100vh;
    padding: 8px;
  }
  .header {
    text-align: center;
    margin-bottom: 6px;
  }
  .header h1 {
    font-size: clamp(16px, 4vw, 22px);
    color: #fff;
    letter-spacing: 1px;
  }
  .status-bar {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 13px;
    margin-bottom: 4px;
  }
  .status-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: #ff4444;
    transition: background 0.3s;
  }
  .status-dot.connected { background: #44ff44; box-shadow: 0 0 6px #44ff44; }
  .status-dot.disconnected { background: #ff4444; }
  .info-row {
    display: flex;
    gap: 16px;
    font-size: 11px;
    color: #888;
    margin-bottom: 4px;
  }
  #eegCanvas {
    width: 100%;
    max-width: 960px;
    height: auto;
    border: 1px solid #222;
    border-radius: 4px;
    background: #111;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 6px 14px;
    justify-content: center;
    margin-top: 6px;
    font-size: 11px;
  }
  .legend span {
    display: flex;
    align-items: center;
    gap: 4px;
  }
  .legend .dot {
    display: inline-block;
    width: 10px; height: 3px;
    border-radius: 2px;
  }
  .footer {
    margin-top: 8px;
    font-size: 10px;
    color: #555;
  }
  @media (max-width: 480px) {
    body { padding: 4px; }
    .header h1 { font-size: 15px; }
  }
</style>
</head>
<body>
<div class="header">
  <h1>ADS1299 EEG 实时数据</h1>
</div>
<div class="status-bar">
  <span class="status-dot" id="statusDot"></span>
  <span id="statusText">连接中...</span>
</div>
<div class="info-row">
  <span id="sampleRate">采样率: --</span>
  <span id="sampleCount">点数: 0</span>
  <span id="yRange">范围: --</span>
</div>
<canvas id="eegCanvas" width="960" height="640"></canvas>
<div class="legend" id="legend"></div>
<div class="footer">ESP32-C3 WiFi · IP: 192.168.4.1 · WebSocket Port: 81</div>

<script>
(function() {
  'use strict';

  // ================== 配置 ==================
  const CHANNELS = 8;
  const MAX_POINTS = 500;
  const COLORS = [
    '#e41a1c', '#377eb8', '#4daf4a', '#984ea3',
    '#ff7f00', '#ffff33', '#a65628', '#f781bf'
  ];

  // ================== 初始化缓冲 ==================
  const buffers = [];
  for (let i = 0; i < CHANNELS; i++) {
    buffers.push([]);
    for (let j = 0; j < MAX_POINTS; j++) buffers[i].push(0);
  }
  let totalSamples = 0;

  // ================== Canvas 设置 ==================
  const canvas = document.getElementById('eegCanvas');
  const ctx = canvas.getContext('2d');

  function resizeCanvas() {
    const maxW = Math.min(window.innerWidth - 16, 960);
    const ratio = maxW / 960;
    canvas.style.width = maxW + 'px';
    canvas.style.height = Math.round(640 * ratio) + 'px';
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  // ================== 图例 ==================
  const legendEl = document.getElementById('legend');
  for (let i = 0; i < CHANNELS; i++) {
    const span = document.createElement('span');
    span.innerHTML = '<i class="dot" style="background:' + COLORS[i] + '"></i>CH' + (i+1);
    legendEl.appendChild(span);
  }

  // ================== DOM 元素 ==================
  const statusDot = document.getElementById('statusDot');
  const statusText = document.getElementById('statusText');
  const sampleRateEl = document.getElementById('sampleRate');
  const sampleCountEl = document.getElementById('sampleCount');
  const yRangeEl = document.getElementById('yRange');

  function setStatus(connected) {
    statusDot.className = 'status-dot ' + (connected ? 'connected' : 'disconnected');
    statusText.textContent = connected ? '● 已连接' : '● 断开 (重连中...)';
  }

  // ================== 渲染 ==================
  let lastRenderTime = 0;
  const RENDER_MS = 33; // ~30fps

  function render() {
    const W = canvas.width, H = canvas.height;
    const bandH = H / CHANNELS;

    // 清屏
    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, W, H);

    // 先计算所有通道的全局 Y 范围（统一缩放，与 plotter.py 行为一致）
    let globalMin = Infinity, globalMax = -Infinity;
    for (let ch = 0; ch < CHANNELS; ch++) {
      const data = buffers[ch];
      for (let i = 0; i < data.length; i++) {
        if (data[i] < globalMin) globalMin = data[i];
        if (data[i] > globalMax) globalMax = data[i];
      }
    }
    let margin = Math.max(Math.abs(globalMin), Math.abs(globalMax)) * 0.15 + 1e-12;
    if (margin < 1e-12) margin = 1e-6; // 没有数据时给个小范围
    const yMin = globalMin - margin;
    const yMax = globalMax + margin;
    const dy = yMax - yMin || 1e-12;

    // 绘制每个通道
    for (let ch = 0; ch < CHANNELS; ch++) {
      const y0 = ch * bandH;
      const data = buffers[ch];
      const len = data.length;
      if (len < 2) continue;

      // 网格线 (5条水平线)
      ctx.strokeStyle = 'rgba(255,255,255,0.08)';
      ctx.lineWidth = 0.5;
      for (let g = 0; g <= 4; g++) {
        const gy = y0 + (bandH * g / 4);
        ctx.beginPath();
        ctx.moveTo(0, gy);
        ctx.lineTo(W, gy);
        ctx.stroke();
      }

      // 通道分隔线
      if (ch > 0) {
        ctx.strokeStyle = 'rgba(255,255,255,0.15)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, y0);
        ctx.lineTo(W, y0);
        ctx.stroke();
      }

      // 零线（如果 yMin < 0 < yMax）
      if (yMin < 0 && yMax > 0) {
        const zeroY = y0 + bandH - ((0 - yMin) / dy) * bandH;
        ctx.strokeStyle = 'rgba(255,255,255,0.12)';
        ctx.lineWidth = 0.5;
        ctx.setLineDash([4, 8]);
        ctx.beginPath();
        ctx.moveTo(0, zeroY);
        ctx.lineTo(W, zeroY);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 波形曲线
      ctx.strokeStyle = COLORS[ch];
      ctx.lineWidth = 1.0;
      ctx.beginPath();
      for (let i = 0; i < len; i++) {
        const x = (i / (MAX_POINTS - 1)) * W;
        const y = y0 + bandH - ((data[i] - yMin) / dy) * bandH;
        // 裁剪到 band 范围内
        const clampedY = Math.max(y0 + 1, Math.min(y0 + bandH - 1, y));
        if (i === 0) ctx.moveTo(x, clampedY);
        else ctx.lineTo(x, clampedY);
      }
      ctx.stroke();

      // Y 范围标注（首尾通道标数值）
      if (ch === 0 || ch === CHANNELS - 1) {
        ctx.fillStyle = 'rgba(255,255,255,0.5)';
        ctx.font = '9px monospace';
        const topLabel = ch === 0 ? (yMax * 1e6).toFixed(2) + 'uV' : '';
        const botLabel = ch === CHANNELS - 1 ? (yMin * 1e6).toFixed(2) + 'uV' : '';
        if (topLabel) ctx.fillText(topLabel, 4, y0 + 11);
        if (botLabel) ctx.fillText(botLabel, 4, y0 + bandH - 3);
      }

      // 通道标签
      ctx.fillStyle = COLORS[ch];
      ctx.font = 'bold 10px monospace';
      const label = 'CH' + (ch + 1);
      ctx.fillText(label, W - 32, y0 + 14);
    }

    // 更新信息栏
    sampleCountEl.textContent = '点数: ' + totalSamples;
    yRangeEl.textContent = '范围: ' + (yMin * 1e6).toFixed(1) + ' ~ ' + (yMax * 1e6).toFixed(1) + ' uV';
  }

  function requestRender(timestamp) {
    if (timestamp - lastRenderTime >= RENDER_MS) {
      lastRenderTime = timestamp;
      render();
    }
    requestAnimationFrame(requestRender);
  }
  requestAnimationFrame(requestRender);

  // ================== WebSocket ==================
  let sampleCount = 0;
  let lastCountTime = Date.now();

  function connect() {
    const wsUrl = 'ws://' + window.location.hostname + ':81';
    const ws = new WebSocket(wsUrl);

    ws.onopen = function() {
      setStatus(true);
      sampleCount = 0;
      lastCountTime = Date.now();
    };

    ws.onmessage = function(e) {
      try {
        const batch = JSON.parse(e.data);
        // batch = [[ch1_v1,...], [ch2_v1,...], ..., [ch8_v1,...]]
        if (!Array.isArray(batch) || batch.length < CHANNELS) return;

        for (let ch = 0; ch < CHANNELS; ch++) {
          const arr = batch[ch];
          if (!Array.isArray(arr)) continue;
          const buf = buffers[ch];
          for (let i = 0; i < arr.length; i++) {
            buf.push(arr[i]);
            buf.shift();
          }
        }

        sampleCount += batch[0] ? batch[0].length : 0;
        totalSamples += batch[0] ? batch[0].length : 0;

        // 每秒更新一次采样率显示
        const now = Date.now();
        const elapsed = now - lastCountTime;
        if (elapsed >= 1000) {
          const rate = Math.round(sampleCount / (elapsed / 1000));
          sampleRateEl.textContent = '采样率: ' + rate + ' SPS';
          sampleCount = 0;
          lastCountTime = now;
        }
      } catch (err) {
        // JSON 解析失败，静默跳过
      }
    };

    ws.onclose = function() {
      setStatus(false);
      setTimeout(connect, 1500);
    };

    ws.onerror = function() {
      ws.close();
    };
  }

  // 启动连接
  connect();
})();
</script>
</body>
</html>
)EOF";

// ================== HTTP 请求处理 ==================
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

// ================== WebSocket 事件处理 ==================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      D_PRINTF("[WS] 客户端 #%u 断开\n", num);
      break;
    case WStype_CONNECTED:
      D_PRINTF("[WS] 客户端 #%u 已连接 (IP: %s)\n",
                    num, webSocket.remoteIP(num).toString().c_str());
      break;
    case WStype_TEXT:
      // 暂不处理客户端消息（可扩展为接收模式切换命令）
      break;
    default:
      break;
  }
}

// ================== 构建批量 JSON ==================
String buildBatchJson() {
  String json;
  json.reserve(2800);
  json += '[';
  for (int ch = 0; ch < 8; ch++) {
    json += '[';
    for (int i = 0; i < BATCH_SIZE; i++) {
      json += String(batchBuffer[ch][i], 6);
      if (i < BATCH_SIZE - 1) json += ',';
    }
    json += ']';
    if (ch < 7) json += ',';
  }
  json += ']';
  return json;
}

// ================== setup ==================
void setup() {
  // ---------- 1. 初始化双串口 + LED（确保至少一路能看到输出）----------
  pinMode(LED_PIN, OUTPUT);
  // 快速闪烁 = "我还活着，正在启动"
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH); // LED 亮 = 启动中

  // 同时初始化两个串口：
  //   Serial  → 原生 USB CDC (GPIO18/19) — 需 ARDUINO_USB_MODE=1
  //   Serial0 → 硬件 UART0 (GPIO20/21)    — 走 CH340 芯片
  Serial.begin(115200);
  Serial0.begin(115200);

  // ⚠️ USB CDC 枚举需要时间（Win 约1~2秒），等久一点确保串口软件连上
  delay(2000);

  // 打印启动横幅（两个串口都发）
  D_PRINTLN("\n=========================================");
  D_PRINTLN("  ADS1299 EEG WiFi 模式启动中...");
  D_PRINTLN("=========================================");
  D_PRINTLN("[INFO] 如果看到此行，说明串口通信正常");
  D_PRINTLN("[INFO] 板载 LED: 快闪=启动中  长亮=就绪  慢闪=WiFi失败");

  // ============ 先启动 WiFi（不依赖 ADS1299）============
  D_PRINTF("\n[WiFi] 正在创建热点: %s ...\n", WIFI_SSID);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(wifiPowerLevel); // 再设置功率
  bool apOk = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  if (apOk) {
    IPAddress ip = WiFi.softAPIP();
    D_PRINTF("[WiFi] ✅ 热点已就绪!\n");
    D_PRINTF("[WiFi] 📡 SSID: %s\n", WIFI_SSID);
    D_PRINTF("[WiFi] 🔑 密码: %s\n", WIFI_PASSWORD);
    D_PRINTF("[WiFi] 🌐 IP:   %s\n", ip.toString().c_str());
    D_PRINTF("[WiFi] 🔗 浏览器打开: http://%s\n", ip.toString().c_str());
    digitalWrite(LED_PIN, LOW); // LED 短暂熄灭
    delay(200);
    digitalWrite(LED_PIN, HIGH);
  } else {
    // WiFi 热点创建失败 → 死循环/慢闪 LED，双串口持续打印
    D_PRINTF("[WiFi] ❌ 热点 \"%s\" 创建失败!\n", WIFI_SSID);
    D_PRINTLN("[WiFi] ╔══════════════════════════════════════╗");
    D_PRINTLN("[WiFi] ║  可能原因:                           ║");
    D_PRINTLN("[WiFi] ║  1. 供电不足 → 换 5V/1A 独立电源     ║");
    D_PRINTLN("[WiFi] ║  2. 天线缺失 → 检查 IPEX 天线是否接  ║");
    D_PRINTLN("[WiFi] ║  3. 硬件故障 → 烧录最小 WiFi 测试固件 ║");
    D_PRINTLN("[WiFi] ║  4. 插错USB口→试试板上另一个USB口    ║");
    D_PRINTLN("[WiFi] ╚══════════════════════════════════════╝");
    while (1) {
      wifiRetryCount++;
      D_PRINTF("[WiFi] ❌ 热点创建失败! 已尝试 %d 次, 5秒后重试...\n", wifiRetryCount);
      // 慢闪 LED = WiFi 故障
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(LED_BLINK_SLOW);
      }
      bool apOk = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
      if (apOk) {
        IPAddress ip = WiFi.softAPIP();
        D_PRINTF("[WiFi] ✅ 第 %d 次重试成功!\n", wifiRetryCount);
        D_PRINTF("[WiFi] 🌐 IP: %s\n", ip.toString().c_str());
        digitalWrite(LED_PIN, HIGH);
        break;
      }
    }
  }

  // ============ 启动 HTTP + WebSocket 服务器 ============
  server.on("/", handleRoot);
  server.onNotFound([]() {
    server.send(404, "text/plain", "404 Not Found");
  });
  server.begin();
  D_PRINTLN("[HTTP] ✅ 网页服务器已启动 (端口 80)");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  D_PRINTLN("[WS]  ✅ WebSocket 服务器已启动 (端口 81)");

  // ============ 然后初始化 ADS1299（依赖硬件连接）============
  D_PRINTLN("\n[ADS] 正在初始化 ADS1299...");

  pinMode(CS_PIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);
  pinMode(START_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  digitalWrite(CS_PIN, HIGH);
  digitalWrite(START_PIN, LOW);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);

  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE1));

  initADS1299();
  getDeviceID();

  attachInterrupt(DRDY_PIN, onDRDYInterrupt, FALLING);
  currentMode = MODE_CONTINUOUS_READ;
  D_PRINTLN("[ADS] ✅ ADS1299 初始化完成\n");

  D_PRINTLN("=========================================");
  D_PRINTLN("  ✅ 系统就绪，等待客户端连接...");
  D_PRINTLN("  📱 用手机/电脑连接 WiFi 热点:");
  D_PRINTF("     SSID: %s  密码: %s\n", WIFI_SSID, WIFI_PASSWORD);
  D_PRINTLN("  🌐 浏览器打开: http://192.168.4.1");
  D_PRINTLN("=========================================\n");

  digitalWrite(LED_PIN, HIGH); // LED 常亮 = 系统就绪
}

// ================== loop ==================
void loop() {
  // 处理 HTTP 请求

  D_PRINT("[ADS] Device ID: 0b");
  Serial.println(id, BIN);           // 二进制打印（如 0b111110）
  D_PRINTF(" (0x%02X)\n", id);        // 也显示十六进制，方便对照数据手册

  server.handleClient();

  // 处理 WebSocket 事件
  webSocket.loop();

  // 处理 ADS1299 数据
  if (dataReady) {
    dataReady = false;
    readData();  // 读取一帧数据到 channelDataBuffer[1..8]

    // 添加到批处理缓冲
    for (int ch = 0; ch < 8; ch++) {
      batchBuffer[ch][batchCount] = channelDataBuffer[ch + 1];
    }
    batchCount++;

    // 当缓冲满 BATCH_SIZE 个采样点时广播
    if (batchCount >= BATCH_SIZE) {
      // 仅在有客户端连接时构建并发送
      if (webSocket.connectedClients() > 0) {
        String json = buildBatchJson();
        webSocket.broadcastTXT(json);
      }
      batchCount = 0;
    }
  }
}

// ================== DRDY 中断服务函数 ==================
void IRAM_ATTR onDRDYInterrupt() {
  dataReady = true;
}

// ================== 模式配置（与 main.cpp 完全一致）==================
void startContinuousReadMode() {
  sendCommand(RESET);
  delay(100);
  sendCommand(SDATAC);

  writeRegister(0x01, CONFIG1_REG);
  writeRegister(0x02, CONFIG2_REG);
  writeRegister(0x03, CONFIG3_REG);

  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, CHnSET_REG);

  writeRegister(0x0D, BIAS_SENSP);
  writeRegister(0x0E, BIAS_SENSN);
  writeRegister(0x15, ENABLE_SRB1);

  sendCommand(START);
  sendCommand(RDATAC);
}

void startImpedanceMeasurementMode() {
  sendCommand(RESET);
  delay(100);
  sendCommand(SDATAC);

  writeRegister(0x04, LOFF_CONFIG);
  writeRegister(0x0F, LOFF_SENSP);
  writeRegister(0x0E, LOFF_SENSN);
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, CHnSET_REG);

  sendCommand(START);
  sendCommand(RDATAC);
  D_PRINTLN("[MODE] 阻抗测量模式");
}

void startSelfTestMode() {
  sendCommand(RESET);
  delay(100);
  sendCommand(SDATAC);

  writeRegister(0x01, 0x95);
  writeRegister(0x02, 0xD1);
  writeRegister(0x03, CONFIG3_REG);
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, 0x65);

  sendCommand(START);
  sendCommand(RDATAC);
  D_PRINTLN("[MODE] 自检模式");
}

// ================== ADS1299 初始化（与 main.cpp 完全一致）==================
void initADS1299() {
  sendCommand(RESET);
  delay(100);
  sendCommand(SDATAC);

  writeRegister(0x01, CONFIG1_REG);
  writeRegister(0x02, CONFIG2_REG);
  writeRegister(0x03, CONFIG3_REG);

  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, CHnSET_REG);

  writeRegister(0x0D, BIAS_SENSP);
  writeRegister(0x0E, BIAS_SENSN);
  writeRegister(0x15, ENABLE_SRB1);

  writeRegister(0x04, 0x00);
  writeRegister(0x0F, 0x00);
  writeRegister(0x10, 0x00);

  sendCommand(START);
  sendCommand(RDATAC);
}

// ================== 数据读取（与 main.cpp 完全一致）==================
void readData() {
  uint8_t data[27];
  digitalWrite(CS_PIN, LOW);

  for (int i = 0; i < 27; i++) data[i] = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);
  convertData(data, channelDataBuffer);
}

// ================== 数据转换（与 main.cpp 完全一致）==================
void convertData(uint8_t *data, double *channelData) {
  long statusValue = ((long)data[0] << 16) | ((long)data[1] << 8) | data[2];
  channelData[0] = (double)statusValue;

  for (int i = 0; i < 8; i++) {
    long raw = ((long)data[3*i+3] << 16) | ((long)data[3*i+4] << 8) | data[3*i+5];
    if (raw & 0x800000) raw |= 0xFF000000;

    double vPerLSB = 4.5 / (24.0 * 8388608.0);
    channelData[i+1] = (double)raw * vPerLSB;
  }
}

// ================== 底层 SPI 操作（与 main.cpp 完全一致）==================
void sendCommand(uint8_t cmd) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(cmd);
  digitalWrite(CS_PIN, HIGH);
}

void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(RREG | reg);
  SPI.transfer(0x00);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return val;
}

void getDeviceID() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(SDATAC);              // 先退出连续读模式
  SPI.transfer(RREG | 0x00);         // 读寄存器 0x00（ID 寄存器）
  SPI.transfer(0x00);                // 读 1 个寄存器
  id = SPI.transfer(0x00);   // 接收 ID 值
  digitalWrite(CS_PIN, HIGH);

  // ✅ 重要: 重新进入连续读取模式，否则 DRDY 不会翻转，数据不会更新！
  sendCommand(START);
  sendCommand(RDATAC);

  D_PRINT("[ADS] Device ID: 0b");
  Serial.println(id, BIN);           // 二进制打印（如 0b111110）
  D_PRINTF(" (0x%02X)\n", id);        // 也显示十六进制，方便对照数据手册
}
