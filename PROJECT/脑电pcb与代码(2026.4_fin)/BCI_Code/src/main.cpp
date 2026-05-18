#include <Arduino.h>      // ESP32 Arduino 核心，提供 Serial/pinMode/delay 等（类似 STM32 HAL + BSP 合集）
#include <SPI.h>           // ESP32 SPI 驱动库，封装了硬件 SPI 外设（类似 STM32 HAL_SPI）
#include "esp_timer.h"     // ESP-IDF 高精度定时器接口，本工程暂未使用，预留

// ================== 宏定义区 ==================
// ---------- 采样率配置（对应 ADS1299 寄存器 0x01~0x03） ----------
#define CONFIG1_REG  0x95   // 寄存器1: [7]=1 Daisy链模式关闭(单片), [6:5]=01 500SPS, [2:0]=101 采样率=500SPS
#define CONFIG2_REG  0xC0   // 寄存器2: [7]=1 内部测试信号关, [6]=0 常规(非测试)模式, 其余0
#define CONFIG3_REG  0xEC   // 寄存器3: [7]=1 内部参考缓冲开启, [5]=1 BIAS缓冲器开启, [4]=1 BIAS_REF内连, [3]=0 PD_REFBUF关, [2:0]=100 BIAS_MEAS=OFF

// ---------- 通道配置（对应寄存器 0x05~0x0C, 0x0D, 0x0E, 0x15） ----------
#define CHnSET_REG   0x60   // 通道n设置: [7]=1 正常输入(非关断), [6:4]=110 增益=24x, [3]=0 正常输入, [2:0]=000 普通电极
#define ENABLE_SRB1  0x20   // MISC1寄存器(0x15): [5]=1 启用 SRB1(右腿驱动参考)
#define BIAS_SENSP   0xFF   // BIAS_SENSP寄存器(0x0D): 所有通道正端连接到BIAS
#define BIAS_SENSN   0xFF   // BIAS_SENSN寄存器(0x0E): 所有通道负端连接到BIAS

// ---------- 阻抗测量配置（对应寄存器 0x04, 0x0F, 0x10） ----------
#define LOFF_CONFIG   0x06  // LOFF寄存器(0x04): [3:2]=01 6nA交流注入电流, [6:4]=000 FLEAD_OFF=6Hz, [5]=1 COMP_TH=95%
#define LOFF_SENSP    0xFF  // LOFF_SENSP寄存器(0x0F): 所有通道正端开启导联脱落检测/注入
#define LOFF_SENSN    0xFF  // LOFF_SENSN寄存器(0x10): 所有通道负端开启导联脱落检测/注入

// ---------- 模式选择（串口命令 '1'/'2'/'3' 切换） ----------
#define MODE_CONTINUOUS_READ   1    // 模式1: 连续读取 EEG 数据
#define MODE_IMPEDANCE_MEASURE 2    // 模式2: 阻抗测量（需外部解调电路）
#define MODE_SELF_TEST         3    // 模式3: ADS1299 内部自检

// ================== 引脚定义（ESP32-C3 GPIO 编号） ==================
// ESP32-C3 GPIO 编号范围 0~21，不同于 STM32 的 PA0/PB5 命名方式
// ⚠️ 注意: ESP32-C3 固定引脚占用:
//   GPIO0  = BOOT 按键/启动模式选择（不要用）
//   GPIO1  = UART0 TX / USB CDC TX（Serial 用！不要占用）
//   GPIO2  = 可用
//   GPIO3  = UART0 RX / USB CDC RX（Serial 用！不要占用）
//   GPIO4~7 = 建议用于 SPI（Flash/SRAM 不占用）
//   GPIO8~9 = Flash 集成 SPI（不可用）
//   GPIO11~17 = Flash/SRAM 集成（不可用）
//   GPIO18~21 = 可用
#define CS_PIN    10    // SPI 片选（避开 GPIO1/3，改用 GPIO10）
#define SCLK_PIN  6     // SPI 时钟线（ESP32-C3 默认 SPI2 SCK）
#define MOSI_PIN  5     // SPI 主出从入（ESP32→ADS1299）
#define MISO_PIN  4     // SPI 主入从出（ADS1299→ESP32）
#define DRDY_PIN  9     // ADS1299 数据就绪信号（避开 GPIO3，改用 GPIO9）
#define START_PIN 8     // ADS1299 START 引脚（任意可用 GPIO）
#define RESET_PIN 7     // ADS1299 硬件复位引脚（避开 GPIO1，改用 GPIO7）

// ================== ADS1299 SPI 命令定义（参考 ADS1299 数据手册） ==================
#define WAKEUP  0x02    // 唤醒：退出 STANDBY 模式
#define STANDBY 0x04    // 待机：降低功耗
#define RESET   0x06    // 复位：软件复位，所有寄存器恢复默认值
#define START   0x08    // 启动：开始数据转换（等效拉高 START 引脚）
#define STOP    0x0A    // 停止：停止数据转换（等效拉低 START 引脚）
#define RDATAC  0x10    // 连续读数据模式：进入后 ADS1299 自动在每个 DRDY 后准备好新数据帧
#define SDATAC  0x11    // 停止连续读数据：回到停止连续读模式，之后可读写寄存器
#define RDATA   0x12    // 单次读数据：读一帧数据（仅在 STOP 模式下可用）
#define RREG    0x20    // 读寄存器命令前缀（| 寄存器地址）
#define WREG    0x40    // 写寄存器命令前缀（| 寄存器地址）

// ================== 全局变量 ==================
volatile bool dataReady = false; // DRDY 中断标志，volatile 防止编译器优化（中断与主循环共享变量）
int currentMode = MODE_CONTINUOUS_READ; // 当前工作模式
double channelDataBuffer[9]; // 数据缓冲区：[0]=STATUS寄存器, [1]~[8]=8通道转换结果（电压值V）

// ================== 函数声明 ==================
// IRAM_ATTR: 将函数放入 IRAM（指令 RAM），ESP32 中断回调必须放在 IRAM 中
// 原因: ESP32 的 Flash 访问不能在中断中安全使用，IRAM 保证零等待执行
// 类比 STM32: 不需要特殊处理，因为 STM32 Flash 通常可直接在中断中执行
void IRAM_ATTR onDRDYInterrupt(); // DRDY 下降沿中断服务函数
void initADS1299();               // ADS1299 上电初始化
void startContinuousReadMode();   // 切换到连续读取模式
void startImpedanceMeasurementMode(); // 切换到阻抗测量模式
void startSelfTestMode();         // 切换到内部自检模式
void readData();                  // 通过 SPI 读取一帧数据（27字节: 3字节STATUS + 8x3字节通道数据）
void convertData(uint8_t *data, double *channelData); // 将原始字节解析为电压值
void sendCommand(uint8_t cmd);    // 发送单字节 SPI 命令
void writeRegister(uint8_t reg, uint8_t value); // 写 ADS1299 寄存器
uint8_t readRegister(uint8_t reg);               // 读 ADS1299 寄存器
void getDeviceID();                // 读取 ADS1299 ID 寄存器验证通信

// ================== setup ==================
// Arduino 入口函数，相当于 STM32 的 main() 中初始化部分
// Arduino 框架会先调用 setup()，再循环调用 loop()
void setup() {
  Serial.begin(115200); // 初始化 USB 串口，波特率 115200（ESP32-C3 内置 USB CDC，类似 STM32 的 USB虚拟串口）

  // ---------- 初始化 GPIO 引脚 ----------
  // pinMode / digitalWrite 等价于 STM32 HAL 的 HAL_GPIO_Init() + HAL_GPIO_WritePin()
  pinMode(CS_PIN, OUTPUT);    // CS 引脚设为推挽输出
  pinMode(DRDY_PIN, INPUT);   // DRDY 引脚设为输入（连接 ADS1299 DRDY 输出）
  pinMode(START_PIN, OUTPUT); // START 引脚设为推挽输出
  pinMode(RESET_PIN, OUTPUT); // RESET 引脚设为推挽输出

  // 初始化 GPIO 电平
  digitalWrite(CS_PIN, HIGH);  // CS 拉高=空闲状态，SPI 协议中 CS 低电平有效（类似 STM32 SPI NSS）
  digitalWrite(START_PIN, LOW); // START 拉低=停止转换
  digitalWrite(RESET_PIN, HIGH); // RESET 拉高=正常工作，拉低=复位
  delay(100); // 等待 100ms，让 ADS1299 上电稳定（类似 HAL_Delay()）

  // ---------- 初始化 SPI 外设 ----------
  // ESP32 Arduino SPI 库封装了 ESP-IDF 的 SPI 驱动
  // SPI.begin(sck, miso, mosi, cs) 指定引脚并初始化硬件 SPI2 外设
  // 类比 STM32: HAL_SPI_Init() + 配置 SPI GPIO
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // SPI.beginTransaction() 配置 SPI 参数，在 ESP32 上会真正生效
  // SPISettings(频率Hz, 位序, 模式):
  //   1000000   = SPI 时钟频率 1MHz（ADS1299 最高 20MHz，1MHz 保守稳定）
  //   SPI_MSBFIRST = 高位先发（ADS1299 要求 MSB first）
  //   SPI_MODE1 = CPOL=0, CPHA=1（ADS1299 要求的 SPI 模式）
  // 类比 STM32: hspi.Init.CLKPolarity = SPI_POLARITY_LOW; hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
  SPI.beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE1));

  // ---------- 初始化 ADS1299 ----------
  initADS1299();  // 配置 ADS1299 所有寄存器
  getDeviceID();  // 读 ID 寄存器验证 SPI 通信正常
  Serial.println("ADS1299 初始化完成");

  // ---------- 配置外部中断 ----------
  // attachInterrupt(GPIO号, 回调函数, 触发方式)
  // FALLING = 下降沿触发（ADS1299 DRDY 从高变低表示数据准备好）
  // 类比 STM32: HAL_GPIO_EXTI_Callback() + EXTI 配置
  // 注意: ESP32 的 GPIO 中断比 STM32 更灵活，任意 GPIO 都可配置中断
  attachInterrupt(DRDY_PIN, onDRDYInterrupt, FALLING);

  currentMode = MODE_CONTINUOUS_READ; // 默认进入连续读取模式
}

// ================== loop ==================
// Arduino 主循环，相当于 STM32 main() 中的 while(1)
// Arduino 框架会无限循环调用此函数
void loop() {
  // ---------- 串口命令处理 ----------
  // Serial.available() 检查串口接收缓冲区是否有数据（类似 STM32 的 UART RXNE 标志）
  if (Serial.available()) {
    char cmd = Serial.read(); // 读取一个字节（类似 HAL_UART_Receive）
    if (cmd == '1') {
      // 串口发送 '1' → 进入连续读取 EEG 模式
      currentMode = MODE_CONTINUOUS_READ;
      startContinuousReadMode();
    } else if (cmd == '2') {
      // 串口发送 '2' → 进入阻抗测量模式
      currentMode = MODE_IMPEDANCE_MEASURE;
      startImpedanceMeasurementMode();
    } else if (cmd == '3') {
      // 串口发送 '3' → 进入自检模式
      currentMode = MODE_SELF_TEST;
      startSelfTestMode();
    }
  }

  // ---------- 数据读取与输出 ----------
  // dataReady 由 DRDY 中断设置为 true，主循环中读取并处理
  if (dataReady) {
    dataReady = false;          // 先清除标志（防止重复读取）
    readData();                 // 从 ADS1299 SPI 读取一帧数据（27字节）并转换为电压值
    Serial.print("channel:");
    for (int i = 1; i <= 8; i++) {
      Serial.print(channelDataBuffer[i], 6); // 打印通道数据，6位小数（单位: 伏特V）
      if (i < 8) Serial.print(","); // 逗号分隔
    }
    Serial.println(); // 换行
  }
}

// ================== 中断服务函数 ==================
// DRDY 下降沿触发此函数
// ⚠️ ESP32 限制: ISR 必须在 IRAM 中，且不能调用 Flash 中的函数
// digitalWrite / SPI.transfer 不在 IRAM 中，所以 ISR 只设置标志
// 实际数据读取移到主循环中完成（类似 STM32 的"中断+轮询"模式）
void IRAM_ATTR onDRDYInterrupt() {
  dataReady = true; // 仅通知主循环有新数据就绪
}

// ================== 模式配置 ==================
// 进入连续读取模式：配置 ADS1299 为标准 EEG 采集状态
void startContinuousReadMode() {
  sendCommand(RESET);  // 先复位 ADS1299，所有寄存器恢复默认
  delay(100);          // 等待复位完成（数据手册要求 tRST ≥ 2^18 / fCLK ≈ 18ms，100ms 充裕）
  sendCommand(SDATAC); // 退出连续读模式，准备写寄存器

  // 写配置寄存器
  writeRegister(0x01, CONFIG1_REG);  // 配置1: 单片模式, 500SPS
  writeRegister(0x02, CONFIG2_REG);  // 配置2: 常规采样, 无测试信号
  writeRegister(0x03, CONFIG3_REG);  // 配置3: 内部参考+BIAS缓冲开启

  // 写 8 个通道设置（寄存器 0x05~0x0C）
  // ADS1299 共 8 个通道（1~8），通道设置寄存器从 0x05 开始，每个通道一个寄存器
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, CHnSET_REG); // 全部设为增益24x, 普通电极

  writeRegister(0x0D, BIAS_SENSP);  // BIAS 正端连接：所有通道开启
  writeRegister(0x0E, BIAS_SENSN);  // BIAS 负端连接：所有通道开启
  writeRegister(0x15, ENABLE_SRB1); // MISC1: 启用 SRB1 右腿驱动参考

  sendCommand(START);  // 开始数据转换
  sendCommand(RDATAC); // 进入连续读数据模式，此后 DRDY 自动翻转
}

// 进入阻抗测量模式：开启导联脱落检测的交流注入电流
// 注意: 此模式需要外部解调电路才能得到实际阻抗值
void startImpedanceMeasurementMode() {
  sendCommand(RESET);
  delay(100);
  sendCommand(SDATAC);

  writeRegister(0x04, LOFF_CONFIG);  // 配置导联脱落检测：6nA注入电流
  writeRegister(0x0F, LOFF_SENSP);   // 所有通道正端开启注入
  writeRegister(0x0E, LOFF_SENSN);   // 所有通道负端开启注入
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, CHnSET_REG); // 通道配置不变

  sendCommand(START);
  sendCommand(RDATAC);
  Serial.println("阻抗测量模式已启用（注意需解调导联频率信号）");
}

// 进入内部自检模式：ADS1299 内部产生 1mV 或 2mV 方波测试信号
// 可验证 SPI 通信和 ADC 转换是否正常
void startSelfTestMode() {
  sendCommand(RESET);
  delay(100);
  sendCommand(SDATAC);

  writeRegister(0x01, 0x95); // CONFIG1: 500SPS
  writeRegister(0x02, 0xD1); // CONFIG2: [7:6]=11 开启内部测试信号（~1mV方波连接到各通道输入）
  writeRegister(0x03, CONFIG3_REG);
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, 0x65); // 通道设为测试信号输入

  sendCommand(START);
  sendCommand(RDATAC);
  Serial.println("自检模式已启用");
}

// ================== ADS1299 上电初始化 ==================
// 配置 ADS1299 所有寄存器到 EEG 采集默认状态
void initADS1299() {
  sendCommand(RESET);  // 软件复位
  delay(100);          // 等待复位完成
  sendCommand(SDATAC); // 停止连续读，准备配置寄存器

  // 写核心配置寄存器
  writeRegister(0x01, CONFIG1_REG);  // 采样率 500SPS
  writeRegister(0x02, CONFIG2_REG);  // 常规模式
  writeRegister(0x03, CONFIG3_REG);  // 内部参考 + BIAS 缓冲

  // 写 8 通道设置
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(i, CHnSET_REG); // 增益24x, 普通电极

  // BIAS（右腿驱动）配置
  writeRegister(0x0D, BIAS_SENSP);  // 正端 BIAS 感测
  writeRegister(0x0E, BIAS_SENSN);  // 负端 BIAS 感测
  writeRegister(0x15, ENABLE_SRB1); // 启用 SRB1

  // 导联脱落检测默认关闭
  writeRegister(0x04, 0x00);  // LOFF 配置: 关闭
  writeRegister(0x0F, 0x00);  // LOFF_SENSP: 全关
  writeRegister(0x10, 0x00);  // LOFF_SENSN: 全关

  sendCommand(START);  // 启动数据转换
  sendCommand(RDATAC); // 进入连续读数据模式
}

// ================== 数据读取 ==================
// 从 ADS1299 读取一帧数据：3字节STATUS + 8通道 x 3字节 = 27字节
// ⚠️ 此函数在主循环中调用（不在 ISR 中），因为 SPI/digitalWrite 不在 IRAM 中
// 必须在 DRDY 下降沿后、下一个 DRDY 上升沿前完成读取（500SPS 下有 ~2ms 时间窗口）
void readData() {
  uint8_t data[27]; // 27字节接收缓冲区
  digitalWrite(CS_PIN, LOW); // 拉低 CS，选中 ADS1299

  // 连续 27 次 SPI 传输，每次收发一个字节
  // SPI.transfer(0x00): 发送 0x00 作为 dummy 时钟，同时接收 ADS1299 返回的数据
  // 类比 STM32: HAL_SPI_TransmitReceive() 发送 dummy 接收数据
  for (int i = 0; i < 27; i++) data[i] = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH); // 拉高 CS，释放 ADS1299
  convertData(data, channelDataBuffer); // 将原始字节转换为电压值

  // 注意: dataReady 标志由 ISR 设置，由主循环清除，此处不操作
}

// ================== 数据转换 ==================
// 将 ADS1299 原始 24 位有符号补码数据转换为实际电压值
// ADS1299 数据格式: 每通道 3 字节 (24位有符号数，MSB first)
// 电压计算: V = raw * VREF / (Gain * 2^23)
// 其中 VREF=4.5V（内部参考2.425V x 2倍），Gain=24
void convertData(uint8_t *data, double *channelData) {
  // 解析 STATUS 字节（前 3 字节）
  long statusValue = ((long)data[0] << 16) | ((long)data[1] << 8) | data[2];
  channelData[0] = (double)statusValue; // 保存原始 STATUS（一般不用）

  // 解析 8 个通道数据（从第 4 字节开始，每通道 3 字节）
  for (int i = 0; i < 8; i++) {
    // 拼接 3 字节为 24 位值（MSB first）
    long raw = ((long)data[3*i+3] << 16) | ((long)data[3*i+4] << 8) | data[3*i+5];

    // 24 位有符号数符号扩展到 32 位
    // 如果第 23 位（最高位）为 1，表示负数，需要将高 8 位全部置 1
    if (raw & 0x800000) raw |= 0xFF000000;

    // 计算每 LSB 对应的电压
    // VREF = 4.5V, Gain = 24, 分辨率 = 2^23 = 8388608
    // vPerLSB = 4.5 / (24.0 * 8388608.0) ≈ 22.35nV
    double vPerLSB = 4.5 / (24.0 * 8388608.0);

    // 转换为实际电压（单位: 伏特）
    channelData[i+1] = (double)raw * vPerLSB;
  }
}

// ================== 底层 SPI 操作 ==================

// 发送 ADS1299 单字节命令
// 流程: 拉低CS → 发送命令字节 → 拉高CS
// 类比 STM32: 拉低 NSS → HAL_SPI_Transmit() → 拉高 NSS
void sendCommand(uint8_t cmd) {
  digitalWrite(CS_PIN, LOW);  // 选中 ADS1299
  SPI.transfer(cmd);           // 发送命令字节
  digitalWrite(CS_PIN, HIGH); // 释放 ADS1299
}

// 写 ADS1299 寄存器
// ADS1299 写寄存器协议: [WREG|地址] [0x00(读1个寄存器)] [数据]
// 第二字节表示要连续写多少-1 个寄存器，写 1 个就是 0x00
void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(WREG | reg);  // 发送写命令 + 寄存器地址
  SPI.transfer(0x00);        // 发送数量-1（写1个寄存器=0x00）
  SPI.transfer(value);       // 发送要写入的值
  digitalWrite(CS_PIN, HIGH);
}

// 读 ADS1299 寄存器
// ADS1299 读寄存器协议: [RREG|地址] [0x00(读1个寄存器)] [dummy] → 收到寄存器值
uint8_t readRegister(uint8_t reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(RREG | reg);  // 发送读命令 + 寄存器地址
  SPI.transfer(0x00);        // 发送数量-1（读1个寄存器=0x00）
  uint8_t val = SPI.transfer(0x00); // 发送 dummy 字节，同时接收寄存器值
  digitalWrite(CS_PIN, HIGH);
  return val;
}

// 读取 ADS1299 设备 ID（寄存器 0x00）
// 正常 ADS1299 的 ID = 0b00111110 (0x3E)，ADS1299R = 0b00111111 (0x3F)
// 如果读到的值不对，说明 SPI 通信有问题（接线、电平等）
void getDeviceID() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(SDATAC);              // 先退出连续读模式（防止当前处于 RDATAC 状态）
  SPI.transfer(RREG | 0x00);         // 读寄存器 0x00（ID 寄存器）
  SPI.transfer(0x00);                // 读 1 个寄存器
  uint8_t id = SPI.transfer(0x00);   // 接收 ID 值
  digitalWrite(CS_PIN, HIGH);
  Serial.print("Device ID: 0b");
  Serial.println(id, BIN);           // 以二进制打印，方便对照数据手册
}
