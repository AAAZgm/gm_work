# 脑电 (EEG/BCI) 采集系统

基于 ESP32-C3 + ADS1299 的 8 通道脑电信号采集与处理系统。

## 功能

- 8 通道 EEG 信号同步采集
- ADS1299 内部自检模式
- 电极阻抗测量（导联脱落检测）
- SPI 高速数据传输
- 串口命令行切换工作模式
- Python 上位机实时波形显示

## 技术栈

- **主控**: ESP32-C3 (RISC-V)
- **模拟前端**: TI ADS1299 (8 通道 ΔΣ ADC, 24 位)
- **通信**: SPI (硬件 SPI, 最高 10MHz)
- **固件**: PlatformIO (Arduino 框架)
- **上位机**: Python (pyserial + matplotlib)

## 目录结构

```
BCI_Code/
├── src/main.cpp          — 主程序 (ADS1299 驱动 + SPI 通信 + 命令处理)
├── include/              — 头文件
├── lib/                  — 第三方库
├── platformio.ini        — PlatformIO 配置
├── plotter.py            — PC 上位机实时波形显示
├── requirements.txt      — Python 依赖
└── test/                 — 测试代码
```

## 模式说明

| 命令 | 模式 | 说明 |
|------|------|------|
| `1` | 连续读取 | 实时采集并输出 8 通道 EEG 数据 |
| `2` | 阻抗测量 | 测量各通道电极与皮肤接触阻抗 |
| `3` | 自检 | ADS1299 内部方波/正弦波自检信号 |

## 硬件清单

- ESP32-C3 开发板
- ADS1299 前端模拟板
- 脑电电极 × 8 (参考 + 偏置)
- PCB 文件: `Gerber_PCB1_2026-04-23_BCI.zip`

## 使用

```bash
# 上位机
pip install pyserial matplotlib
python plotter.py
```

> ⚠ **注意事项**: 本项目为个人学习研究用途，不构成医疗设备。请遵守当地法律法规。
