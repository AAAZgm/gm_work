# 巡检小车

基于 STM32F103 的智能巡检小车嵌入式控制系统，配合上位机实时数据监控。

## 功能

- 温湿度检测（DHT11）
- 超声波避障/测距
- OLED 实时显示传感器数据
- 电压监测（ADC）
- 串口通信 + 上位机 Python 实时波形显示

## 技术栈

- **主控**: STM32F103 (Cortex-M3)
- **开发环境**: Keil MDK (uvprojx)
- **上位机**: Python (pyserial + matplotlib 实时绘图)
- **外设驱动**: GPIO、ADC、Serial、Timer、I2C

## 目录结构

```
car_dht/    — STM32 下位机固件 (Keil 工程)
Serial.py   — PC 上位机串口数据接收与实时可视化脚本
```

## 使用方法

### 下位机
用 Keil MDK 打开 `car_dht/Project.uvprojx`，编译下载至 STM32F103。

### 上位机
```bash
pip install pyserial matplotlib numpy
python Serial.py
```
根据实际串口修改 `Serial.py` 中的 `SERIAL_PORT`。

## 硬件清单

- STM32F103 最小系统板
- 0.96" OLED 显示屏
- DHT11 温湿度传感器
- HC-SR04 超声波模块
- 电机驱动模块 + 直流电机
- 电压检测分压电路
