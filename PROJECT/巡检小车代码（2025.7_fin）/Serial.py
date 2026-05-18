import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque

# 配置串口参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际情况修改串口
BAUD_RATE = 9600  # 波特率
TIMEOUT = 1  # 超时时间

# 数据阈值
VOLT_THRESHOLD = 5.0
TEMP_THRESHOLD = 30
HUMID_THRESHOLD = 60
METER_THRESHOLD = 10.0

# 初始化数据队列
MAX_DATA_POINTS = 100  # 最大显示数据点数
volt_data = deque(maxlen=MAX_DATA_POINTS)
temp_data = deque(maxlen=MAX_DATA_POINTS)
humid_data = deque(maxlen=MAX_DATA_POINTS)
meter_data = deque(maxlen=MAX_DATA_POINTS)

# 初始化时间轴
time_data = deque(maxlen=MAX_DATA_POINTS)

# 初始化串口
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

# 初始化matplotlib
fig, ax = plt.subplots(2, 2, figsize=(12, 8))
ax_volt, ax_temp, ax_humid, ax_meter = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]

# 设置标题和标签
ax_volt.set_title('Voltage')
ax_temp.set_title('Temperature')
ax_humid.set_title('Humidity')
ax_meter.set_title('Meter')

# 初始化曲线
line_volt, = ax_volt.plot([], [], 'b-')
line_temp, = ax_temp.plot([], [], 'r-')
line_humid, = ax_humid.plot([], [], 'g-')
line_meter, = ax_meter.plot([], [], 'y-')

# 标注阈值线
ax_volt.axhline(y=VOLT_THRESHOLD, color='b', linestyle='--')
ax_temp.axhline(y=TEMP_THRESHOLD, color='r', linestyle='--')
ax_humid.axhline(y=HUMID_THRESHOLD, color='g', linestyle='--')
ax_meter.axhline(y=METER_THRESHOLD, color='y', linestyle='--')

# 初始化标注
annotation_volt = ax_volt.annotate('', xy=(0, 0), xytext=(10, 10),
                                    textcoords='offset points', color='b',
                                    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
annotation_temp = ax_temp.annotate('', xy=(0, 0), xytext=(10, 10),
                                    textcoords='offset points', color='r',
                                    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
annotation_humid = ax_humid.annotate('', xy=(0, 0), xytext=(10, 10),
                                      textcoords='offset points', color='g',
                                      bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                                      arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
annotation_meter = ax_meter.annotate('', xy=(0, 0), xytext=(10, 10),
                                      textcoords='offset points', color='y',
                                      bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                                      arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))

# 隐藏标注
annotation_volt.set_visible(False)
annotation_temp.set_visible(False)
annotation_humid.set_visible(False)
annotation_meter.set_visible(False)

# 更新函数
def update(frame):
    try:
        # 读取串口数据
        line = ser.readline().decode('utf-8').strip()
        if line:
            # 解析数据
            parts = line.split()
            volt = float(parts[0].split('=')[1])
            temp = int(parts[1].split('=')[1])
            humid = int(parts[2].split('=')[1])
            meter = float(parts[3].split('=')[1])

            # 添加数据到队列
            time_data.append(frame)
            volt_data.append(volt)
            temp_data.append(temp)
            humid_data.append(humid)
            meter_data.append(meter)

            # 更新电压曲线
            line_volt.set_data(time_data, volt_data)
            ax_volt.set_xlim(min(time_data), max(time_data))
            ax_volt.set_ylim(min(volt_data) - 1, max(volt_data) + 1)
            if volt > VOLT_THRESHOLD:
                annotation_volt.set_text(f'Voltage: {volt:.2f}')
                annotation_volt.xy = (frame, volt)
                annotation_volt.set_visible(True)
            else:
                annotation_volt.set_visible(False)

            # 更新温度曲线
            line_temp.set_data(time_data, temp_data)
            ax_temp.set_xlim(min(time_data), max(time_data))
            ax_temp.set_ylim(min(temp_data) - 1, max(temp_data) + 1)
            if temp > TEMP_THRESHOLD:
                annotation_temp.set_text(f'Temperature: {temp}')
                annotation_temp.xy = (frame, temp)
                annotation_temp.set_visible(True)
            else:
                annotation_temp.set_visible(False)
    	# 更新湿度曲线
            line_humid.set_data(time_data, humid_data)
            ax_humid.set_xlim(min(time_data), max(time_data))
            ax_humid.set_ylim(min(humid_data) - 1, max(humid_data) + 1)
            if humid > HUMID_THRESHOLD:
                annotation_humid.set_text(f'Humidity: {humid}')
                annotation_humid.xy = (frame, humid)
                annotation_humid.set_visible(True)
            else:
                annotation_humid.set_visible(False)
            # 更新米数曲线
            line_meter.set_data(time_data, meter_data)
            ax_meter.set_xlim(min(time_data), max(time_data))
            ax_meter.set_ylim(min(meter_data) - 1, max(meter_data) + 1)
            if meter > METER_THRESHOLD:
                annotation_meter.set_text(f'Meter: {meter:.2f}')
                annotation_meter.xy = (frame, meter)
                annotation_meter.set_visible(True)
            else:
                annotation_meter.set_visible(False)
    except Exception as e:
        print(f"Error: {e}")
    return line_volt, line_temp, line_humid, line_meter, annotation_volt, annotation_temp, annotation_humid, annotation_meter
# 动画
ani = animation.FuncAnimation(fig, update, interval=500)  # 间隔0.5秒
# 显示图形
plt.tight_layout()
plt.show()