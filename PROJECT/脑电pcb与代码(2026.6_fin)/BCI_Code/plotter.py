"""
ADS1299 EEG 数据实时可视化工具
用法: python plotter.py [COM端口号]
示例: python plotter.py COM3
"""

import sys
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ================== 配置区 ==================
SERIAL_BAUD = 115200        # 串口波特率，与 ESP32 Serial.begin() 保持一致
MAX_POINTS = 500            # 显示的采样点数（越多越慢）
CHANNELS = 8                # ADS1299 的 8 个 EEG 通道
UPDATE_INTERVAL = 30        # 刷新间隔 ms
PORT = None                 # COM 端口号，可通过命令行指定或自动检测
# ============================================


def find_esp32_port():
    """自动查找可能的 ESP32 串口"""
    ports = serial.tools.list_ports.comports()
    # 常见的 ESP32 设备标识
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "ch340" in desc or "cp210" in desc or "usb-serial" in desc \
           or "esp32" in desc or "esp32s2" in desc or "esp32c3" in desc \
           or "silicon" in desc:
            print(f"[INFO] 自动识别到设备: {p.device} ({p.description})")
            return p.device
    if ports:
        for p in ports:
            if not p.device.startswith("COM") and not p.device.startswith("/dev/tty.Bluetooth"):
                return p.device
    return None


class EEGPlotter:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.buf = ""
        # 每个通道一个 deque 缓冲区（环形队列），固定长度 MAX_POINTS
        self.data = [deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for _ in range(CHANNELS)]
        self.fig = None
        self.axes = None
        self.lines = []
        self.channel_colors = [
            '#e41a1c', '#377eb8', '#4daf4a', '#984ea3',
            '#ff7f00', '#ffff33', '#a65628', '#f781bf'
        ]

    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"[OK] 已连接 {self.port} @ {self.baud}bps")
            return True
        except Exception as e:
            print(f"[ERROR] 无法打开串口: {e}")
            print(f"       请检查: 1) 端口号是否正确  2) USB线是否连接  3) 设备是否被占用")
            return False

    def setup_plot(self):
        """初始化 matplotlib 图表布局"""
        plt.style.use('dark_background')
        self.fig, self.axes = plt.subplots(CHANNELS, 1, sharex=True,
                                            figsize=(12, 10),
                                            gridspec_kw={'hspace': 0.05})
        self.fig.suptitle('ADS1299 EEG 实时数据', fontsize=14, color='white')

        y_range = 5e-6  # +/- 5uV 初始范围，会自动调整

        for i, ax in enumerate(self.axes):
            line, = ax.plot([], [], color=self.channel_colors[i], linewidth=0.8)
            self.lines.append(line)
            ax.set_ylabel(f'CH{i+1}', fontsize=9, rotation=0,
                          labelpad=15, va='center')
            ax.set_ylim(-y_range, y_range)
            ax.grid(True, alpha=0.2)
            ax.tick_params(labelsize=7)

        self.axes[-1].set_xlabel('Sample', fontsize=10)
        plt.tight_layout(rect=[0, 0, 1, 0.97])

    def parse_serial_line(self):
        """读取并解析一行串口数据"""
        while self.ser.in_waiting > 0:
            raw = self.ser.readline()
            try:
                line = raw.decode('utf-8').strip()
            except UnicodeDecodeError:
                continue

            if not line.startswith("channel:"):
                continue

            values_str = line[len("channel:"):]
            parts = values_str.split(",")
            if len(parts) != CHANNELS:
                continue

            try:
                values = [float(p) for p in parts]
            except ValueError:
                continue

            for i in range(CHANNELS):
                if i < len(values):
                    self.data[i].append(values[i])

    def update(self, frame):
        """动画更新回调函数 - 每帧调用一次"""
        self.parse_serial_line()

        all_vals = []
        for i in range(CHANNELS):
            dlist = list(self.data[i])
            self.lines[i].set_data(range(len(dlist)), dlist)
            all_vals.extend(dlist)

        # 自动调整 Y 轴范围
        if all_vals:
            vmin, vmax = min(all_vals), max(all_vals)
            margin = max(abs(vmin), abs(vmax)) * 0.15 + 1e-10
            ymin = vmin - margin
            ymax = vmax + margin
            for ax in self.axes:
                ax.set_ylim(ymin, ymax)

        return self.lines

    def run(self):
        """启动可视化主循环"""
        if not self.connect():
            sys.exit(1)
        self.setup_plot()

        ani = animation.FuncAnimation(
            self.fig, self.update,
            interval=UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )

        print("[INFO] 波形窗口已打开，关闭窗口即退出")
        print("[INFO] 按 Ctrl+C 终止")
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("\n[INFO] 串口已关闭")


def main():
    global PORT

    if len(sys.argv) > 1:
        PORT = sys.argv[1]
    else:
        PORT = find_esp32_port()
        if not PORT:
            print("[ERROR] 未找到 ESP32 串口!")
            print("       请手动指定端口: python plotter.py <COM端口号>")
            print("")
            print("       可用串口列表:")
            for p in serial.tools.list_ports.comports():
                print(f"         {p.device}  -  {p.description}")
            sys.exit(1)
        else:
            print(f"[INFO] 使用端口: {PORT}")

    print("=" * 50)
    print("  ADS1299 EEG 实时可视化工具")
    print("=" * 50)
    print(f"  端口:   {PORT}")
    print(f"  波特率: {SERIAL_BAUD}")
    print(f"  通道数: {CHANNELS}")
    print(f"  显示点: {MAX_POINTS}")
    print("=" * 50)
    print("[提示] 在串口中发送 '1' 开始连续读取模式")
    print("[提示] 发送 '2' 阻抗测量 / '3' 自检模式\n")

    plotter = EEGPlotter(PORT, SERIAL_BAUD)
    plotter.run()


if __name__ == "__main__":
    main()
