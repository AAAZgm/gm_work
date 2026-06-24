#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct #struct 是 Python 标准库，用于二进制数据的打包和解包。
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler#转换的函数
from gm_car_interfaces.msg import RobotStatus  # 注意：消息名是RobotStatus不是RobotStatus


class DiguaRobotNode(Node):#继承node,获得定时器，发布者，订阅，etc
    def __init__(self,nodename):
        super().__init__(nodename)
        self.print_log_count = 0  # 打印计数器
        self.init_task_count = 0   # 初始化任务发送计数器  ← 新增这行
        
        # ============ 帧格式配置 ============
        
        self.FRAME_HEAD = 0xAA
        self.FRAME_TAIL = 0x7E
        

        # ============ 下发帧格式配置 ============
        self.RE_FRAME_HEAD = 0xAA  
        self.RE_FRAME_TAIL = 0x01
        self.default_task = 0  # 默认任务号
        
        # 偏移量定义
        self.RE_FRAME_LENGTH = 16
        self.FRAME_LENGTH = 16
        self.BAT_VOLT_OFFSET = 1
        self.V_OFFSET = 3
        self.W_OFFSET = 5
        self.X_POSITION = 7
        self.Y_POSITION = 9
        self.ANGLE_POSITION = 11
        self.CHECK_OFFSET = 14
        self.TASKER_OFFSET = 13      # 任务号位置（对应下位机 TASKER-1）



        

        # 缓冲区
        self.buffer = bytearray()
        
        # ============ 创建发布者 ============
        self.robot_pub = self.create_publisher(RobotStatus, 'robot_state', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)  # odom发布
        
        # from std_msgs.msg import String
        # self.raw_pub = self.create_publisher(String, 'raw_frame', 10)
        
        # ============ 订阅 cmd_vel ============，也是回调函数
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # ============ 打开串口 ============
        serial_port = '/dev/gm_robot_1'
        baudrate = 115200
        
        try:
            self.serial_port = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=0.001
            )
            self.get_logger().info(f'已打开串口: {serial_port}, 波特率: {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口: {e}')
            self.serial_port = None
        
        # 定时器：定时读取一次回调函数
        self.timer = self.create_timer(0.05, self.read_serial)
        self.init_task_timer = self.create_timer(0.01, self.send_init_task)
        # ============ 新增：发送任务号的函数 ============
    def send_task(self, task_id: int):
        """
        向下位机发送任务号
        :param task_id: 任务号 (1-255)
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error('串口未打开，无法发送任务')
            return False
        
        # 构建任务帧（与速度帧格式相同，只是多填了任务号）
        frame = bytearray(self.RE_FRAME_LENGTH)
        frame[0] = self.RE_FRAME_HEAD              # 帧头 0xAA
        frame[1:3] = struct.pack('<h', 0)          # V速度 = 0
        frame[3:5] = struct.pack('<h', 0)          # W速度 = 0
        frame[self.TASKER_OFFSET] = task_id & 0xFF # 任务号,确保只取低8位
        frame[14] = sum(frame[:14]) & 0xFF         # 校验和
        frame[15] = self.RE_FRAME_TAIL             # 帧尾 0x01
        
        try:
            self.serial_port.write(frame)
            self.get_logger().info(f'已发送任务号: {task_id}')
            return True
        except Exception as e:
            self.get_logger().error(f'发送任务失败: {e}')
            return False
    
    # ============ 新增：启动时发送初始化任务 ============
    def send_init_task(self):
        """节点启动后发送初始化任务"""
        # 根据你的需求修改这里的任务号
        INIT_TASK_ID = 1  # 例如：任务1 = 初始化
        self.send_task(INIT_TASK_ID)
        self.init_task_count += 1
        if self.init_task_count >= 3:
            self.init_task_timer.cancel()  # 发够10次后取消定时器
        self.get_logger().info(f'已完成 {self.init_task_count} 次任务{INIT_TASK_ID}')

    
    def cmd_vel_callback(self, msg: Twist):
        """接收cmd_vel并下发给下位机"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        # 提取线速度和角速度，转换为整数（乘以1000）
        v_raw = int(msg.linear.x * 1000)
        w_raw = int(msg.angular.z * 1000)
        
        # 构建下发帧
        frame = bytearray(self.RE_FRAME_LENGTH)
        frame[0] = self.RE_FRAME_HEAD           # 帧头
        frame[1:3] = struct.pack('<h', v_raw)   # V速度
        frame[3:5] = struct.pack('<h', w_raw)   # W速度
        # frame[5:12] 保持为0或根据协议填充
        frame[self.TASKER_OFFSET] = self.default_task
        frame[14] = sum(frame[:14]) & 0xFF      # 校验和
        frame[15] = self.RE_FRAME_TAIL          # 帧尾
        
        try:
            self.serial_port.write(frame)
            self.get_logger().info(f'下发: v={msg.linear.x:.3f}, w={msg.angular.z:.3f}')
        except Exception as e:
            self.get_logger().error(f'串口写入错误: {e}')
    
    def read_serial(self):
        """读取串口数据"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting > 0:#硬件缓冲区不为空
                data = self.serial_port.read(self.serial_port.in_waiting)#读入
                self.buffer.extend(data)#把刚从串口读取的数据追加到缓冲区末尾
            
            self.parse_frame()
            
        except Exception as e:
            self.get_logger().error(f'读取错误: {e}')
    
    def parse_frame(self):
        """解析数据帧"""
        while len(self.buffer) >= self.FRAME_LENGTH:
            try:
                head_idx = self.buffer.index(self.FRAME_HEAD)#获取头位置
            except ValueError:
                self.buffer.clear()
                return
            
            if head_idx > 0:
                self.buffer = self.buffer[head_idx:]
            
            if len(self.buffer) < self.FRAME_LENGTH:#小了就不读
                return
            
            frame = bytes(self.buffer[:self.FRAME_LENGTH])#从头取到16，转为bytes类型
            self.buffer = self.buffer[self.FRAME_LENGTH:]#从16位后从新开始，前面的没用，py释放了
            
            if frame[self.FRAME_LENGTH - 1] != self.FRAME_TAIL:
                self.get_logger().warn(f'帧尾错误')
                continue
            
            if not self.verify_checksum(frame):
                self.get_logger().warn(f'校验和失败')
                continue
            
            self.publish_data(frame)
    
    def verify_checksum(self, frame):
        """验证校验和"""
        calculated = sum(frame[:self.CHECK_OFFSET])
        received = frame[self.CHECK_OFFSET]
        return (calculated & 0xFF) == received
    
    def publish_data(self, frame):
        """解析并发布数据"""
        try:
            # ============ 解析数据 ============
            bat_int = frame[self.BAT_VOLT_OFFSET]
            bat_float = frame[self.BAT_VOLT_OFFSET + 1]
            battery_voltage = bat_int + bat_float / 100.0
            
            v_raw = struct.unpack('<h', frame[self.V_OFFSET:self.V_OFFSET+2])[0]
            linear_velocity = v_raw / 1000.0
            
            w_raw = struct.unpack('<h', frame[self.W_OFFSET:self.W_OFFSET+2])[0]
            angular_velocity = w_raw / 1000.0
            
            x_raw = struct.unpack('<h', frame[self.X_POSITION:self.X_POSITION+2])[0]
            y_raw = struct.unpack('<h', frame[self.Y_POSITION:self.Y_POSITION+2])[0]
            x_position = x_raw / 1000.0
            y_position = y_raw / 1000.0
            
            angle_raw = struct.unpack('<h', frame[self.ANGLE_POSITION:self.ANGLE_POSITION+2])[0]
            angle = angle_raw / 1000.0
            
            # ============ 发布自定义消息 ============
            msg = RobotStatus()
            msg.stamp = self.get_clock().now().to_msg()
            msg.battery_voltage = battery_voltage
            msg.linear_velocity = linear_velocity
            msg.angular_velocity = angular_velocity
            msg.x_position = x_position
            msg.y_position = y_position
            msg.angle = angle
            msg.is_valid = True
            self.robot_pub.publish(msg)
            
            # ============ 发布 odom ============
            odom_msg = Odometry()#创建一个空的里程计消息对象。
            odom_msg.header.stamp = self.get_clock().now().to_msg()#带上时间戳
            odom_msg.header.frame_id = 'odom'#父亲，也就是静止的
            odom_msg.child_frame_id = 'base_footprint'
            
            odom_msg.pose.pose.position.x = x_position
            odom_msg.pose.pose.position.y = y_position
            odom_msg.pose.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, angle)#四元
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            
            odom_msg.twist.twist.linear.x = linear_velocity
            odom_msg.twist.twist.angular.z = angular_velocity
            
            self.odom_pub.publish(odom_msg)
            
# 修改 publish_data 里的打印（约第 244-249 行）
            self.print_log_count += 1
            if self.print_log_count >= 100:  # 每 20 帧打印一次（约 1 秒打印一次）
                self.print_log_count = 0
                self.get_logger().info(
                    f'电压: {battery_voltage:.2f}V, '
                    f'速度: {linear_velocity:.3f}m/s, '
                    f'角速度: {angular_velocity:.3f}rad/s, '
                    f'位置: x={x_position:.3f}, y={y_position:.3f}, 角度={angle:.3f}rad')
            
        except Exception as e:
            self.get_logger().error(f'解析数据错误: {e}')
    
    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):#没传就是默认
    rclpy.init(args=args)#把命令行的传进去，相当于初始化好了 例子“内容: ['节点名', '--ros-args', '-r', '__node:=xxx']”
    node = DiguaRobotNode('serial2ros2')#创建对象
    try:
        rclpy.spin(node)#spin(node) 就是盯着这个 node，触发里面的定时器和回调，自动帮你调用这个类里面函数
        #spin里面的参数就是Node,我这里是子类
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
