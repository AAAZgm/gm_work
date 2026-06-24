#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from gm_car_interfaces.msg import RobotStatus


class OminiRobotNode(Node):
    def __init__(self, nodename):
        super().__init__(nodename)
        
        # ============ 接收帧格式配置（下位机发送，17字节）============
        self.FRAME_HEAD = 0xAA
        self.FRAME_TAIL = 0x7E
        self.FRAME_LENGTH = 17  # 改为17
        
        # 偏移量定义
        self.BAT_VOLT_OFFSET = 1
        self.VX_OFFSET = 3
        self.VY_OFFSET = 5
        self.W_OFFSET = 7
        self.Y_POSITION = 9
        self.X_POSITION = 11
        self.ANGLE_POSITION = 13
        self.CHECK_OFFSET = 15   # 改为15
        self.FRAME_TAIL_POS = 16  # 帧尾位置
        
        # ============ 下发帧格式配置（发给下位机，16字节）============
        self.RE_FRAME_LENGTH = 16
        self.RE_FRAME_HEAD = 0xAA
        self.REVX_OFFSET = 1
        self.REVY_OFFSET = 3
        self.REW_OFFSET = 5
        self.TASKER_OFFSET = 13
        self.RECHECK_OFFSET = 14
        self.RE_FRAME_TAIL = 0x01  # 0x01=RDK, 0x02=HC_08
        
        self.default_task = 0
        
        # 缓冲区
        self.buffer = bytearray()
        
        # ============ 创建发布者 ============
        self.robot_pub = self.create_publisher(RobotStatus, 'robot_state', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # ============ 订阅 cmd_vel ============
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
        
        # 定时器
        self.timer = self.create_timer(0.05, self.read_serial)
        self.init_task_timer = self.create_timer(1.0, self.send_init_task)

    def send_task(self, task_id: int):
        """向下位机发送任务号"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error('串口未打开，无法发送任务')
            return False
        
        frame = bytearray(self.RE_FRAME_LENGTH)
        frame[0] = self.RE_FRAME_HEAD
        frame[1:3] = struct.pack('<h', 0)
        frame[3:5] = struct.pack('<h', 0)
        frame[5:7] = struct.pack('<h', 0)
        frame[self.TASKER_OFFSET] = task_id & 0xFF
        frame[self.RECHECK_OFFSET] = sum(frame[:self.RECHECK_OFFSET]) & 0xFF
        frame[15] = self.RE_FRAME_TAIL
        
        try:
            self.serial_port.write(frame)
            self.get_logger().info(f'已发送任务号: {task_id}')
            return True
        except Exception as e:
            self.get_logger().error(f'发送任务失败: {e}')
            return False

    def send_init_task(self):
        INIT_TASK_ID = 1
        self.send_task(INIT_TASK_ID)
        self.get_logger().info('启动初始化任务已发送')
        self.init_task_timer.cancel()

    def cmd_vel_callback(self, msg: Twist):
        """接收cmd_vel并下发给下位机"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        vx_raw = int(msg.linear.x * 1000)
        vy_raw = int(msg.linear.y * 1000)
        w_raw = int(msg.angular.z * 1000)
        
        frame = bytearray(self.RE_FRAME_LENGTH)
        frame[0] = self.RE_FRAME_HEAD
        frame[self.REVX_OFFSET:self.REVX_OFFSET+2] = struct.pack('<h', vx_raw)
        frame[self.REVY_OFFSET:self.REVY_OFFSET+2] = struct.pack('<h', vy_raw)
        frame[self.REW_OFFSET:self.REW_OFFSET+2] = struct.pack('<h', w_raw)
        frame[self.TASKER_OFFSET] = self.default_task
        frame[self.RECHECK_OFFSET] = sum(frame[:self.RECHECK_OFFSET]) & 0xFF
        frame[15] = self.RE_FRAME_TAIL
        
        try:
            self.serial_port.write(frame)
            self.get_logger().info(f'下发: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, w={msg.angular.z:.3f}')
        except Exception as e:
            self.get_logger().error(f'串口写入错误: {e}')

    def read_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                self.buffer.extend(data)
            self.parse_frame()
        except Exception as e:
            self.get_logger().error(f'读取错误: {e}')

    def parse_frame(self):
        while len(self.buffer) >= self.FRAME_LENGTH:
            try:
                head_idx = self.buffer.index(self.FRAME_HEAD)
            except ValueError:
                self.buffer.clear()
                return
            
            if head_idx > 0:
                self.buffer = self.buffer[head_idx:]
            
            if len(self.buffer) < self.FRAME_LENGTH:
                return
            
            frame = bytes(self.buffer[:self.FRAME_LENGTH])
            self.buffer = self.buffer[self.FRAME_LENGTH:]
            
            if frame[self.FRAME_TAIL_POS] != self.FRAME_TAIL:
                self.get_logger().warn('帧尾错误')
                continue
            
            if not self.verify_checksum(frame):
                self.get_logger().warn('校验和失败')
                continue
            
            self.publish_data(frame)

    def verify_checksum(self, frame):
        calculated = sum(frame[:self.CHECK_OFFSET]) & 0xFF
        received = frame[self.CHECK_OFFSET]
        return calculated == received

    def publish_data(self, frame):
        try:
            # 解析数据
            bat_int = frame[self.BAT_VOLT_OFFSET]
            bat_float = frame[self.BAT_VOLT_OFFSET + 1]
            battery_voltage = bat_int + bat_float / 100.0
            
            vx_raw = struct.unpack('<h', frame[self.VX_OFFSET:self.VX_OFFSET+2])[0]
            vy_raw = struct.unpack('<h', frame[self.VY_OFFSET:self.VY_OFFSET+2])[0]
            w_raw = struct.unpack('<h', frame[self.W_OFFSET:self.W_OFFSET+2])[0]
            vx = vx_raw / 1000.0
            vy = vy_raw / 1000.0
            angular_velocity = w_raw / 1000.0
            
            x_raw = struct.unpack('<h', frame[self.X_POSITION:self.X_POSITION+2])[0]
            y_raw = struct.unpack('<h', frame[self.Y_POSITION:self.Y_POSITION+2])[0]
            x_position = x_raw / 1000.0
            y_position = y_raw / 1000.0
            
            angle_raw = struct.unpack('<h', frame[self.ANGLE_POSITION:self.ANGLE_POSITION+2])[0]
            angle = angle_raw / 1000.0
            
            # 发布自定义消息
            msg = RobotStatus()
            msg.stamp = self.get_clock().now().to_msg()
            msg.battery_voltage = battery_voltage
            msg.linear_velocity = vx
            msg.angular_velocity = angular_velocity
            msg.x_position = x_position
            msg.y_position = y_position
            msg.angle = angle
            msg.is_valid = True
            self.robot_pub.publish(msg)
            
            # 发布 odom
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            
            odom_msg.pose.pose.position.x = x_position
            odom_msg.pose.pose.position.y = y_position
            odom_msg.pose.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, angle)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = angular_velocity
            
            self.odom_pub.publish(odom_msg)
            
            self.get_logger().info(
                f'电压: {battery_voltage:.2f}V, '
                f'速度: vx={vx:.3f}, vy={vy:.3f}, w={angular_velocity:.3f}, '
                f'位置: x={x_position:.3f}, y={y_position:.3f}, θ={angle:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'解析数据错误: {e}')

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OminiRobotNode('serial2ros2_omini')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
