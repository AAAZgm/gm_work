#!/usr/bin/env python3
"""
机械臂关节角度键盘控制节点

直接控制每个舵机的角度，不经过运动学计算。
适合调试和手动校准各关节。

四轴机械臂结构：
  关节0 — 旋转底座（角度增大 → 逆时针）
  关节1 — 肩关节（角度增大 → 升高）
  关节2 — 肘关节（角度增大 → 升高）
  关节3 — 夹爪

按键映射：
  1 / 2   关节0 角度 +/-
  3 / 4   关节1 角度 +/-
  5 / 6   关节2 角度 +/-
  7 / 8   关节3 角度 +/-
  c       夹紧
  v       松开
  h       回家
  r / f   步进 ±1°
  Ctrl+C  退出
"""

# ======== 标准库导入 ========
import sys
import select
import termios
import tty

# ======== ROS2 导入 ========
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from gm_4dof_interfaces.srv import Catch, Loosen, Home

# ============================================================
# 参数
# ============================================================
SERVO_MIN = 0.0
SERVO_MAX = 180.0

HOME_ANGLES = [90.0, 90.0, 90.0, 90.0]
GRIPPER_OPEN = 135.0
GRIPPER_CLOSE = 45.0

STEP_DEFAULT = 2.0   # 默认步进（度）
STEP_MIN = 0.5
STEP_MAX = 15.0
STEP_CHANGE = 0.5

# ============================================================
# 按键映射：'按键': (显示名, 关节索引, 方向)
# ============================================================
KEY_MAP = {
    '1': ('J0+', 0, +1),
    '2': ('J0-', 0, -1),
    '3': ('J1+', 1, +1),
    '4': ('J1-', 1, -1),
    '5': ('J2+', 2, +1),
    '6': ('J2-', 2, -1),
    '7': ('J3+', 3, +1),
    '8': ('J3-', 3, -1),
}


# ============================================================
# 节点类
# ============================================================
class ArmJointKeyboard(Node):

    def __init__(self):
        super().__init__('arm_joint_keyboard')

        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/arm_joint_commands', 10)

        qos_be = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/arm_joint_angles',
            self._angle_cb, qos_be)

        self.angles = list(HOME_ANGLES)
        self.step = STEP_DEFAULT

        self.home_client = self.create_client(Home, '/home_object')
        self.catch_client = self.create_client(Catch, '/catch_object')
        self.loosen_client = self.create_client(Loosen, '/loosen_object')

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(
            f'初始角度: {self.angles}')

    # ------ 回调 ------
    def _angle_cb(self, msg):
        data = list(msg.data)
        self.angles = data[1:] if len(data) > 4 else data

    # ------ 发布 ------
    def _publish(self):
        msg = Float32MultiArray()
        msg.data = self.angles
        self.cmd_pub.publish(msg)

    # ------ 非阻塞读键 ------
    def _get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    # ------ 状态行 ------
    def _status_line(self, action=''):
        ang = self.angles
        s = f'  [{action}]  ' if action else '  '
        s += (f'J0={ang[0]:.1f}  J1={ang[1]:.1f}  '
              f'J2={ang[2]:.1f}  J3={ang[3]:.1f}')
        if self.step != STEP_DEFAULT:
            s += f'  step={self.step:.1f}°'
        return '\r' + s + '                    '

    # ------ 关节移动 ------
    def _joint_move(self, joint_idx, delta, action):
        new_val = self.angles[joint_idx] + delta * self.step
        new_val = max(SERVO_MIN, min(SERVO_MAX, new_val))
        self.angles[joint_idx] = new_val
        self._publish()
        print(self._status_line(action))

    # ------ 服务调用 ------
    def _call_service_async(self, client, srv_type, action_name, delay=2):
        if not client.service_is_ready():
            print(f'\r  \033[31m[错误] {action_name}服务未就绪\033[0m')
            return
        req = srv_type.Request()
        req.delay_time = delay
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self._service_done_cb(f, action_name))

    def _service_done_cb(self, future, action_name):
        try:
            result = future.result()
            status = '\033[32m成功\033[0m' if result.result else '\033[31m失败\033[0m'
        except Exception as e:
            status = f'\033[31m异常: {e}\033[0m'
        print(f'\r  {action_name} {status}')

    # ------ 主循环 ------
    def run(self):
        tty.setraw(sys.stdin.fileno())

        print(f"""
                -- Arm Joint Control --
1/2:J0+/-  3/4:J1+/-  5/6:J2+/-  7/8:J3+/-
c:grab  v:release  h:home
step={self.step:.1f}°  Ctrl+C:quit
{self._status_line('ready')}
                """)

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                key = self._get_key()
                if key is None:
                    continue
                if key == '\x03':
                    break

                # ---------- 夹爪 / 回家 ----------
                if key == 'h':
                    self._call_service_async(
                        self.home_client, Home, '\033[36m[回家]\033[0m')
                    self.angles = list(HOME_ANGLES)
                    print(self._status_line('回家'))
                    continue

                if key == 'c':
                    self._call_service_async(
                        self.catch_client, Catch, '\033[36m[夹紧]\033[0m')
                    self.angles[3] = GRIPPER_CLOSE
                    print(self._status_line('夹紧'))
                    continue

                if key == 'v':
                    self._call_service_async(
                        self.loosen_client, Loosen, '\033[36m[松开]\033[0m')
                    self.angles[3] = GRIPPER_OPEN
                    print(self._status_line('松开'))
                    continue

                # ---------- 步进调节 ----------
                if key == 'r':
                    self.step = min(self.step + STEP_CHANGE, STEP_MAX)
                    print(self._status_line(f'步进 {self.step:.1f}°'))
                    continue
                if key == 'f':
                    self.step = max(self.step - STEP_CHANGE, STEP_MIN)
                    print(self._status_line(f'步进 {self.step:.1f}°'))
                    continue

                # ---------- 关节角度移动 ----------
                if key in KEY_MAP:
                    action, joint_idx, delta = KEY_MAP[key]
                    self._joint_move(joint_idx, delta, action)

        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print('\n\033[32m已退出键盘控制\033[0m')


def main():
    rclpy.init()
    node = ArmJointKeyboard()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
