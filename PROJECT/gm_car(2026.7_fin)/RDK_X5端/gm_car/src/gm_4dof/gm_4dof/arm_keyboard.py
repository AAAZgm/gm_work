#!/usr/bin/env python3
"""
机械臂键盘控制节点（XYZ笛卡尔模式）

四轴机械臂结构：
  关节0 — 旋转底座，控制末端在水平面(XY)内的方向
  关节1 — 肩关节弯曲，控制末端Z轴高度（上抬/下降）
  关节2 — 肘关节弯曲，控制末端Z轴高度（上抬/下降）
  关节3 — 夹爪，夹紧/松开

舵机物理约定：
  所有关节90°时，机械臂垂直指向 +Z 轴
  底座角度增大 → 逆时针旋转
  肩/肘角度增大 → 升高（趋近垂直）

按键映射（世界坐标系，单位 mm）：
  w / s   X+ / X-（前进 / 后退）
  a / d   Y+ / Y-（左移 / 右移）
  q / e   Z+ / Z-（上升 / 下降）
  c       夹紧
  v       松开
  h       回家
  r / f   步进 ±1mm
  Ctrl+C  退出
"""

# ======== 标准库导入 ========
import sys           # sys.stdin 用于读取键盘输入
import select        # select.select 用于非阻塞键盘读取
import termios       # termios 保存/恢复终端设置（退出时恢复正常模式）
import tty           # tty.setraw 设置终端为原始模式（按键立即返回，不回显）
import math          # math 提供三角函数（sin/cos/atan2/acos），用于运动学计算

# ======== ROS2 导入 ========
import rclpy                          # ROS2 Python 客户端库
from rclpy.node import Node           # ROS2 节点基类
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # QoS 策略
from std_msgs.msg import Float32MultiArray  # ROS2 浮点数组消息，用于发布关节角度
from gm_4dof_interfaces.srv import Catch, Loosen, Home  # 机械臂服务接口

# ============================================================
# 机械臂连杆长度（mm）
# 用于正/逆运动学计算，根据实际硬件尺寸修改
# ============================================================
L1 = 80.0    # 底座高度：底座底部地面到肩关节的垂直距离
L2 = 105.0   # 大臂长度：肩关节轴心到肘关节轴心的距离
L3 = 150.0    # 小臂长度：肘关节轴心到末端执行器（夹爪根部）的距离

# 舵机角度物理限制范围（度）
SERVO_MIN = 0.0     # 舵机最小角度
SERVO_MAX = 180.0   # 舵机最大角度

# ============================================================
# XYZ 模式参数 —— 每次按键末端移动的距离
# ============================================================
XYZ_STEP_DEFAULT = 5.0   # 默认步进距离（mm），每次按键末端移动的毫米数
XYZ_STEP_MIN = 1.0       # 最小步进（mm），精确定位用
XYZ_STEP_MAX = 50.0      # 最大步进（mm），快速移动用
XYZ_STEP_CHANGE = 1.0    # 每次按 r/f 改变 xyz_step 的幅度（mm）

# ============================================================
# 固定角度值 —— 机械臂的几个特殊姿态
# ============================================================
HOME_ANGLES = [90.0, 90.0, 90.0, 90.0]  # 工作位：[底座, 肩, 肘, 夹爪]，肘弯曲留活动空间
GRIPPER_OPEN = 135.0   # 夹爪松开时的舵机角度（第4轴）
GRIPPER_CLOSE = 45.0   # 夹爪夹紧时的舵机角度（第4轴）

# ============================================================
# XYZ 模式按键映射表
# 格式: '按键': (终端显示名, 坐标轴索引, 方向)
#   坐标轴索引: 0=X, 1=Y, 2=Z
#   方向: +1=正方向, -1=负方向（实际运行时乘以 self.xyz_step）
# ============================================================
KEY_MAP_XYZ = {
    'w': ('X+', 0, +1),    # X 正方向：前进（远离底座）
    's': ('X-', 0, -1),    # X 负方向：后退（靠近底座）
    'a': ('Y+', 1, +1),    # Y 正方向：左移
    'd': ('Y-', 1, -1),    # Y 负方向：右移
    'q': ('Z+', 2, +1),    # Z 正方向：上升
    'e': ('Z-', 2, -1),    # Z 负方向：下降
}


# ============================================================
# 正运动学（Forward Kinematics, FK）
# 输入：舵机角度 [θ0, θ1, θ2]（度）
# 输出：末端世界坐标 [x, y, z]（mm）
# ============================================================
def forward_kinematics(angles):
    """
    正运动学：舵机角度 → 末端世界坐标

    舵机角度约定：
      θ0=90° → 臂平面沿 +X 轴（正前方）
      θ1=90° → 大臂垂直向上(Z+)，θ1<90° → 前倾（远离Z轴）
      θ2=90° → 小臂与大臂共线（无弯曲），θ2<90° → 肘下弯

    坐标系：X=前进, Y=左, Z=上, 原点在底座底部地面
    """
    # 将 3 个舵机角度从度转换为弧度
    t0 = math.radians(angles[0])  # 底座旋转角度（弧度）
    t1 = math.radians(angles[1])  # 肩关节角度（弧度）
    t2 = math.radians(angles[2])  # 肘关节角度（弧度）

    # ---- 舵机角度 → 内部角度 ----
    # 实际舵机行为：所有角度90°时垂直指向Z轴（向上）
    #   θ1=90° → 大臂垂直向上，θ1<90° → 前倾（远离Z轴）
    #   θ2=90° → 小臂与大臂共线（都向上），θ2<90° → 肘下弯
    #
    # 内部角度约定（标准运动学，从 +Z 轴量起，正=远离Z轴方向）：
    #   q1: 肩关节内部角度，从垂直向上(Z+)量起，正=前倾
    #     θ1=90°(垂直) → q1=0°, θ1<90°(前倾) → q1>0
    #   q2: 肘关节内部角度，相对大臂方向
    #     θ2=90°(无弯曲) → q2=0°, θ2<90°(肘下弯) → q2>0
    q1 = math.radians(90.0) - t1  # 垂直为零，前倾为正
    q2 = math.radians(90.0) - t2  # 无弯曲为零，肘下弯为正

    # ---- 2D 臂平面内计算 ----
    # q1 从 +Z 轴量起（0=垂直向上，正值=前倾远离Z轴）
    # 大臂末端: r = L2*sin(q1), z = L2*cos(q1)
    # 小臂末端（相对大臂偏转 q2）: r += L3*sin(q1+q2), z += L3*cos(q1+q2)
    r_plane = L2 * math.sin(q1) + L3 * math.sin(q1 + q2)  # 水平前伸距离
    z_plane = L2 * math.cos(q1) + L3 * math.cos(q1 + q2)  # 相对肩关节的高度

    # ---- 底座旋转映射到世界 3D 坐标 ----
    base_rot = t0 - math.radians(90.0)
    x = r_plane * math.cos(base_rot)  # X = 水平距离 × cos(底座旋转角)
    y = r_plane * math.sin(base_rot)  # Y = 水平距离 × sin(底座旋转角)
    z = z_plane + L1                  # Z = 臂平面高度 + 底座高度

    return [x, y, z]


# ============================================================
# 逆运动学（Inverse Kinematics, IK）
# 输入：末端世界坐标 (x, y, z)（mm）
# 输出：舵机角度 [θ0, θ1, θ2]（度），不可达时返回 None
# ============================================================
def inverse_kinematics(x, y, z, current_base=None):
    """
    逆运动学：末端世界坐标 → 舵机角度 [θ0, θ1, θ2]

    默认使用肘下解（elbow-down），适合桌面抓取场景。
    不可达时返回 None（目标距离超出臂展范围）。
    current_base: 当 r≈0 时的底座角度，避免奇异。
    """
    # 计算目标点在水平面上的投影距离
    r = math.sqrt(x ** 2 + y ** 2)

    # ---- 底座角度 θ0 ----
    if r < 1.0:
        # r≈0 时 atan2(y,x) 无意义（奇异点），保持当前底座角度
        if current_base is not None:
            theta0 = current_base
        else:
            return None
    else:
        # θ0 = atan2(y, x) + 90°（舵机角度约定）
        theta0 = math.degrees(math.atan2(y, x)) + 90.0

    # ---- 2D 臂平面内求解（肩关节为原点）----
    z_shoulder = z - L1                    # 目标相对肩关节的高度
    d_sq = r ** 2 + z_shoulder ** 2        # 肩关节到目标的距离平方
    d = math.sqrt(d_sq)                    # 肩关节到目标的距离

    # 可达性检查：留 0.1mm 余量避免数值不稳定
    if d > L2 + L3 - 0.1 or d < abs(L2 - L3) + 0.1:
        return None  # 不可达

    # ---- 余弦定理求肘关节角 q2 ----
    cos_q2 = (d_sq - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
    cos_q2 = max(-1.0, min(1.0, cos_q2))  # 浮点精度保护
    q2 = math.acos(cos_q2)                 # 正值 = 肘下弯

    # ---- 求肩关节角 q1 ----
    # 目标角度：从 +Z 轴量起
    k1 = L2 + L3 * cos_q2
    k2 = L3 * math.sin(q2)
    q1 = math.atan2(r, z_shoulder) - math.atan2(k2, k1)

    # ---- 内部角度 → 舵机角度 ----
    theta1 = 90.0 - math.degrees(q1)
    theta2 = 90.0 - math.degrees(q2)

    # 钳位到舵机物理范围 [0°, 180°]
    theta0 = max(SERVO_MIN, min(SERVO_MAX, theta0))
    theta1 = max(SERVO_MIN, min(SERVO_MAX, theta1))
    theta2 = max(SERVO_MIN, min(SERVO_MAX, theta2))

    return [theta0, theta1, theta2]


# ============================================================
# 节点类：机械臂笛卡尔键盘控制
# ============================================================
class ArmKeyboard(Node):
    """
    机械臂笛卡尔键盘控制节点

    通过逆运动学（IK）控制末端在世界坐标系 XYZ 中的位置。

    通信话题：
      发布 /arm_joint_commands  → 发送关节角度命令到微控制器
      订阅 /arm_joint_angles   ← 接收微控制器反馈的实际关节角度
    """

    def __init__(self):
        super().__init__('arm_keyboard')

        # 创建发布器：向微控制器发送关节角度命令
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/arm_joint_commands', 10)

        # 创建订阅器：接收微控制器反馈的实际关节角度
        # 使用 BEST_EFFORT 与 micro-ROS 发布端保持一致
        qos_be = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/arm_joint_angles',
            self._angle_cb, qos_be)

        # 当前关节角度 [底座, 肩, 肘, 夹爪]，初始为 HOME 位
        self.angles = list(HOME_ANGLES)

        # ---- 服务客户端（夹紧/松开/回家通过 dof_control 服务调用）----
        self.home_client = self.create_client(Home, '/home_object')
        self.catch_client = self.create_client(Catch, '/catch_object')
        self.loosen_client = self.create_client(Loosen, '/loosen_object')

        # ---- XYZ 状态 ----
        self.xyz_step = XYZ_STEP_DEFAULT      # 步进距离（mm）
        self.target_xyz = forward_kinematics(self.angles)  # 从 HOME 位算出起始坐标

        # 保存当前终端设置
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            f'初始位置 XYZ=({self.target_xyz[0]:.1f}, {self.target_xyz[1]:.1f}, '
            f'{self.target_xyz[2]:.1f}) mm')

    # ------ 工具方法 ------
    def _status_line(self, action=''):
        """生成格式化的状态行"""
        xyz = self.target_xyz
        ang = self.angles
        s = f'  [{action}]  ' if action else '  '
        s += (f'XYZ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) '
              f'Joint[{ang[0]:.1f}, {ang[1]:.1f}, {ang[2]:.1f}, {ang[3]:.1f}]')
        if self.xyz_step != XYZ_STEP_DEFAULT:
            s += f'  step={self.xyz_step:.0f}mm'
        return '\r' + s + '                    '  # 尾部空格清残留

    # ------ 回调 ------
    def _angle_cb(self, msg):
        """关节角度反馈回调：更新当前角度为微控制器报告的实际角度"""
        data = list(msg.data)
        # ESP32 micro-ROS 在 index 0 附加了时间戳，跳过它（和 dof_control 一致）
        self.angles = data[1:] if len(data) > 4 else data

    # ------ 发布 ------
    def _publish(self):
        """将当前 self.angles 发布到 /arm_joint_commands 话题"""
        msg = Float32MultiArray()
        msg.data = self.angles
        self.cmd_pub.publish(msg)

    # ------ 非阻塞读键 ------
    def _get_key(self):
        """
        非阻塞读取一个键盘按键
        select.select 检查 stdin 是否可读，timeout=0 立即返回
        """
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    # ------ IK 移动 ------
    def _ik_move(self, dx=0, dy=0, dz=0, action=''):
        """
        XYZ 模式下移动末端并求解 IK
        参数 dx/dy/dz: 方向（+1 或 -1），实际移动量 = 方向 × self.xyz_step
        action: 动作显示名，如 'X+'
        """
        # 在当前目标坐标上叠加位移
        self.target_xyz[0] += dx * self.xyz_step
        self.target_xyz[1] += dy * self.xyz_step
        self.target_xyz[2] += dz * self.xyz_step

        # 地面碰撞保护：Z 不能低于 0
        if self.target_xyz[2] < 0.0:
            self.target_xyz[2] = 0.0

        # 用逆运动学求解：目标 XYZ → 关节角度
        x, y, z = self.target_xyz
        result = inverse_kinematics(x, y, z, current_base=self.angles[0])

        if result is None:
            # 不可达 → 回退 target_xyz（撤销位移）
            self.target_xyz[0] -= dx * self.xyz_step
            self.target_xyz[1] -= dy * self.xyz_step
            self.target_xyz[2] -= dz * self.xyz_step
            print(f'\r  \033[31m[不可达]\033[0m ({x:.1f}, {y:.1f}, {z:.1f}) 已回退')
            return

        # IK 求解成功 → 更新关节角度并发布
        self.angles[0] = result[0]
        self.angles[1] = result[1]
        self.angles[2] = result[2]
        self._publish()
        print(self._status_line(action))

    # ------ 服务调用 ------
    def _call_service_async(self, client, srv_type, action_name, delay=2):
        """异步调用服务，不阻塞键盘输入"""
        if not client.service_is_ready():
            print(f'\r  \033[31m[错误] {action_name}服务未就绪\033[0m')
            return
        req = srv_type.Request()
        req.delay_time = delay
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self._service_done_cb(f, action_name))

    def _service_done_cb(self, future, action_name):
        """服务调用完成回调"""
        try:
            result = future.result()
            status = '\033[32m成功\033[0m' if result.result else '\033[31m失败\033[0m'
        except Exception as e:
            status = f'\033[31m异常: {e}\033[0m'
        print(f'\r  {action_name} {status}')

    # ------ 主循环 ------
    def run(self):
        """主循环：打印操作说明 → 进入 raw 模式 → 循环读取按键并发送命令"""
        tty.setraw(sys.stdin.fileno())

        # 打印操作说明
        print(f"""
                -- Arm XYZ Control --\nw/s:X+/-  a/d:Y+/-  q/e:Z+/-\nc:grab  v:release  h:home\nstep={self.xyz_step:.0f}mm  Ctrl+C:quit\n{self._status_line('ready')}
                """)

        try:
            while rclpy.ok():
                # 非阻塞处理 ROS2 回调（50ms）
                rclpy.spin_once(self, timeout_sec=0.05)

                key = self._get_key()
                if key is None:
                    continue
                if key == '\x03':  # Ctrl+C
                    break

                # ---------- 夹爪 / 回家（通过服务调用）----------
                if key == 'h':
                    self._call_service_async(
                        self.home_client, Home, '\033[36m[回家]\033[0m')
                    # 同步更新本地状态
                    self.angles = list(HOME_ANGLES)
                    self.target_xyz = forward_kinematics(self.angles)
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
                    self.xyz_step = min(
                        self.xyz_step + XYZ_STEP_CHANGE, XYZ_STEP_MAX)
                    print(self._status_line(f'步进 {self.xyz_step:.0f}mm'))
                    continue
                if key == 'f':
                    self.xyz_step = max(
                        self.xyz_step - XYZ_STEP_CHANGE, XYZ_STEP_MIN)
                    print(self._status_line(f'步进 {self.xyz_step:.0f}mm'))
                    continue

                # ---------- XYZ 移动 ----------
                if key in KEY_MAP_XYZ:
                    action, axis, delta = KEY_MAP_XYZ[key]
                    self._ik_move(
                        dx=delta if axis == 0 else 0,
                        dy=delta if axis == 1 else 0,
                        dz=delta if axis == 2 else 0,
                        action=action,
                    )

        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端为正常模式
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print('\n\033[32m已退出键盘控制\033[0m')


def main():
    """程序入口：初始化 ROS2 → 创建节点 → 运行主循环 → 清理"""
    rclpy.init()
    node = ArmKeyboard()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
