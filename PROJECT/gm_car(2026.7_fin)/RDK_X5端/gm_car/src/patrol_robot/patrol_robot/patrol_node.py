# 导入ROS2核心库
import rclpy
import os
from rclpy.duration import Duration

from rclpy.time import Time

# 导入位姿消息类型：PoseStamped(带时间戳位姿)、Pose(纯位姿)
from geometry_msgs.msg import PoseStamped

# 导入Nav2简易导航器，用于控制机器人移动
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 导入TF2坐标变换相关库：变换缓存、监听
from tf2_ros import TransformListener, Buffer

# 导入欧拉角 ↔ 四元数转换函数
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# 导入自定义服务接口
from tts_asr_interfaces.srv import Tts

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 定义巡逻导航类，继承自BasicNavigator
class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node', namespace=''):
        super().__init__(node_name, namespace)
        
        self.buffer_ = Buffer()
        self.tf_listener = TransformListener(self.buffer_, self)
        
        # 声明参数
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.5])
        self.declare_parameter('image_save_path', '')
        self.declare_parameter('nav_timeout', 60.0)
        self.declare_parameter('patrol_interval', 1.0)

        # 获取参数
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.image_save_path_ = self.get_parameter('image_save_path').value

        self.tts_client = self.create_client(Tts, 'tts')
        self.bridge = CvBridge()
        self.latest_image = None
        self.sub_img = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        self.latest_image = msg

    def record_image(self):
        # 主动等待最新图像，最多2秒
        wait_cnt = 0
        while self.latest_image is None and wait_cnt < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_cnt += 1
        if self.latest_image is None:
            self.get_logger().warn('没有收到图像数据，无法保存')
            return
    
        try:
            pose = self.get_current_pose()
            if pose is None:
                self.get_logger().error("获取位姿失败，跳过保存图片")
                return
        
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        
            if not os.path.exists(self.image_save_path_):
                os.makedirs(self.image_save_path_)
                self.get_logger().info(f'创建目录: {self.image_save_path_}')
        
            x = pose.translation.x
            y = pose.translation.y
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            filename = f'{sec}_{nsec}_x{x:.3f}_y{y:.3f}.png'
            filepath = os.path.join(self.image_save_path_, filename)
        
            if cv2.imwrite(filepath, cv_image):
                self.get_logger().info(f'✓ 图像保存成功: {filepath}')
            else:
                self.get_logger().error(f'✗ 图像保存失败: {filepath}')
        except Exception as e:
            self.get_logger().error(f'保存图像时出错: {str(e)}')

    def speech_text(self, text):
        if not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('TTS服务未上线')
            return
        req = Tts.Request()
        req.text = text
        future = self.tts_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            if res.result:
                self.get_logger().info('语音播报成功')
            else:
                self.get_logger().info('语音播报失败')
        else:
            self.get_logger().error('TTS服务调用失败')

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def init_robot_pose(self):
        init_pt = self.get_parameter('initial_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(init_pt[0], init_pt[1], init_pt[2]))
        self.waitUntilNav2Active()

    def get_target_points(self):
        points = []
        pts_list = self.get_parameter('target_points').value
        if len(pts_list) % 3 != 0:
            self.get_logger().error("目标点参数长度不是3的整数倍！")
            return points
        for i in range(len(pts_list) // 3):
            x = pts_list[i*3]
            y = pts_list[i*3+1]
            yaw = pts_list[i*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'GET {i+1} TARGET {x},{y},{yaw}')
        return points

    def nav_to_pose(self, target_pose):
        self.waitUntilNav2Active()
        timeout_s = self.get_parameter('nav_timeout').value
        self.goToPose(target_pose)

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            fb = self.getFeedback()
            if fb:
                remain = Duration.from_msg(fb.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'预计 {remain:.2f} s 后到达')
                if Duration.from_msg(fb.navigation_time) > Duration(seconds=timeout_s):
                    self.cancelTask()
                    self.get_logger().info(f'导航超时{timeout_s}s，已取消')

        res = self.getResult()
        if res == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
            return True
        elif res == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
            return False
        elif res == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
            return False
        else:
            self.get_logger().error('导航结果：状态无效')
            return False

    def get_current_pose(self, max_retries=10):
        retry = 0
        while rclpy.ok() and retry < max_retries:
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint',
                    Time(seconds=0), Duration(seconds=1)
                )
                trans = tf.transform
                rpy = euler_from_quaternion([trans.rotation.x,trans.rotation.y,trans.rotation.z,trans.rotation.w])
                self.get_logger().info(f'当前位置:{trans.translation}, 欧拉角:{rpy}')
                return trans
            except Exception as e:
                self.get_logger().warn(f'TF重试{retry+1}：{str(e)}')
                retry += 1
        self.get_logger().error(f'获取TF失败，重试{max_retries}次')
        return None


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text(text='初始化位置')
    patrol.init_robot_pose()
    patrol.speech_text(text='初始化完成')

    points = patrol.get_target_points()
    if not points:
        patrol.get_logger().error("无有效目标点，退出！")
        rclpy.shutdown()
        return

    while rclpy.ok():
        for pt in points:
            x, y, yaw = pt
            patrol.speech_text(text=f'准备前往目标点{x},{y}')
            ok = patrol.nav_to_pose(patrol.get_pose_by_xyyaw(x,y,yaw))
            if ok:
                patrol.speech_text(text=f"已到达目标点{x},{y},准备记录图像")
                patrol.record_image()
                patrol.speech_text(text=f"图像记录完成")

                # ========== 替换rclpy.sleep，兼容所有ROS2版本 ==========
                wait_s = patrol.get_parameter('patrol_interval').value
                start = patrol.get_clock().now()
                wait_dur = Duration(seconds=wait_s)
                while patrol.get_clock().now() - start < wait_dur:
                    rclpy.spin_once(patrol, timeout_sec=0.05)

    rclpy.shutdown()

if __name__ == '__main__':
    main()