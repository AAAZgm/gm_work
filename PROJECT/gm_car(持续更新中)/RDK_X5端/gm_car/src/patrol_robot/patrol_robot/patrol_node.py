# 导入ROS2核心库
import rclpy

import os

# 导入位姿消息类型：PoseStamped(带时间戳位姿)、Pose(纯位姿)
from geometry_msgs.msg import PoseStamped, Pose

# 导入Nav2简易导航器，用于控制机器人移动
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 导入TF2坐标变换相关库：变换缓存、监听
from tf2_ros import TransformListener, Buffer

# 导入欧拉角 ↔ 四元数转换函数（修复拼写错误：tf2_transfomations → tf2_transformations）
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# 导入自定义服务接口：SpeachText（文字转语音的请求/响应格式）
from autopatrol_interface.srv import SpeachText

# 导入ROS2时间相关库
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 定义巡逻导航类，继承自BasicNavigator（基础导航器）
class PatrolNode(BasicNavigator):
    # ===================== 初始化函数 =====================
    def __init__(self, node_name='patrol_node', namespace=''):
        # 调用父类BasicNavigator的初始化
        super().__init__(node_name, namespace)
        
        # 创建TF2坐标变换缓存区
        self.buffer_ = Buffer()
        
        # 创建TF2监听器，绑定缓存区与当前节点，用于获取机器人实时位姿
        self.tf_buffer = TransformListener(self.buffer_, self)
        
        # 声明参数：机器人初始点 [x, y, yaw]
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        
        # 声明参数：导航目标点序列 [x1,y1,yaw1, x2,y2,yaw2, ...]
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.5])
        self.declare_parameter('image_save_path', '')

        # 获取参数：初始点
        self.initial_point_ = self.get_parameter('initial_point').value
        
        # 获取参数：目标点列表
        self.target_points_ = self.get_parameter('target_points').value
        self.image_save_path_ = self.get_parameter('image_save_path').value
        self.speach_client = self.create_client(SpeachText, 'speach_text')
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscriptions_image=self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        self.latest_image = msg

    def record_image(self):
        if self.latest_image is None:
            self.get_logger().warn('没有收到图像数据，无法保存')
            return
    
        try:
            # 获取当前位置
            pose = self.get_current_pose()
        
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        
            # 检查保存路径
            if not os.path.exists(self.image_save_path_):
                os.makedirs(self.image_save_path_)
                self.get_logger().info(f'创建目录: {self.image_save_path_}')
        
            # 生成文件名
            x = pose.translation.position.x
            y = pose.translation.position.y
            timestamp = self.get_clock().now().seconds_nanoseconds()[0]
            filename = f'{timestamp}_x{x:.3f}_y{y:.3f}.png'
            filepath = os.path.join(self.image_save_path_, filename)
        
            # 保存图像
            if cv2.imwrite(filepath, cv_image):
                self.get_logger().info(f'✓ 图像保存成功: {filepath}')
            else:
                self.get_logger().error(f'✗ 图像保存失败: {filepath}')
        except Exception as e:
            self.get_logger().error(f'保存图像时出错: {str(e)}')


    def speach_text(self, text):
        while not self.speach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务启动...')
        request = SpeachText.Request()
        #.Request()
        # SpeachText 里面的 “请求部分”
        # 一个服务一定分两部分：
        # Request：客户端 → 服务端（发过去的内容）
        # Response：服务端 → 客户端（返回来的内容）
        request.text = text
        future = self.speach_client.call_async(request)#异步发送调用
        rclpy.spin_until_future_complete(self, future)#等
        if future.result() is not None:
#raise = 主动抛出异常（报错
            result=future.result()
            if result.result:
                self.get_logger().info('语音播报成功')
            else:
                self.get_logger().info('语音播报失败')
        else:
            self.get_logger().info('服务调用失败')
    # ===================== 工具函数：x,y,yaw → PoseStamped =====================
    def get_pose_by_xyyaw(self, x, y, w):
        #不直接认识角度，只认识四元数
        # 创建带时间戳的位姿对象
        pose = PoseStamped()
        
        # 坐标系设置为地图坐标系
        pose.header.frame_id = 'map'
        
        # 设置X坐标
        pose.pose.position.x = x
        
        # 设置Y坐标
        pose.pose.position.y = y
        
        # 将欧拉角(yaw) → 四元数
        rotation_quat = quaternion_from_euler(0, 0, w)
        
        # 赋值四元数（朝向）
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]    
        pose.pose.orientation.w = rotation_quat[3]
        
        # 返回生成的目标位姿
        return pose

    # ===================== 初始化机器人初始位姿 =====================
    def init_robot_pose(self):
        # 获取参数服务器里的初始点
        initial_pose = self.get_parameter('initial_point').value
        
        # 把初始点转为PoseStamped并设置给导航系统
        self.setInitialPose(self.get_pose_by_xyyaw(initial_pose[0], initial_pose[1], initial_pose[2]))
        
        # 等待Nav2导航系统启动完成
        self.waitUntilNav2Active()

    # ===================== 解析目标点列表 =====================
    def get_target_points(self):
        # 新建空列表存储目标点
        points = []
        
        # 重新获取目标点参数
        self.target_points_ = self.get_parameter('target_points').value
        
        # 每3个值为一组：x, y, yaw
        for index in range(0, int(len(self.target_points_) / 3)):
            # 取出x
            x = self.target_points_[index * 3]
            # 取出y
            y = self.target_points_[index * 3 + 1]
            # 取出yaw
            yaw = self.target_points_[index * 3 + 2]
            
            # 加入目标点列表
            points.append([x, y, yaw])
            
            # 打印日志：成功获取第几个目标点
            self.get_logger().info(f'GET {index+1} TARGET {x},{y},{yaw}')
        
        # 返回所有目标点
        return points

    # ===================== 导航到目标点 =====================
    def nav_to_pose(self, target_pose):
        # 等待导航系统可用
        self.waitUntilNav2Active()
        
        # 发送导航目标点
        self.goToPose(target_pose)
        
        # 循环等待导航任务完成
        while not self.isTaskComplete():
            # 获取导航实时反馈
            feedback = self.getFeedback()
            
            if feedback:
                # 打印预计剩余时间
                self.get_logger().info(f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
            
            # 超时5秒自动取消任务（修复缩进）
            if feedback and Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                self.cancelTask()
                self.get_logger().info(f'导航超时，已取消任务')

        # 获取最终导航结果
        result = self.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
            return False
        else:
            self.get_logger().error('导航结果：返回状态无效')
            return False

    # ===================== 获取机器人当前位姿 =====================
    def get_current_pose(self):
        # ROS2运行中循环获取TF
        while rclpy.ok():
            try:
                # 查找map到base_footprint的坐标变换（机器人在地图中的位置）
                #args = [from_frame, to_frame, nearest time, timeout]
                current_pose = self.buffer_.lookup_transform(
                    'map', 
                    'base_footprint',
                    rclpy.time.Time(seconds=0),
                    rclpy.Duration(seconds=1)
                )
                
                # 获取变换数据
                transform = current_pose.transform
                
                # 四元数 → 欧拉角
                rotation_euler = euler_from_quaternion(
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                )
                
                # 打印当前位置与角度
                self.get_logger().info(f'当前位置:{transform.translation}, 欧拉角:{rotation_euler}')
                
                # 返回当前坐标变换
                return transform
            
            # 捕获异常（TF未准备好）
            except Exception as e:
                self.get_logger().error(str(e))
                self.get_logger().warn(f'等待TF数据，原因:{str(e)}')

# ===================== 主函数 =====================
def main():
    # 初始化ROS2
    rclpy.init()
    
    # 创建巡逻节点
    patrol = PatrolNode()
    patrol.speach_text(text='初始化位置')
    # 初始化机器人位姿
    patrol.init_robot_pose()
    patrol.speach_text(text='初始化完成')
    # 获取所有目标点
    points = patrol.get_target_points()
    while rclpy.ok():
        # 依次导航到每个目标点
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speach_text(text=f'准备前往目标点{x},{y}')
            if patrol.nav_to_pose(target_pose):
                patrol.speach_text(text=f"已到达目标点{x},{y},准备记录图像")
                patrol.record_image()
                patrol.speach_text(text=f"图像记录完成")
    
    # 关闭ROS2
    rclpy.shutdown()

# 运行主函数
if __name__ == '__main__':
    main()