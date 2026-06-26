#!/usr/bin/env python3
"""
自主探索建图节点
基于边界探索（Frontier-based Exploration）算法实现未知区域的自动探索。

算法原理：
1. 边界（Frontier）定义：已知空闲区域（free=0）与未知区域（unknown=-1）之间的交界点。
   在栅格地图中，如果一个空闲格子的8邻域中有未知格子，则该点为边界点。
2. 边界聚类：将相邻的边界点通过 BFS 聚类成簇，每个簇代表一个潜在的探索目标区域。
3. 目标选择：根据距离（越近越好）和簇大小（越大越好，代表更多未知区域）综合评分，
   选择得分最优的边界簇中心作为下一个导航目标。
4. 循环探索：不断检测边界 → 选择目标 → 导航前往 → 到达后重复，直到没有剩余边界为止。

依赖：
- /map 话题：来自 SLAM（如 cartographer/slam_toolbox）的栅格地图
- /amcl_pose 话题：来自 AMCL 的机器人定位位姿
- navigate_to_pose Action：Nav2 的导航 Action Server
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer
import numpy as np
import math
import random
from collections import deque


class ExplorationNode(BasicNavigator):
    """
    自主探索建图主节点

    工作流程：
    1. 订阅 /map 和 /amcl_pose，获取地图和机器人位姿
    2. 定时器（1Hz）触发探索步骤：
       a. 检测地图中的边界点
       b. 对边界点进行 BFS 聚类
       c. 综合评分选择最佳探索目标
       d. 通过 Nav2 导航到目标位置
    3. 到达目标后，标记为已访问，继续下一次探索
    4. 当没有剩余边界时，探索完成
    """

    def __init__(self):
        super().__init__('exploration_node')
        
        # ReentrantCallbackGroup 使同一个回调可以并发
        # 防止 Action 的回调被定时器阻塞（否则导航目标发送后会卡住）
        self.callback_group = ReentrantCallbackGroup()
        # 把初始点转为PoseStamped并设置给导航系统

          # 补充缺失的 TF 初始化
        self.buffer_ = Buffer()
        self.tf_buffer = TransformListener(self.buffer_, self)
        # ===================== 参数声明 =====================
        # 这些参数可在 launch 文件或 YAML 配置文件中覆盖
        self.declare_parameters(
            namespace='',
            parameters=[
                ('exploration.frontier_threshold', 0.3),      # 边界最小距离（米）：目标离机器人至少多远才被考虑，避免原地打转
                ('exploration.min_frontier_size', 3),         # 最小边界簇大小（像素数）：小于此值的簇被视为噪声忽略
                ('exploration.goal_timeout', 40.0),           # 目标超时时间（秒）—— 当前未实现超时检测
                ('exploration.max_attempts', 3),              # 最大连续失败次数：超过则停止探索，防止在死角无限重试
                ('exploration.map_save_interval', 30.0),      # 地图保存间隔（秒）—— 当前未实现定时保存
            ]
        )
        
        # ===================== 订阅话题 =====================
        # 订阅 SLAM 输出的栅格地图
        # OccupancyGrid.data 中：0=空闲, 100=障碍, -1=未知
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10  # 适当增大队列以避免地图更新丢失（默认 ROS2 QoS）
        )
    
        # ===================== Nav2 Action Client =====================
        # 导航到目标点的 Action 客户端，与 Nav2 的 navigate_to_pose Action Server 通信
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',#名称
            callback_group=self.callback_group
        )
        
        # ===================== 状态变量 =====================
        self.current_map = None          # 最新接收的栅格地图数据（OccupancyGrid）
        self.current_pose = None         # 最新接收的机器人位姿（Pose，取自 PoseWithCovarianceStamped）
        self.frontier_goals = []         # 当前检测到的所有边界目标（列表，暂未使用）
        self.visited_goals = set()       # 已访问目标集合，key 为 "x,y" 字符串（精度0.01m），避免重复前往同一区域
        self.is_exploring = False        # 是否正在导航前往目标（导航中时暂停新的目标选择）
        self.current_goal = None         # 当前正在导航的目标边界信息字典
        self.failed_attempts = 0         # 连续导航失败次数，用于判断是否应该放弃探索
        self._initial_pose_set = False   # 初始位姿是否已成功设置
        self._log_counter = 0            # 日志计数器：控制进度日志打印频率

        # ===================== 定时器 =====================
        # 每2秒尝试设置初始位姿（直到AMCL成功接收为止）
        self._initial_pose_timer = self.create_timer(
            2.0,
            self._try_set_initial_pose,
            callback_group=self.callback_group
        )
        
        # 每秒触发一次探索步骤
        # 使用 ReentrantCallbackGroup 确保不会阻塞 Action 回调
        self.exploration_timer = self.create_timer(
            1.0,
            self.exploration_step,#主循环
            callback_group=self.callback_group
        )
        
        self.get_logger().info('✅ 自主探索节点已启动')
        self.get_logger().info('📌 等待地图和定位数据...')
    
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

    def _try_set_initial_pose(self):
        """
        定时尝试设置AMCL初始位姿
        
        只尝试发送一次初始位姿给 AMCL，成功后立即停止定时器。
        （pose_sub 已注释，无法检测 AMCL 是否输出位姿）
        """
        if self._initial_pose_set:
            return

        try:
            self.setInitialPose(self.get_pose_by_xyyaw(0.0, 0.0, 0.0))
            self._initial_pose_set = True
            self._initial_pose_timer.cancel()
            self.get_logger().info('✅ 已设置AMCL初始位姿 (0, 0, 0)')
        except Exception as e:
            self.get_logger().warn(f'设置初始位姿失败: {e}')

    def map_callback(self, msg: OccupancyGrid):#读取地图
        """
        地图回调：每次收到 /map 话题消息时更新本地地图缓存
        注意：地图会频繁更新（通常 1-5Hz），这里不做处理，仅在探索步骤中使用最新数据
        """
        self.current_map = msg#读取到地图数据
        
    def calculate_exploration_progress(self):
        """
        计算当前地图的探索完成度
        
        公式：已探索比例 = 已探索空闲区域 / (已探索空闲区域 + 未知区域)
        
        返回值范围 [0, 1]，0 表示完全未探索，1 表示完全探索完
        """
        if self.current_map is None:
            return 0.0

        data = np.array(self.current_map.data)
        free_count = np.sum(data == 0)       # 已探索的空闲区域
        unknown_count = np.sum(data == -1)     # 未知区域
        total = free_count + unknown_count

        if total == 0:
            return 1.0

        return free_count / total

    def exploration_step(self):
        """
        探索步骤主循环（定时器回调，1Hz）
        
        流程：
        1. 检查地图和位姿数据是否就绪
        2. 计算探索完成度，达到阈值则停止
        3. 如果正在导航中，跳过（等待导航完成）
        4. 检测当前地图中的边界点并聚类
        5. 如果没有边界 → 探索完成
        6. 选择最佳边界目标
        7. 通过 Nav2 导航到目标
        """
        # 检查地图数据是否就绪
        if self.current_map is None:
            self.get_logger().info('等待地图数据...')
            return
        
        # 通过 TF 获取机器人当前位姿（替代被注释的 AMCL pose_sub）
        try:
            transform = self.buffer_.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            # 从 TF 变换中构造 Pose 对象
            from geometry_msgs.msg import Pose
            p = Pose()
            p.position.x = transform.transform.translation.x
            p.position.y = transform.transform.translation.y
            p.position.z = transform.transform.translation.z
            p.orientation = transform.transform.rotation
            self.current_pose = p
        except Exception as e:
            self.get_logger().info(f'等待定位数据... (TF错误: {e})')
            return
        
        # 如果正在导航中，不发送新目标（等待当前导航完成）
        if self.is_exploring:
            return
        
        # 1. 检测边界点并聚类，返回边界中心列表
        frontiers = self.detect_frontiers()
        
        # 没有检测到任何边界 → 所有可达区域都已探索
        if not frontiers:
            self.get_logger().info('🎉 探索完成！没有更多未知区域')
            self.save_map()  # 探索完成，保存地图
            return
        
        # 2. 根据距离和大小评分，选择最佳探索目标
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier is None:
            # 所有目标都不满足条件（例如全部太近）
            self.get_logger().warn('无法选择有效的探索目标，尝试随机旋转寻找新边界')
            return
        
        # 3. 发送导航目标到 Nav2
        self.send_exploration_goal(best_frontier)

    def detect_frontiers(self):
        """
        检测边界点（已知空闲区域与未知区域的交界）
        
        算法：
        1. 遍历地图中每个像素（跳过边缘一圈避免越界）
        2. 如果该像素是空闲区域（值为0），检查其8邻域
        3. 如果8邻域中存在未知区域（值为-1），则该像素是边界点
        4. 对所有边界点进行 BFS 聚类，返回每个簇的世界坐标中心
        
        返回：边界簇列表，每个簇包含：
            - position: (world_x, world_y) 世界坐标
            - size: 簇中边界点的数量
            - center: (pixel_x, pixel_y) 像素坐标中心
        """
        if self.current_map is None:
            return []
        
        map_data = self.current_map
        width = map_data.info.width       # 地图像素宽度
        height = map_data.info.height     # 地图像素高度
        resolution = map_data.info.resolution  # 每个像素对应的实际尺寸（米/像素）
        origin_x = map_data.info.origin.position.x  # 地图左下角在世界坐标系中的 x 坐标
        origin_y = map_data.info.origin.position.y  # 地图左下角在世界坐标系中的 y 坐标
        
        # 将一维地图数据 reshape 为二维 numpy 数组（行=y，列=x）
        data = np.array(map_data.data).reshape((height, width))
        
        # 定义8邻域偏移（用于检测边界点）
        neighbors = [(-1,-1), (-1,0), (-1,1),
                     (0,-1),          (0,1),
                     (1,-1),  (1,0),  (1,1)]
        
        # 遍历地图，寻找边界点
        # 跳过边缘一圈（range 从 1 到 size-2），避免数组越界
        frontier_points = []#边界点列表
        for y in range(1, height-1):
            for x in range(1, width-1):
                # 当前点是空闲区域（值为 0 表示可通行）
                if data[y, x] == 0:
                    # 检查8邻域中是否有未知区域（值为 -1）
                    # 有 → 说明该空闲点紧邻未知区域，是"边界"
                    for dy, dx in neighbors:#读取xy邻域
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            if data[ny, nx] == -1:
                                frontier_points.append((x, y))
                                break  # 找到一个未知邻居即可，无需继续检查
        
        if not frontier_points:
            return []
        
        # 对边界点进行 BFS 聚类，提取每个簇的中心点作为候选探索目标
        frontiers = self.cluster_frontier_points(
            frontier_points,
            resolution,
            origin_x,
            origin_y
        )
        
        return frontiers

    def find_nearest_free_cell(self, px, py, data, width, height, search_radius=10):
        """
        在栅格地图中搜索离 (px, py) 最近的 free 格子

        参数：
            px, py: 像素坐标
            data: 2D numpy 数组 (height, width)
            width, height: 地图尺寸
            search_radius: 搜索半径（像素）

        返回：
            (nx, ny) 最近 free 格子的像素坐标，找不到则返回 None
        """
        # 如果当前已经是 free，直接返回
        if 0 <= py < height and 0 <= px < width and data[py, px] == 0:
            return (px, py)

        # BFS 在搜索半径内找最近的 free 格子
        visited_local = set()
        queue = deque([(px, py)])
        visited_local.add((px, py))

        while queue:
            cx, cy = queue.popleft()
            if abs(cx - px) > search_radius or abs(cy - py) > search_radius:
                continue
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= ny < height and 0 <= nx < width:
                        if (nx, ny) not in visited_local:
                            visited_local.add((nx, ny))
                            if data[ny, nx] == 0:
                                return (nx, ny)
                            queue.append((nx, ny))
        return None

    def cluster_frontier_points(self, points, resolution, origin_x, origin_y):
        """
        使用 BFS（广度优先搜索）对边界点进行聚类
        
        原理：
        - 相邻的边界点（8邻域内）属于同一个"边界簇"
        - 每个簇代表地图上一块连续的未知区域边界
        - 簇的中心作为候选探索目标（中心位置通常是最佳观测点）
        
        参数：
            points: 边界点列表 [(x, y), ...]
            resolution: 地图分辨率（米/像素）
            origin_x, origin_y: 地图原点在世界坐标系中的坐标
        
        返回：聚类后的边界簇列表
        """
        if not points:
            return []
        
        # 将边界点转为集合，支持 O(1) 查找
        points_set = set(points)
        
        # BFS 聚类
        visited = set()       # 已访问的边界点集合
        clusters = []         # 聚类结果列表
        
        for point in points:
            if point in visited:  # 点已访问过，跳过
                continue
            
            # 初始化 BFS 队列，从当前点开始扩展
            cluster = []            # 当前簇的所有点
            queue = deque([point])#放入双端队列
            visited.add(point)#已经访问
            
            # BFS 遍历：将所有相邻的边界点归入同一簇
            while queue:
                cx, cy = queue.popleft()#从最左边取出一个点
                cluster.append((cx, cy))
                
                # 检查8邻域
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue  # 跳过自身
                        nx, ny = cx + dx, cy + dy
                        # 如果邻域点也是边界点且未被访问过，加入当前簇
                        if (nx, ny) in points_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            queue.append((nx, ny))
            
            # 过滤掉太小的簇（可能是噪声或地图边缘伪影）
            min_size = self.get_parameter('exploration.min_frontier_size').value
            if len(cluster) >= min_size:
                # 计算簇中心的像素坐标
                center_x = sum(p[0] for p in cluster) / len(cluster)
                center_y = sum(p[1] for p in cluster) / len(cluster)

                # 将中心偏移到最近的可通行点（避免目标落在障碍物上）
                cx_int, cy_int = int(round(center_x)), int(round(center_y))
                map_data = self.current_map
                w = map_data.info.width
                h = map_data.info.height
                arr = np.array(map_data.data).reshape((h, w))

                nearest = self.find_nearest_free_cell(cx_int, cy_int, arr, w, h, search_radius=20)
                if nearest is not None:
                    center_x, center_y = nearest[0], nearest[1]

                # 像素坐标 → 世界坐标
                world_x = origin_x + center_x * resolution
                world_y = origin_y + center_y * resolution

                goal_key = f'{round(world_x * 2) / 2:.1f},{round(world_y * 2) / 2:.1f}'
                if goal_key not in self.visited_goals:
                    clusters.append({
                        'position': (world_x, world_y),
                        'size': len(cluster),
                        'center': (center_x, center_y)
                    })
        
        return clusters

    def select_best_frontier(self, frontiers):
        """
        从候选边界簇中选择最佳探索目标

        评分策略（归一化版本）：
        - 将距离和簇大小各自归一化到 [0,1]
        - score = w_dist * norm_dist - w_size * norm_size
        - 选择 score 最小的：距离近 + 簇大 → 最优

        额外约束：
        - 目标距离必须大于 frontier_threshold
        """
        if not frontiers or self.current_pose is None:
            return None

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # 过滤：距离阈值
        min_dist = self.get_parameter('exploration.frontier_threshold').value
        progress = self.calculate_exploration_progress()
        if progress < 0.2:
            min_dist = 0.1

        valid_frontiers = []
        for f in frontiers:
            fx, fy = f['position']
            distance = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            if distance < min_dist:
                continue
            valid_frontiers.append((f, distance))

        if not valid_frontiers:
            return None

        # 归一化评分
        distances = [d for _, d in valid_frontiers]
        sizes = [f['size'] for f, _ in valid_frontiers]
        max_dist = max(distances) if distances else 1.0
        max_size = max(sizes) if sizes else 1.0
        if max_dist == 0:
            max_dist = 1.0
        if max_size == 0:
            max_size = 1.0

        w_dist = 0.6   # 距离权重（越近越好）
        w_size = 0.4   # 簇大小权重（越大越好）

        best_frontier = None
        best_score = float('inf')

        for f, distance in valid_frontiers:
            norm_dist = distance / max_dist
            norm_size = f['size'] / max_size
            score = w_dist * norm_dist - w_size * norm_size

            if score < best_score:
                best_score = score
                best_frontier = f

        return best_frontier

    def send_exploration_goal(self, frontier):
        """
        发送探索目标到 Nav2 导航栈
        
        通过 NavigateToPose Action 向 Nav2 发送导航请求：
        1. 等待 Nav2 Action Server 就绪
        2. 构造 PoseStamped 目标消息（frame_id='map'）
        3. 异步发送目标，注册响应和结果回调
        
        参数：
            frontier: 边界簇字典，包含 'position' 键
        """
        # 等待 Nav2 的 navigate_to_pose Action Server 上线
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server 不可用，请确认 Nav2 已启动')
            return
        
        # 标记正在探索中，防止定时器重复发送目标
        self.is_exploring = True
        self.current_goal = frontier
        
        # 构造 NavigateToPose 的 Goal 消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'                              # 在 map 坐标系下指定目标
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()       # 当前时间戳
        goal_msg.pose.pose.position.x = frontier['position'][0]            # 目标 x（世界坐标）
        goal_msg.pose.pose.position.y = frontier['position'][1]            # 目标 y（世界坐标）
        # goal_msg.pose.pose.position.z = 0.0                                # z=0（2D导航）
        goal_msg.pose.pose.orientation.w = 1.0                             # 朝向由规划器自行决定
        
        self.get_logger().info(
            f"🎯 发送探索目标: ({frontier['position'][0]:.2f}, {frontier['position'][1]:.2f}), "
            f"簇大小: {frontier['size']}"
        )
        
        # 异步发送导航目标 send_goal_async () 自带的参数;同时注册了个监听器
        # feedback_callback：导航过程中的周期性反馈（距离、时间等）
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )## 1. 发送目标，同时绑定【实时反馈】
        # goal_response_callback：Nav2 接受/拒绝目标后的回调
        send_goal_future.add_done_callback(self.goal_response_callback)#进入结果
    def nav_feedback_callback(self, feedback_msg):
        """
        Nav2 导航反馈回调（导航过程中的周期性回调）
        
        feedback 中包含：
        - current_pose: 当前位姿
        - navigation_time: 导航已用时间
        - estimated_time_remaining: 预计剩余时间
        - distance_remaining: 剩余距离
        
        功能：导航进度日志、超时检测（navigation_time > goal_timeout 时取消目标）
        """
        feedback = feedback_msg.feedback
        # 导航超时检测：超过阈值则取消当前目标
        timeout = self.get_parameter('exploration.goal_timeout').value
        nav_time = feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9
        if nav_time > timeout:
            self.get_logger().warn(f'⏰ 导航超时({nav_time:.1f}s > {timeout:.1f}s)，取消当前目标')
            self.nav_client.cancel_goal_async()

    def goal_response_callback(self, future):
        """
        Nav2 目标接受/拒绝回调
        
        当 Nav2 收到目标后，会返回是否接受：
        - 接受 → 等待导航结果
        - 拒绝 → 释放探索锁，增加失败计数
        """
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                # 目标被拒绝（可能目标在障碍物上、不可达等）
                self.get_logger().error('❌ 目标被 Nav2 拒绝')
                self.is_exploring = False
                self.failed_attempts += 1
                return
            
            self.get_logger().info('✅ 目标已被 Nav2 接受，正在导航...')
            
            # 注册结果回调，等待导航完成（成功/失败/取消）
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'⚠️ goal_response_callback 异常: {e}')
            self.is_exploring = False
            self.failed_attempts += 1

    def get_result_callback(self, future):
        """
        导航结果回调
        
        Nav2 导航完成后触发：
        - STATUS_SUCCEEDED（成功到达）→ 标记目标为已访问，重置失败计数
        - 其他状态（失败/取消）→ 增加失败计数，超过上限则停止探索
        
        GoalStatus 常见值：
            STATUS_UNKNOWN = 0
            STATUS_ACCEPTED = 1
            STATUS_EXECUTING = 2
            STATUS_CANCELING = 3
            STATUS_SUCCEEDED = 4
            STATUS_CANCELED = 5
            STATUS_ABORTED = 6
        """
        try:
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('✅ 成功到达探索目标')
                # 将该目标标记为已访问，后续不再选择（精度 0.5m，与 cluster_frontier_points 一致）
                if self.current_goal:
                    gx, gy = self.current_goal['position']
                    self.visited_goals.add(
                        f'{round(gx * 2) / 2:.1f},{round(gy * 2) / 2:.1f}'
                    )
                self.failed_attempts = 0  # 成功到达，重置失败计数
            else:
                status_names = {4: '成功', 5: '取消', 6: '中止'}
                status_name = status_names.get(status, f'未知({status})')
                self.get_logger().warn(f'⚠️ 导航失败，状态: {status_name}')
                self.failed_attempts += 1
                
                # 连续失败次数超过上限 → 停止探索（可能地图中有不可达的死区）
                max_attempts = self.get_parameter('exploration.max_attempts').value
                if self.failed_attempts >= max_attempts:
                    self.get_logger().error(
                        f'❌ 连续失败 {self.failed_attempts} 次（上限 {max_attempts}），停止探索。'
                        '可能所有剩余区域都不可达。'
                    )
        except Exception as e:
            self.get_logger().error(f'⚠️ get_result_callback 异常: {e}')
            self.failed_attempts += 1
        
        # 释放探索锁，允许下一次探索步骤
        self.is_exploring = False
        self.current_goal = None


    def save_map(self):
        """
        保存地图（探索完成时调用）
        
        当前为空实现，可扩展为：
        - 调用 Nav2 的 map_saver_cli 服务保存 pgm/yaml
        - 调用 slam_toolbox 的 serialize_map 服务
        - 自定义保存路径
        
        示例调用方式（需要在节点中创建 Service Client）：
            # 使用 map_saver
            self.save_map_client.call_async(SaveMap.Request())
        """
        self.get_logger().info('💾 探索完成，建议保存地图')
        self.get_logger().info('   运行: ros2 run nav2_map_server map_saver_cli -f ~/map')


def main(args=None):
    """
    节点入口函数
    
    使用 MultiThreadedExecutor（多线程执行器）而非单线程 Spin，
    原因：Action 的异步回调（goal_response、get_result、feedback）
    需要 ReentrantCallbackGroup 配合多线程才能正常执行，
    否则定时器回调会阻塞 Action 回调导致导航请求卡住。
    """
    rclpy.init(args=args)
    
    node = ExplorationNode()

    node.get_logger().info('🚀 自主探索节点已启动，等待AMCL和地图就绪...')
    # 多线程执行器，支持并发回调
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()