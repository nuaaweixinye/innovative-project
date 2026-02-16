import time
import math  # 添加math模块导入
import random

from shapely.geometry import Point

from DroneTypes import Position
from MyDroneClient import MyDroneClient
from PathPlanner import PathPlanner
from TangentBug import TangentBug
from Config  import config
from utils import get_parallelogram_missing_point
from TrajectoryLogger import TrajectoryLogger


class TangentBugAgent:

    def __init__(self):
        self._client = MyDroneClient()
        self._tangent_bug = TangentBug(config.destination)
        # 初始化轨迹记录器
        self.trajectory_logger = TrajectoryLogger()
        
        # 设置避障参数
        self.danger_threshold = 1.5  # 降低危险阈值，提前发现障碍物
        self.warning_threshold = 3.0  # 降低警告阈值
        self.turn_distance = 3.5  # 增加转向距离
        self.turn_angle = 3 * math.pi / 4  # 135度转向角
        self.escape_velocity = config.velocity * 1.8  # 进一步提高躲避速度
        
        # 脱困机制参数
        self.max_stuck_steps = 5  # 连续无法有效移动的最大步数
        self.stuck_counter = 0  # 当前连续无法移动的步数计数
        self.last_position = None  # 上一步的位置
        self.position_history = []  # 位置历史记录
        self.max_history_length = 10  # 位置历史最大长度
        self.movement_threshold = 0.5  # 判断有效移动的距离阈值
        
        # 高度调整参数
        self.min_height = config.height - 10  # 最低飞行高度
        self.max_height = config.height + 20  # 最高飞行高度
        self.vertical_escape_distance = 5  # 垂直方向避障的距离
        
        # 初始化随机种子
        random.seed(time.time())

    def connect_and_spawn(self):
        self._client.reset()
        print("Connecting.....")
        self._client.connect()
        time.sleep(2)
        self._client.setAtPosition(config.source.x, config.source.y , config.height)
        time.sleep(2)
        print(self._client.isConnected())
        time.sleep(2)
        # 连接成功后启动轨迹记录器
        self.trajectory_logger.start()

    def reached_goal_2D(self, curr_pos: Position, goal: Position):
        diff_x = curr_pos.x_m - goal.x_m
        diff_y = curr_pos.y_m - goal.y_m
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        if dist < 5.0:
            return True
        return False

    def point_reached_goal_2D(self, curr_pos: Point, goal: Point):
        diff_x = curr_pos.x - goal.x
        diff_y = curr_pos.y - goal.y
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        if dist < 5.0:
            return True
        return False
    
    def calculate_optimal_turn_direction(self, full_lidar_scan, curr_pos, next_step):
        """计算最优转向方向，选择障碍物最少的方向"""
        
        # 获取当前移动方向
        original_angle = math.atan2(next_step.y - curr_pos.y, next_step.x - curr_pos.x)
        
        # 将激光雷达数据分成多个扇区
        sectors = 12  # 分为12个扇区，提高方向识别精度
        sector_size = len(full_lidar_scan) // sectors
        sector_distances = []
        
        # 计算每个扇区的平均距离
        for i in range(sectors):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size
            sector_data = full_lidar_scan[start_idx:end_idx]
            valid_distances = [d for d in sector_data if not math.isinf(d)]
            
            if valid_distances:
                avg_distance = sum(valid_distances) / len(valid_distances)
            else:
                avg_distance = float('inf')
            
            sector_distances.append(avg_distance)
        
        # 找到障碍物最少（平均距离最大）的扇区
        best_sector = sector_distances.index(max(sector_distances))
        
        # 计算对应扇区的角度
        sector_angle = (best_sector * 360 / sectors) * math.pi / 180
        
        # 如果最佳扇区在当前方向的后方，选择第二佳的扇区
        if abs(sector_angle - original_angle) > math.pi * 0.75:
            # 找到所有可能的次优解
            sorted_sectors = sorted(enumerate(sector_distances), key=lambda x: x[1], reverse=True)
            
            # 寻找不在后方的最佳扇区
            for idx, (sector, distance) in enumerate(sorted_sectors):
                if idx == 0:  # 跳过已经检查过的最佳扇区
                    continue
                
                candidate_angle = (sector * 360 / sectors) * math.pi / 180
                if abs(candidate_angle - original_angle) <= math.pi * 0.75:
                    best_sector = sector
                    sector_angle = candidate_angle
                    break
        
        print(f"选择最优转向方向: 扇区 {best_sector+1}/{sectors}, 角度: {sector_angle*180/math.pi:.1f}度")
        return sector_angle
    
    def is_stuck(self, current_position):
        """检测无人机是否卡住"""
        # 检查是否有历史位置记录
        if not self.last_position:
            self.last_position = current_position
            return False
        
        # 计算与上一位置的距离
        distance_moved = math.sqrt(
            (current_position.x - self.last_position.x) ** 2 + 
            (current_position.y - self.last_position.y) ** 2
        )
        
        # 更新位置历史
        self.position_history.append(current_position)
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
        
        # 检查是否在原地踏步
        if distance_moved < self.movement_threshold:
            self.stuck_counter += 1
            print(f"检测到移动受限: 移动距离={distance_moved:.2f}, 连续受限步数={self.stuck_counter}")
        else:
            self.stuck_counter = 0  # 重置卡住计数器
        
        # 更新上一位置
        self.last_position = current_position
        
        # 判断是否卡住
        return self.stuck_counter >= self.max_stuck_steps
    
    def escape_maneuver(self, current_position, current_height):
        """执行脱困机动"""
        print("执行紧急脱困机动！")
        
        # 1. 首先尝试垂直方向避障（上升或下降）
        vertical_direction = 1 if random.random() > 0.5 else -1  # 随机选择上升或下降
        new_height = current_height + vertical_direction * self.vertical_escape_distance
        
        # 确保高度在安全范围内
        new_height = max(self.min_height, min(self.max_height, new_height))
        
        print(f"尝试垂直避障: 从高度{current_height}调整到{new_height}")
        self._client.flyToPosition(current_position.x, current_position.y, new_height, self.escape_velocity)
        time.sleep(1.5)  # 给予足够时间完成高度调整
        
        # 2. 随机选择一个水平方向快速逃离
        escape_angle = random.uniform(0, 2 * math.pi)  # 随机选择一个方向
        escape_distance = self.turn_distance * 2  # 加大逃离距离
        
        escape_x = current_position.x + escape_distance * math.cos(escape_angle)
        escape_y = current_position.y + escape_distance * math.sin(escape_angle)
        
        print(f"执行水平方向逃离: 角度={escape_angle*180/math.pi:.1f}度, 距离={escape_distance}")
        self._client.flyToPosition(escape_x, escape_y, new_height, self.escape_velocity)
        time.sleep(1.5)  # 给予足够时间远离障碍物
        
        # 3. 重置卡住计数器
        self.stuck_counter = 0
        
        return new_height

    def fly_to_destination(self):
        
        curr_pos = config.source
        next_step = curr_pos
        current_height = config.height
        self._client.flyToPosition(curr_pos.x+2, curr_pos.y+2, current_height, 0.01)
        try:
            while not self.point_reached_goal_2D(next_step, config.destination):
                # 获取当前位置并记录轨迹
                pose = self._client.getPose()
                self.trajectory_logger.log_position(pose.pos.x_m, pose.pos.y_m, pose.pos.z_m)
                
                # 创建当前位置点对象
                current_point = Point(pose.pos.x_m, pose.pos.y_m)
                
                # 检查是否卡住
                if self.is_stuck(current_point):
                    current_height = self.escape_maneuver(current_point, current_height)
                    # 重新获取位置和传感器数据
                    pose = self._client.getPose()
                
                full_lidar_scan = self._client.full_lidar_scan()
                curr_pose = self._client.getPose()
                next_step = self._tangent_bug.step(curr_pose, full_lidar_scan)
                
                # 检查next_step是否有效
                if next_step is None or next_step == -1:
                    print("无法找到有效路径，目标不可达")
                    # 触发脱困机动
                    current_height = self.escape_maneuver(current_point, current_height)
                    # 重新计算路径
                    next_step = self._tangent_bug.step(curr_pose, full_lidar_scan)
                    if next_step is None or next_step == -1:
                        break
                
                # 获取当前位置坐标
                curr_x, curr_y = pose.pos.x_m, pose.pos.y_m
                
                # 过滤出有效的激光雷达距离数据
                valid_distances = [d for d in full_lidar_scan if not math.isinf(d)]
                
                if valid_distances:
                    min_obstacle_distance = min(valid_distances)
                    
                    # 多级避障策略
                    if min_obstacle_distance < self.danger_threshold:
                        # 极度危险：紧急停止并大角度转向
                        print(f"极度危险！障碍物距离过近({min_obstacle_distance:.2f})，立即停止并紧急转向")
                        
                        # 紧急停止
                        self._client.flyToPosition(curr_x, curr_y, current_height, 0)
                        time.sleep(0.7)  # 延长停止时间以确保安全
                        
                        # 计算最优转向方向
                        optimal_angle = self.calculate_optimal_turn_direction(full_lidar_scan, curr_pos, next_step)
                        
                        # 加大转向距离
                        escape_distance = self.turn_distance * 2  # 增加100%的转向距离
                        
                        # 计算紧急避让位置
                        escape_x = curr_x + escape_distance * math.cos(optimal_angle)
                        escape_y = curr_y + escape_distance * math.sin(optimal_angle)
                        
                        # 快速逃离
                        self._client.flyToPosition(escape_x, escape_y, current_height, self.escape_velocity)
                        time.sleep(1.5)  # 给予足够时间逃离危险区域
                    elif min_obstacle_distance < self.warning_threshold:
                        # 警告：提前调整方向
                        print(f"警告：检测到前方有障碍物({min_obstacle_distance:.2f})，提前调整方向")
                        
                        # 计算最优转向方向
                        optimal_angle = self.calculate_optimal_turn_direction(full_lidar_scan, curr_pos, next_step)
                        
                        # 计算调整后的新路径点，结合原始目标方向和最优逃离方向
                        original_angle = math.atan2(next_step.y - curr_y, next_step.x - curr_x)
                        adjusted_angle = (original_angle + optimal_angle) / 2
                        adjusted_x = curr_x + self.turn_distance * math.cos(adjusted_angle)
                        adjusted_y = curr_y + self.turn_distance * math.sin(adjusted_angle)
                        
                        # 使用调整后的路径点
                        self._client.flyToPosition(adjusted_x, adjusted_y, current_height, config.velocity)
                    else:
                        # 安全距离：正常按照计算的路径移动
                        self._client.flyToPosition(next_step.x, next_step.y, current_height, config.velocity)
                else:
                    # 无障碍物：正常飞行
                    self._client.flyToPosition(next_step.x, next_step.y, current_height, config.velocity)
                
                time.sleep(0.5)
                print(f"current position: ({next_step.x}, {next_step.y}), height: {current_height}")
            
            # 添加到达目的地提示
            if self.point_reached_goal_2D(next_step, config.destination):
                print("到达目的地")
        finally:
            # 确保在结束时停止轨迹记录器
            self.trajectory_logger.stop()

    @property
    def client(self):
        return self._client