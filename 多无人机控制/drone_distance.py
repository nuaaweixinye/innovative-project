import airsim
import numpy as np
import time
import os
import json

# 全局变量
drone_names = []
drone_positions = {}  # 存储GPS位置坐标（经纬度转换为本地坐标系）
drone_gps_positions = {}  # 存储原始GPS数据
drone_velocities = {}
MIN_DISTANCE = 2.0

# 配置选项
USE_CUSTOM_POSITIONS = False  # 禁用自定义位置设置，使用settings.json中的位置
CUSTOM_POSITIONS = {
    "Drone1": (0, 0, 0),
    "Drone2": (5, 0, 0),
    "Drone3": (-5, 0, 0)
}  # 与settings.json匹配的位置配置
POSITION_RETRIES = 3  # 设置位置时的重试次数
RETRY_DELAY = 0.5     # 重试间隔（秒）

# GPS相关配置
earth_radius = 6371000  # 地球半径（米）
reference_lat = None  # 参考点纬度
reference_lon = None  # 参考点经度
reference_alt = None  # 参考点海拔


def get_vector3r_values(vec):
    """
    获取Vector3r对象的值，兼容不同版本的AirSim
    
    参数:
        vec: Vector3r对象
    
    返回值:
        tuple: (x, y, z)坐标值
    """
    try:
        # 尝试新版本属性访问方式
        if hasattr(vec, 'x_val'):
            return vec.x_val, vec.y_val, vec.z_val
        # 尝试旧版本属性访问方式
        elif hasattr(vec, 'x'):
            return vec.x, vec.y, vec.z
        # 尝试索引访问方式（某些版本可能支持）
        else:
            return float(vec[0]), float(vec[1]), float(vec[2])
    except Exception as e:
        print(f"获取Vector3r值时出错: {e}")
        return 0.0, 0.0, 0.0


# 将GPS经纬度转换为本地坐标系的函数
def gps_to_local(lat, lon, alt):
    """
    将GPS经纬度转换为以第一个无人机位置为原点的本地坐标系
    
    参数:
        lat: 纬度
        lon: 经度
        alt: 海拔高度
    
    返回值:
        tuple: (x, y, z)本地坐标系坐标
    """
    global reference_lat, reference_lon, reference_alt
    
    # 如果是第一个无人机，设置为参考点
    if reference_lat is None:
        reference_lat = lat
        reference_lon = lon
        reference_alt = alt
        return 0.0, 0.0, 0.0
    
    # 转换为弧度
    lat1 = np.radians(reference_lat)
    lon1 = np.radians(reference_lon)
    lat2 = np.radians(lat)
    lon2 = np.radians(lon)
    
    # 计算距离（基于Haversine公式的简化版本，适用于短距离）
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1
    
    # 计算本地坐标系的x和y（东和北）
    x = delta_lon * earth_radius * np.cos(lat1)
    y = delta_lat * earth_radius
    z = alt - reference_alt  # 高度差
    
    return x, y, z


# 创建Vector3r对象函数保持不变
def create_vector3r(x, y, z):
    """
    创建Vector3r对象，兼容不同版本的AirSim
    
    参数:
        x, y, z: 坐标值
    
    返回值:
        Vector3r对象
    """
    try:
        vec = airsim.Vector3r()
        # 根据版本设置属性
        if hasattr(vec, 'x_val'):
            vec.x_val = x
            vec.y_val = y
            vec.z_val = z
        else:
            vec.x = x
            vec.y = y
            vec.z = z
        return vec
    except Exception as e:
        print(f"创建Vector3r对象时出错: {e}")
        return None


# API兼容性检查函数保持不变
def check_api_compatibility():
    """
    检查AirSim API兼容性
    """
    print("\n===== API兼容性检查 ====")
    try:
        # 测试Vector3r对象属性
        test_vec = airsim.Vector3r(1, 2, 3)
        x, y, z = get_vector3r_values(test_vec)
        print(f"Vector3r属性测试成功: ({x}, {y}, {z})")
        
        # 检查可用属性名称
        vec_attrs = [attr for attr in dir(test_vec) if not attr.startswith('__')]
        print(f"Vector3r可用属性: {vec_attrs}")
        
        return True
    except Exception as e:
        print(f"API兼容性检查失败: {e}")
        return False


# 修改init_info函数，只使用GPS数据
def init_info(client):
    """
    初始化无人机信息，包括名称和GPS位置
    
    参数:
        client: AirSim客户端连接
    
    返回值:
        bool: 初始化是否成功
    """
    global drone_names, drone_positions, drone_gps_positions, reference_lat, reference_lon, reference_alt
    
    # 重置参考点
    reference_lat = None
    reference_lon = None
    reference_alt = None
    
    # 获取所有无人机名称
    drone_names = client.listVehicles()
    if not drone_names:
        print("未检测到无人机")
        return False
    
    print(f"\n===== 无人机列表 ({len(drone_names)}) ====")
    
    # 添加延迟，让物理引擎有时间稳定
    print("等待物理引擎稳定...")
    time.sleep(3.0)
    
    # 获取每架无人机的GPS位置
    for drone_name in drone_names:
        # 初始化为0向量
        drone_velocities[drone_name] = np.array([0.0, 0.0, 0.0])
        
        try:
            # 启用API控制
            client.enableApiControl(True, drone_name)
            # 解锁电机
            client.armDisarm(True, drone_name)
            
            # 仅使用GPS数据作为位置源
            gps_data = client.getGpsData(vehicle_name=drone_name)
            lat = gps_data.gnss.geo_point.latitude
            lon = gps_data.gnss.geo_point.longitude
            alt = gps_data.gnss.geo_point.altitude
            
            # 存储原始GPS数据
            drone_gps_positions[drone_name] = np.array([lat, lon, alt])
            
            # 转换GPS数据到本地坐标系
            pos_x, pos_y, pos_z = gps_to_local(lat, lon, alt)
            
            # 记录本地坐标系位置信息
            drone_positions[drone_name] = np.array([pos_x, pos_y, pos_z])
            
            # 打印详细位置信息，包括GPS数据和本地坐标系数据
            print(f"无人机 {drone_name} 本地坐标: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f}) 米")
            print(f"  GPS数据: ({lat:.6f}, {lon:.6f}, {alt:.2f}) 米")
            
        except Exception as e:
            print(f"初始化无人机 {drone_name} 时出错: {e}")
            return False
    
    # 验证位置是否合理（避免所有无人机位置都相同的情况）
    all_positions = [pos.tolist() for pos in drone_positions.values()]
    if len(set(tuple(pos) for pos in all_positions)) == 1:
        print("警告: 检测到所有无人机位置相同，可能存在GPS数据获取问题")
    
    return True


# check_drone_distances函数保持不变
def check_drone_distances():
    """
    检查所有无人机之间的距离
    
    返回值:
        list: 包含距离小于安全距离的无人机对及其距离 [(name1, name2, distance), ...]
    """
    too_close_pairs = []
    
    if len(drone_names) < 2:
        return too_close_pairs  # 无人机数量少于2，无需检查
    
    # 检查每对无人机之间的距离
    for i in range(len(drone_names)):
        for j in range(i + 1, len(drone_names)):
            name1 = drone_names[i]
            name2 = drone_names[j]
            
            # 计算欧几里得距离
            pos1 = drone_positions[name1]
            pos2 = drone_positions[name2]
            distance = np.linalg.norm(pos1 - pos2)
            
            # 检查是否小于安全距离
            if distance < MIN_DISTANCE:
                too_close_pairs.append((name1, name2, distance))
                print(f"警告: 无人机 {name1} 和 {name2} 之间距离过近: {distance:.2f} 米")
    
    return too_close_pairs


# calculate_repulsive_forces函数保持不变
def calculate_repulsive_forces(too_close_pairs, k=1.5, max_accel=5.0):
    """
    计算无人机之间的排斥力
    
    参数:
        too_close_pairs: 距离过近的无人机对列表
        k: 排斥系数，控制排斥力大小
        max_accel: 最大加速度限制
    
    返回值:
        dict: 无人机名称到加速度向量的映射
    """
    accelerations = {}
    # 初始化每架无人机的加速度为0向量
    for name in drone_names:
        accelerations[name] = np.array([0.0, 0.0, 0.0])
    
    if not too_close_pairs:
        return accelerations
    
    print(f"检测到 {len(too_close_pairs)} 对无人机距离过近，计算排斥加速度...")
    
    for name1, name2, distance in too_close_pairs:
        # 获取位置信息
        pos1 = drone_positions[name1]
        pos2 = drone_positions[name2]
        
        # 计算距离向量
        distance_vector = pos1 - pos2
        
        # 避免距离为零导致除零错误
        if distance <= 0.1:
            print(f"警告: 无人机 {name1} 和 {name2} 位置几乎重合，添加微小扰动")
            # 添加随机微小扰动以打破对称性
            np.random.seed(hash(name1 + name2) % 1000)
            perturbation = np.random.normal(0, 0.5, 3)
            perturbation[2] *= 0.5  # 减小z轴方向的扰动
            distance_vector = perturbation
            distance = np.linalg.norm(distance_vector)
        
        # 计算单位方向向量
        direction = distance_vector / distance
        
        # 计算排斥力大小
        repulsion_force = k * (MIN_DISTANCE - distance) / MIN_DISTANCE
        
        # 距离越近，排斥力越强
        if distance < MIN_DISTANCE * 0.5:
            repulsion_force *= 2.0
        
        # 限制最大加速度
        acceleration_magnitude = min(max_accel, abs(repulsion_force))
        
        # 调整各轴的权重，鼓励在xy平面移动
        weighted_direction = np.copy(direction)
        weighted_direction[0] *= 2.0  # x轴权重增加
        weighted_direction[1] *= 2.0  # y轴权重增加
        weighted_direction[2] *= 0.5  # z轴权重减小
        # 重新归一化方向向量
        weighted_direction = weighted_direction / np.linalg.norm(weighted_direction)
        
        # 计算加速度向量
        acceleration = weighted_direction * acceleration_magnitude
        
        # 更新每架无人机的总加速度
        accelerations[name1] += acceleration  # name1受到远离name2的加速度
        accelerations[name2] -= acceleration  # name2受到远离name1的加速度
    
    return accelerations


# 修改apply_accelerations函数，使用GPS数据更新位置
def apply_accelerations(client, accelerations, time_step=0.1, max_velocity=3.0):
    """
    将计算出的加速度应用到无人机上，使用GPS数据更新位置
    
    参数:
        client: AirSim客户端连接
        accelerations: 无人机名称到加速度向量的映射
        time_step: 时间步长
        max_velocity: 最大允许速度
    
    返回值:
        bool: 操作是否成功
    """
    try:
        global drone_velocities, drone_positions, drone_gps_positions
        
        # 存储所有异步任务
        velocity_tasks = []
        
        for drone_name, accel in accelerations.items():
            try:
                # 获取当前速度
                current_vel = drone_velocities[drone_name]
                
                # 计算新速度 (v = v0 + a*t)
                new_vel_x = current_vel[0] + accel[0] * time_step
                new_vel_y = current_vel[1] + accel[1] * time_step
                new_vel_z = current_vel[2] + accel[2] * time_step
                
                # 计算速度大小
                vel_magnitude = np.sqrt(new_vel_x**2 + new_vel_y**2 + new_vel_z**2)
                
                # 如果速度超过最大值，按比例缩小
                if vel_magnitude > max_velocity:
                    scale_factor = max_velocity / vel_magnitude
                    new_vel_x *= scale_factor
                    new_vel_y *= scale_factor
                    new_vel_z *= scale_factor
                
                # 更新缓存中的速度
                drone_velocities[drone_name] = np.array([new_vel_x, new_vel_y, new_vel_z])
                
                # 发送速度指令
                task = client.moveByVelocityAsync(
                    new_vel_x, 
                    new_vel_y, 
                    new_vel_z, 
                    duration=time_step * 2,  # 设置持续时间
                    vehicle_name=drone_name
                )
                velocity_tasks.append(task)
                
            except Exception as e:
                print(f"调整无人机 {drone_name} 速度时出错: {e}")
        
        # 等待短暂时间，让指令生效
        time.sleep(time_step / 2)
        
        # 更新所有无人机的位置信息 - 使用GPS数据
        for drone_name in drone_names:
            try:
                # 只使用GPS数据更新位置
                gps_data = client.getGpsData(vehicle_name=drone_name)
                lat = gps_data.gnss.geo_point.latitude
                lon = gps_data.gnss.geo_point.longitude
                alt = gps_data.gnss.geo_point.altitude
                
                # 更新原始GPS数据
                drone_gps_positions[drone_name] = np.array([lat, lon, alt])
                
                # 转换GPS数据到本地坐标系
                pos_x, pos_y, pos_z = gps_to_local(lat, lon, alt)
                
                # 更新本地坐标系位置
                drone_positions[drone_name] = np.array([pos_x, pos_y, pos_z])
                
            except Exception as e:
                print(f"更新无人机 {drone_name} GPS位置时出错: {e}")
        
        return True
        
    except Exception as e:
        print(f"应用加速度时发生错误: {e}")
        return False


# maintain_distance_control_loop函数保持不变
def maintain_distance_control_loop(client, loop_duration=20, update_rate=10):
    """
    无人机保持距离的主控制循环
    
    参数:
        client: AirSim客户端连接
        loop_duration: 循环持续时间(秒)，0表示无限循环
        update_rate: 更新频率(Hz)
    """
    print(f"启动无人机保持距离控制，安全距离: {MIN_DISTANCE}米")
    
    time_step = 1.0 / update_rate
    start_time = time.time()
    loop_count = 0
    
    try:
        while True:
            # 检查是否达到循环持续时间
            if loop_duration > 0 and time.time() - start_time > loop_duration:
                print("保持距离控制循环已达到设定的持续时间")
                break
            
            loop_count += 1
            if loop_count % update_rate == 0:  # 每秒打印一次状态
                print(f"保持距离控制中... 已运行 {(time.time() - start_time):.1f}秒")
                # 打印当前所有无人机位置
                for name, pos in drone_positions.items():
                    print(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            
            # 检查无人机之间的距离
            too_close_pairs = check_drone_distances()
            
            # 计算排斥力
            accelerations = calculate_repulsive_forces(too_close_pairs)
            
            # 应用加速度调整位置
            if apply_accelerations(client, accelerations, time_step):
                # 控制循环频率
                elapsed = time.time() - start_time
                expected_next = start_time + loop_count * time_step
                sleep_time = max(0, expected_next - time.time())
                time.sleep(sleep_time)
            else:
                # 如果应用失败，等待更长时间再重试
                time.sleep(time_step * 2)
    
    except KeyboardInterrupt:
        print("用户中断保持距离控制循环")
    except Exception as e:
        print(f"保持距离控制循环发生错误: {e}")
    
    return True


# takeoff_and_maintain_distance函数保持不变
def takeoff_and_maintain_distance(client, altitude=-3, takeoff_time=3):
    """
    无人机起飞并保持距离
    
    参数:
        client: AirSim客户端连接
        altitude: 起飞高度(负值表示高于地面)
        takeoff_time: 起飞时间
    """
    # 初始化无人机信息
    if not init_info(client):
        return False
    
    print(f"\n所有无人机起飞到高度 {altitude} 米")
    
    # 同时起飞所有无人机
    takeoff_tasks = []
    for name in drone_names:
        task = client.takeoffAsync(takeoff_time, name)
        takeoff_tasks.append(task)
    
    # 等待所有无人机起飞完成
    for i, name in enumerate(drone_names):
        takeoff_tasks[i].join()
        print(f"无人机 {name} 起飞完成")
    
    # 起飞后悬停一段时间，确保稳定
    time.sleep(2)
    
    # 启动保持距离控制
    return maintain_distance_control_loop(client)


# 主程序部分保持不变
if __name__ == "__main__":
    try:
        # 连接到AirSim客户端
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("\n===== 系统信息 ====")
        # 检查API版本兼容性
        check_api_compatibility()
        
        # 打印settings.json配置信息（如果可用）
        try:
            settings_file = r"c:\Users\xk\Documents\AirSim\settings.json"
            if os.path.exists(settings_file):
                with open(settings_file, 'r') as f:
                    settings = json.load(f)
                print(f"\n加载的settings.json配置:")
                for drone, config in settings.get("Vehicles", {}).items():
                    print(f"  {drone}: 位置=({config.get('X', 0)}, {config.get('Y', 0)}, {config.get('Z', 0)})")
            else:
                print(f"未找到settings.json文件: {settings_file}")
        except Exception as e:
            print(f"无法加载settings.json: {e}")
        
        # 启动起飞并保持距离控制
        takeoff_and_maintain_distance(client)
        
        # 控制结束后，降落所有无人机
        print("\n开始降落所有无人机")
        land_tasks = []
        for name in drone_names:
            task = client.landAsync(5, name)
            land_tasks.append(task)
        
        # 等待所有无人机降落完成
        for i, name in enumerate(drone_names):
            land_tasks[i].join()
            print(f"无人机 {name} 降落完成")
            # 关闭电机和API控制
            client.armDisarm(False, name)
            client.enableApiControl(False, name)
        
        print("所有操作完成")
        
    except Exception as e:
        print(f"程序运行出错: {e}")
        # 确保在发生错误时也关闭所有无人机
        if 'client' in locals() and 'drone_names' in globals():
            try:
                for name in drone_names:
                    client.armDisarm(False, name)
                    client.enableApiControl(False, name)
            except:
                pass