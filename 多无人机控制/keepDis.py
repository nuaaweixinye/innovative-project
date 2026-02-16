import airsim
import time
import numpy as np
import asyncio

#初始化参数
INIT_POSITIONS = {
    "Drone1": airsim.Vector3r(0, 0, 0),
    "Drone2": airsim.Vector3r(-5, -5, 0),
    "Drone3": airsim.Vector3r(5, 5, 0),}

drone_names=[]


def takeoffAll(client):

    # 起飞 - 同时启动所有无人机的起飞任务
    print("所有无人机同时起飞...")
    takeoff_tasks = []
    for name in drone_names:
        # 打开电机
        client.enableApiControl(True,name)
        client.armDisarm(True, name)
        # 只启动任务，不立即等待完成
        task = client.takeoffAsync(3, name)
        takeoff_tasks.append(task)
    
    # 等待所有无人机起飞完成
    for i, name in enumerate(drone_names):
        takeoff_tasks[i].join()

def landAll(client):
    # 降落 - 同时启动所有无人机的降落任务
    print("所有无人机同时降落...")
    land_tasks = []
    for name in drone_names:
        task = client.landAsync(5, name)
        land_tasks.append(task)
    
    # 等待所有无人机降落完成
    for i, name in enumerate(drone_names):
        land_tasks[i].join()
        
        # 关闭电机
        client.armDisarm(False, name)
        client.enableApiControl(False, name)

def GetPosition(client):
    positions={}
    
    # 获取无人机的位置
    for name in drone_names:
        pose = client.simGetVehiclePose(name)
        position = pose.position
        print(f"{name}位置:{position}")
        positions[name]=position
 
    return positions

def reset_and_set_position(client):
    
    global drone_names, INIT_POSITIONS
      
    print("设置无人机位置...")
    for name in drone_names:
        
        position=INIT_POSITIONS.get(name)
        if position is not None:
            client.simSetVehiclePose(airsim.Pose(position,
                airsim.Quaternionr(0, 0, 0, 1)), True, name)
        



# 计算排斥加速度函数
def calculate_repulsion_acceleration(client, target_distance, repulsion_strength):
    """
    计算每个无人机的排斥加速度，以保持给定的目标距离。
    :param client: AirSim 客户端
    :param drone_names: 无人机名称列表
    :param target_distance: 目标保持距离
    :param repulsion_strength: 排斥力强度
    :return: 包含每个无人机加速度的字典
    """
    accelerations = {name: np.array([0.0, 0.0, 0.0]) for name in drone_names}  # 确保初始加速度是 float 类型
    positions = {name: np.array([client.simGetVehiclePose(name).position.x_val, 
                                client.simGetVehiclePose(name).position.y_val, 
                                client.simGetVehiclePose(name).position.z_val]) for name in drone_names}
    
    for i, name1 in enumerate(drone_names):
        for j, name2 in enumerate(drone_names):
            if i < j:  # 避免重复计算和自我排斥
                pos1 = positions[name1]
                pos2 = positions[name2]
                distance = np.linalg.norm(pos1 - pos2)
                if distance < target_distance:
                    direction = (pos1 - pos2) / distance
                    repulsion = repulsion_strength * (1 - distance / target_distance)  # 简单的线性排斥力
                    accelerations[name1] += repulsion * direction
                    accelerations[name2] -= repulsion * direction  # 排斥力对另一个无人机的作用方向相反
    
    return accelerations

# 无人机运动控制函数
async def move_drones(client, accelerations, maxSpeed=10, duration=0.1):
    """
    根据计算出的加速度让无人机移动。
    :param client: AirSim 客户端
    :param drone_names: 无人机名称列表
    :param accelerations: 每个无人机的加速度
    :param speed: 速度
    :param duration: 持续时间
    """
    for name in drone_names:
        # 获取当前状态
        state = client.getMultirotorState(vehicle_name=name)
        # 获取当前速度
        current_velocity = np.array([state.kinematics_estimated.linear_velocity.x_val, 
                                     state.kinematics_estimated.linear_velocity.y_val, 
                                     state.kinematics_estimated.linear_velocity.z_val])
        # 计算新的速度
        new_velocity = current_velocity + accelerations[name] * duration
        # 限制速度的大小
        if np.linalg.norm(new_velocity) > maxSpeed:
            new_velocity = new_velocity / np.linalg.norm(new_velocity) * maxSpeed  # 保持方向，调整大小
        elif np.linalg.norm(new_velocity) < 0.05:  # 速度太小，可能是因为排斥力太小，不再移动
            new_velocity = np.array([0.0, 0.0, 0.0])
            print(f"{name}速度太小，不再移动")
        
        # 发送速度命令
        client.moveByVelocityAsync(new_velocity[0], new_velocity[1], new_velocity[2], duration, vehicle_name=name).join()
        
        # 等待一段时间，确保无人机移动
        await asyncio.sleep(duration)

# 主函数，控制无人机保持距离
async def main(client, target_distance, repulsion_strength, maxSpeed, duration):
    try:
        while True:
            # 计算排斥加速度
            accelerations = calculate_repulsion_acceleration(client,  target_distance, repulsion_strength)
            
            # 根据加速度让无人机移动
            await move_drones(client, accelerations, maxSpeed, duration)
            
            # 等待一段时间后再次计算
            await asyncio.sleep(duration/2)
    except KeyboardInterrupt:
        print("手动停止程序...")
        # 降落所有无人机
        landAll(client, drone_names)
        print("所有无人机已降落")



# 连接到 AirSim 模拟器并初始化无人机
if __name__ == "__main__":
    
    """
    初始化
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    drone_names=client.listVehicles()
    if not drone_names:
        drone_names = ["Drone0"]
        print("使用默认无人机名称: Drone0")
    
    reset_and_set_position(client)
    time.sleep(1)

    GetPosition(client)
    
    # 起飞所有无人机
    takeoffAll(client)
    
    # 添加一个短暂的悬停，确保无人机稳定
    print("无人机起飞后悬停1.2秒...")
    time.sleep(1.2)
    
    """
    运行保持距离
    """
    
    # 设置保持距离参数
    target_distance = 10  # 目标距离
    repulsion_strength = 100  # 排斥力强度
    maxSpeed = 5  # 最大速度
    duration = 0.5  # 持续时间

    
    # 运行保持距离的主函数
    asyncio.run(main(client, target_distance,
        repulsion_strength, maxSpeed, duration))

    
