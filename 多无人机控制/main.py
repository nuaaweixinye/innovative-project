import airsim
import time
import numpy as np
import asyncio
import byTrace 
import math
import os
import pandas as pd

def init_drone(client):   
    # 重置无人机状态
    client.reset()
    
    # 获取所有无人机的名称
    drone_names = client.listVehicles()
    if not drone_names:
        # 如果没有无人机，默认使用一个无人机
        drone_names = ["Drone0"]
        print("使用默认无人机名称: Drone0")
    
    # 为每个无人机设置合理的初始属性
    print("初始化无人机...")
    for name in drone_names:
        

        
        # 2. 设置动力学属性 - 通过 API 控制而非直接设置
        # 解锁电机
        client.enableApiControl(True, name)
        # 预启动电机
        client.armDisarm(True, name)
        
        # 3. 设置飞行控制参数（如果需要）
        # 这些通常在 settings.json 中配置
        
        # 4. 设置传感器参数（如果需要）
        # 例如，设置相机参数
        # client.simSetCameraOrientation("front_center", airsim.to_quaternion(0, 0, 0), name)
        
    return drone_names

def takeoffAll(client, drone_names):
    # 起飞 - 同时启动所有无人机的起飞任务
    print("所有无人机同时起飞...")
    takeoff_tasks = []
    for name in drone_names:
        # 只启动任务，不立即等待完成
        task = client.takeoffAsync(3, name)
        takeoff_tasks.append(task)
    
    # 等待所有无人机起飞完成
    for i, name in enumerate(drone_names):
        takeoff_tasks[i].join()
    
def landAll(client, drone_names):
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

def hoverAll(client, drone_names,duration):
    # 悬停 - 同时启动所有无人机的悬停任务
    print("所有无人机同时悬停...")
    hover_tasks = []
    for name in drone_names:
        task = client.hoverAsync(vehicle_name=name)
        hover_tasks.append(task)
    
    # 等待所有无人机悬停完成
    for i, name in enumerate(drone_names):
        hover_tasks[i].join()
    # 悬停时间
    time.sleep(duration)

# 控制所有无人机同时移动
def moveAll(client, drone_names, duration, vx, vy, vz):
    print(f"所有无人机同时移动... 速度: vx={vx}, vy={vy}, vz={vz}, 持续时间: {duration}秒")
    
    move_tasks = []
    
    # 为每个无人机创建移动任务
    for name in drone_names:
        try:
            # 强制启用API控制和解锁电机
            client.enableApiControl(True, name)
            client.armDisarm(True, name)
            
            # 发送移动命令
            task = client.moveByVelocityAsync(vx, vy, vz, duration, vehicle_name=name)
            move_tasks.append((name, task))
            
            print(f"已发送移动命令到无人机 {name}")
        except Exception as e:
            print(f"处理无人机 {name} 时出错: {str(e)}")
    
    # 等待移动完成
    time.sleep(duration + 0.5)  # 加0.5秒确保所有无人机完成移动
    
    print("移动操作完成")

#让所有无人机按轨迹飞行
async def fly_along_trajectory(client):
    """
    使所有无人机同时按照记录的轨迹飞行
    """
    print("开始使无人机按照轨迹飞行...")
    drone_names = client.listVehicles()#获取所有无人机的名称
    if not drone_names:
        print("没有找到无人机，无法执行轨迹飞行")
        return
    
    print(f"找到 {len(drone_names)} 个无人机: {', '.join(drone_names)}")

    #csv文件路径 - 直接使用实际的无人机名称来匹配文件
    data_dir = "drone_trace_data"
    
    # 从csv文件中读取轨迹数据
    trajectories = {}
    for drone_name in drone_names:
        file_path = os.path.join(data_dir, f"{drone_name}_trajectory.csv")
        try:
            # 读取轨迹数据，header=0表示第一行是标题行，会自动跳过
            df = pd.read_csv(file_path, header=0)
            
            # 检查列名是否正确
            if not all(col in df.columns for col in ['x', 'y', 'z']):
                # 如果列名不是小写的x,y,z，尝试转换列名或使用位置索引
                if all(col.upper() in [c.upper() for c in df.columns] for col in ['x', 'y', 'z']):
                    # 将列名转换为小写
                    df.columns = [col.lower() for col in df.columns]
                else:
                    # 假设列的顺序是timestamp, x, y, z
                    df.columns = ['timestamp', 'x', 'y', 'z']
            
            # 转换为 numpy 数组，使用小写的x,y,z列
            trajectory = df[['x', 'y', 'z']].values
            trajectories[drone_name] = trajectory
            print(f"成功加载 {drone_name} 的轨迹数据，共 {len(trajectory)} 个点")
        except Exception as e:
            print(f"读取 {drone_name} 轨迹数据失败: {str(e)}")

    # 所有无人机同时按轨迹飞行
    all_tasks = []
    
    print("所有无人机开始同时按轨迹飞行...")
    for drone_name, trajectory in trajectories.items():
        if len(trajectory) == 0:
            print(f"警告: {drone_name} 的轨迹数据为空，跳过该无人机")
            continue
            
        # 使用functools.partial或lambda来正确捕获变量
        async def fly_drone_trajectory(name, points):
            try:
                # 遍历轨迹点
                for _, point in enumerate(points):
                    x, y, z = point
                    
                    # 发送位置命令 - 使用join()替代await，因为返回的可能是自定义Future
                    task = client.moveToPositionAsync(x, y, z, 1, vehicle_name=name)
                    task.join()  # 等待这个移动操作完成
                print(f"{name} 完成轨迹飞行")
            except Exception as e:
                print(f"{name} 执行轨迹飞行时出错: {str(e)}")
        
        # 创建异步任务并添加到所有任务列表
        # 使用partial或默认参数来捕获当前的drone_name和trajectory
        task = asyncio.create_task(fly_drone_trajectory(drone_name, trajectory))
        all_tasks.append(task)
    
    # 等待所有无人机的轨迹飞行任务完成
    if all_tasks:
        await asyncio.gather(*all_tasks)
        print("所有无人机轨迹飞行任务已完成")
    else:
        print("没有可执行的轨迹飞行任务")





async def main(client):
    # 初始化客户端和无人机
    drone_names = init_drone(client)
    
    # 起飞所有无人机
    takeoffAll(client, drone_names)
    
    # 添加一个短暂的悬停，确保无人机稳定
    print("无人机起飞后悬停2秒...")
    time.sleep(2)
    
    # 初始化停止事件
    stop_events = {drone_name: asyncio.Event() for drone_name in drone_names}
    
    # 启动异步轨迹记录任务（先启动记录）
    trajectory_tasks = []
    for drone_name in drone_names:
        task = asyncio.create_task(
            byTrace.record_drone_trajectory_async(client, drone_name, 
                stop_event=stop_events[drone_name]#传入停止事件参数
            )
        )
        trajectory_tasks.append(task)
    
    # 等待一小段时间，确保轨迹记录已经开始
    await asyncio.sleep(0.5)
    
    # 执行移动操作（使用更大的速度值以确保可见效果）
    moveAll(client, drone_names, 5, 2, 0, -2)  # 尝试使用5m/s的速度，更容易观察到变化
    
    # 等待移动完成后，停止轨迹记录
    for drone_name in drone_names:
        stop_events[drone_name].set()
    
    # 等待所有轨迹记录任务完成
    trajectories = await asyncio.gather(*trajectory_tasks)
    
    # 降落所有无人机
    landAll(client, drone_names)

async def trace_main(client):
    # 显示所有无人机的轨迹
    byTrace.show_alldrone_trajectories_in_unreal(client)

    # 初始化客户端和无人机
    drone_names = init_drone(client)
    takeoffAll(client, drone_names)
    # 添加一个短暂的悬停，确保无人机稳定
    print("无人机起飞后悬停2秒...")
    time.sleep(2)

    #按轨迹飞行
    await fly_along_trajectory(client)
    # 降落所有无人机
    landAll(client, drone_names)
    print("所有无人机已降落")





if __name__ == "__main__":

    # 连接到 AirSim 模拟器
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # 重置无人机状态
    client.reset()

    mode_num=int(input("请输入模式：1.main 2.trace"))
    if mode_num==1:
        asyncio.run(main(client))
    elif mode_num==2:
        asyncio.run(trace_main(client))