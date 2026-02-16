import airsim
import numpy as np
import time
import asyncio
import os
from airsim.types import Pose, Vector3r, Quaternionr
# Simplified font settings using only the most common Windows fonts
import matplotlib.pyplot as plt
plt.rcParams["font.family"] = ["SimHei", "Microsoft YaHei"]

# 确保drone_trace_data目录存在
data_dir = "drone_trace_data"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)
    print(f"创建数据目录: {data_dir}")


# 异步记录无人机轨迹
async def record_drone_trajectory_async(client, drone_name, stop_event=None, sampling_rate=10):
    """
    异步记录无人机的飞行轨迹，直到收到停止信号，确保完整记录移动过程
    
    参数:
        client: AirSim客户端对象
        drone_name: 无人机名称
        stop_event: 可选的停止事件对象，用于控制何时停止记录
        sampling_rate: 采样频率（赫兹）- 增加到20Hz以获取更密集的数据点
    
    返回:
        trajectory: 包含位置(x,y,z)和时间戳的轨迹数据
    """
    print(f"开始异步记录无人机 {drone_name} 的轨迹")
    
    # 初始化轨迹数据列表
    trajectory = []
    
    # 计算采样间隔
    sample_interval = 1.0 / sampling_rate
    
    # 开始记录轨迹
    start_time = time.time()
    
    # 如果没有提供停止事件，则创建一个
    if stop_event is None:
        stop_event = asyncio.Event()
    
    # 获取初始位置并标记为起点
    try:
        initial_state = client.getMultirotorState(drone_name)
        initial_position = initial_state.kinematics_estimated.position
        initial_point = {
            'timestamp': 0.0,
            'x': initial_position.x_val,
            'y': initial_position.y_val,
            'z': initial_position.z_val,
            'is_moving': False,  # 标记是否处于移动状态
            'annotation': 'start'  # 标注为起点
        }
        trajectory.append(initial_point)
        print(f"记录起点位置: x={initial_position.x_val:.2f}, y={initial_position.y_val:.2f}, z={initial_position.z_val:.2f}")
    except Exception as e:
        print(f"获取无人机 {drone_name} 初始状态时出错: {e}")
    
    # 记录移动状态的变量
    is_moving = False
    prev_position = None
    movement_threshold = 0.05  # 移动阈值（米），超过此值认为开始移动
    
    # 持续记录直到收到停止信号
    try:
        while not stop_event.is_set():
            # 获取当前时间戳
            timestamp = time.time() - start_time
            
            # 获取无人机当前位置
            try:
                drone_state = client.getMultirotorState(drone_name)
                position = drone_state.kinematics_estimated.position
                
                # 判断是否开始移动
                current_position = np.array([position.x_val, position.y_val, position.z_val])
                
                if prev_position is not None:
                    # 计算与上一位置的距离
                    distance = np.linalg.norm(current_position - prev_position)
                    
                    # 如果距离超过阈值且之前未标记为移动状态，则标记为开始移动
                    if distance > movement_threshold and not is_moving:
                        is_moving = True
                        print(f"无人机 {drone_name} 开始移动")
                        # 在轨迹中添加移动开始的标记
                        movement_start_point = {
                            'timestamp': timestamp,
                            'x': position.x_val,
                            'y': position.y_val,
                            'z': position.z_val,
                            'is_moving': True,
                            'annotation': 'movement_start'
                        }
                        trajectory.append(movement_start_point)
                
                # 记录当前位置
                prev_position = current_position.copy()
                
                # 将位置和时间戳添加到轨迹数据中
                trajectory_point = {
                    'timestamp': timestamp,
                    'x': position.x_val,
                    'y': position.y_val,
                    'z': position.z_val,
                    'is_moving': is_moving
                }
                trajectory.append(trajectory_point)
            except Exception as e:
                print(f"获取无人机 {drone_name} 状态时出错: {e}")
                # 添加重试逻辑或继续下一次采样
                await asyncio.sleep(sample_interval)
                continue
            
            # 等待下一个采样点（使用异步sleep避免阻塞）
            await asyncio.sleep(sample_interval)
    except asyncio.CancelledError:
        print(f"无人机 {drone_name} 的轨迹记录被取消")
    finally:
        # 记录终点位置
        try:
            final_state = client.getMultirotorState(drone_name)
            final_position = final_state.kinematics_estimated.position
            final_point = {
                'timestamp': time.time() - start_time,
                'x': final_position.x_val,
                'y': final_position.y_val,
                'z': final_position.z_val,
                'is_moving': False,
                'annotation': 'end'  # 标注为终点
            }
            trajectory.append(final_point)
            print(f"记录终点位置: x={final_position.x_val:.2f}, y={final_position.y_val:.2f}, z={final_position.z_val:.2f}")
        except Exception as e:
            print(f"获取无人机 {drone_name} 最终状态时出错: {e}")
        
        # 计算总移动距离
        total_distance = 0.0
        if len(trajectory) > 1:
            for i in range(1, len(trajectory)):
                p1 = np.array([trajectory[i-1]['x'], trajectory[i-1]['y'], trajectory[i-1]['z']])
                p2 = np.array([trajectory[i]['x'], trajectory[i]['y'], trajectory[i]['z']])
                total_distance += np.linalg.norm(p2 - p1)
        
        print(f"轨迹记录完成，共采集 {len(trajectory)} 个点，总移动距离约为 {total_distance:.2f} 米")
        
        # 保存轨迹数据为CSV文件
        save_trajectory_to_csv(trajectory, f"{drone_name}_trajectory.csv")
        
        # 可视化轨迹，增强对移动过程的显示
        plot_enhanced_trajectory(trajectory, drone_name)
    
    return trajectory


# 修改保存轨迹数据的函数，确保文件保存到drone_trace_data目录
def save_trajectory_to_csv(trajectory, filename):
    """
    将轨迹数据保存到CSV文件
    """
    try:
        # 构建完整的文件路径
        filepath = os.path.join(data_dir, filename)
        
        # 提取x,y,z数据
        timestamps = [point['timestamp'] for point in trajectory]
        x_values = [point['x'] for point in trajectory]
        y_values = [point['y'] for point in trajectory]
        z_values = [point['z'] for point in trajectory]
        
        # 创建数据数组
        data = np.column_stack((timestamps, x_values, y_values, z_values))
        
        # 保存到CSV文件
        np.savetxt(filepath, data, delimiter=',', header='timestamp,x,y,z', comments='')
        print(f"轨迹数据已保存到 {filepath}")
    except Exception as e:
        print(f"保存轨迹数据时出错: {e}")


def show_trajectory_in_unreal(client, filepath, color_rgba=(1.0, 0.0, 0.0, 1.0),
        thickness=2.0, duration=-1, show_points=False, point_size=5.0):
    """
    在Unreal Engine中显示无人机轨迹
    
    参数:
        client: AirSim客户端对象
        filepath: 轨迹数据文件的完整路径
        color_rgba: 轨迹颜色，格式为(R,G,B,A)，范围0.0-1.0
        thickness: 轨迹线的粗细
        duration: 轨迹显示的持续时间（秒），-1表示一直显示
        show_points: 是否显示轨迹点
        point_size: 轨迹点的大小（仅当show_points为True时有效）
    """
    print(f"在Unreal中显示轨迹... 从文件 {filepath} 读取数据")
    
    try:
        # 检查文件是否存在
        if not os.path.exists(filepath):
            print(f"错误: 文件 {filepath} 不存在")
            return
        
        # 从CSV文件读取轨迹数据
        trajectory = []
        data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
        
        # 检查数据是否为空
        if data.size == 0:
            print("错误: 轨迹数据为空")
            return
        
        # 确保数据是二维数组
        if len(data.shape) == 1:
            data = np.array([data])
        
        # 直接从numpy数组创建Vector3r对象列表
        points = [Vector3r(row[1], row[2], row[3]) for row in data if len(row) >= 4]
        
        if not points:
            print("错误: 读取的轨迹数据为空")
            return
        
        # 在Unreal中绘制轨迹线
        client.simPlotLineStrip(
            points,
            color_rgba=color_rgba,
            thickness=thickness,
            duration=duration,
            is_persistent=True
        )
        
        # 如果需要显示轨迹点
        if show_points:
            client.simPlotPoints(
                points,
                color_rgba=color_rgba,
                size=point_size,
                duration=duration,
                is_persistent=True
            )
            
        print(f"轨迹已在Unreal中显示，共{len(points)}个点")
    except Exception as e:
        print(f"在Unreal中显示轨迹时出错: {e}")


def clear_trajectories_in_unreal(client):
    pass

# 增强版轨迹可视化函数，确保图像保存到drone_trace_data目录
def plot_enhanced_trajectory(trajectory, drone_name):
    """
    增强版轨迹可视化，突出显示移动过程
    """
    try:
        # 提取x,y,z数据和相关信息
        x_values = [point['x'] for point in trajectory]
        y_values = [point['y'] for point in trajectory]
        z_values = [point['z'] for point in trajectory]
        timestamps = [point['timestamp'] for point in trajectory]
        
        # 查找起点、移动开始点和终点
        start_idx = None
        move_start_idx = None
        end_idx = None
        
        for i, point in enumerate(trajectory):
            if 'annotation' in point:
                if point['annotation'] == 'start':
                    start_idx = i
                elif point['annotation'] == 'movement_start':
                    move_start_idx = i
                elif point['annotation'] == 'end':
                    end_idx = i
        
        # 创建3D图形
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制完整轨迹
        ax.plot(x_values, y_values, z_values, 'b-', linewidth=1, alpha=0.7, label='完整轨迹')
        
        # 如果检测到移动开始点，则突出显示移动部分
        if move_start_idx is not None and end_idx is not None:
            # 突出显示移动部分
            ax.plot(
                x_values[move_start_idx:end_idx+1], 
                y_values[move_start_idx:end_idx+1], 
                z_values[move_start_idx:end_idx+1], 
                'r-', linewidth=2.5, label='移动过程'
            )
        
        # 标记起点和终点
        if start_idx is not None:
            ax.scatter(x_values[start_idx], y_values[start_idx], z_values[start_idx], 
                      c='g', marker='o', s=150, label='起点')
        
        if end_idx is not None:
            ax.scatter(x_values[end_idx], y_values[end_idx], z_values[end_idx], 
                      c='r', marker='x', s=150, label='终点')
        
        # 如果检测到移动开始点，也标记出来
        if move_start_idx is not None:
            ax.scatter(x_values[move_start_idx], y_values[move_start_idx], z_values[move_start_idx], 
                      c='y', marker='^', s=120, label='移动开始')
        
        # 添加时间标签（可选）
        # 如果轨迹不是特别长，可以添加几个关键时间点的标签
        if len(trajectory) <= 200:
            for i in range(0, len(trajectory), max(1, len(trajectory)//10)):
                ax.text(x_values[i], y_values[i], z_values[i], 
                       f"t={timestamps[i]:.1f}s", fontsize=8)
        
        # 设置坐标轴标签
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_zlabel('Z (米)')
        
        # 设置标题，包含关键信息
        ax.set_title(f'无人机 {drone_name} 飞行轨迹（{len(trajectory)}个采样点）')
        
        # 添加图例
        ax.legend()
        
        # 添加网格线以提高可读性
        ax.grid(True, alpha=0.3)
        
        # 调整视角以更好地展示轨迹
        ax.view_init(elev=30, azim=45)  # 调整仰角和方位角
        
        # 构建完整的文件路径并保存图形
        filepath_enhanced = os.path.join(data_dir, f"{drone_name}_trajectory_enhanced.png")
        plt.savefig(filepath_enhanced)
        plt.close()
        print(f"增强版轨迹图已保存到 {filepath_enhanced}")
        
        # 同时也保存2D俯视图，更直观地显示水平移动
        fig2 = plt.figure(figsize=(10, 8))
        ax2 = fig2.add_subplot(111)
        
        # 绘制水平轨迹（俯视图）
        ax2.plot(x_values, y_values, 'b-', linewidth=1, alpha=0.7)
        
        # 如果检测到移动开始点，则突出显示移动部分
        if move_start_idx is not None and end_idx is not None:
            ax2.plot(
                x_values[move_start_idx:end_idx+1], 
                y_values[move_start_idx:end_idx+1], 
                'r-', linewidth=2.5
            )
        
        # 标记起点和终点
        if start_idx is not None:
            ax2.scatter(x_values[start_idx], y_values[start_idx], c='g', marker='o', s=150)
        
        if end_idx is not None:
            ax2.scatter(x_values[end_idx], y_values[end_idx], c='r', marker='x', s=150)
        
        # 设置坐标轴标签和标题
        ax2.set_xlabel('X (米)')
        ax2.set_ylabel('Y (米)')
        ax2.set_title(f'无人机 {drone_name} 水平轨迹（俯视图）')
        
        # 添加网格线
        ax2.grid(True, alpha=0.3)
        
        # 构建完整的文件路径并保存俯视图
        filepath_top = os.path.join(data_dir, f"{drone_name}_trajectory_top_view.png")
        plt.savefig(filepath_top)
        plt.close()
        print(f"水平轨迹图已保存到 {filepath_top}")
        
    except Exception as e:
        print(f"绘制增强版轨迹图时出错: {e}")


def show_alldrone_trajectories_in_unreal(client):
    """
    在Unreal Engine中显示所有无人机的轨迹
    
    参数:
        client: AirSim客户端对象
    """
    print("开始显示所有无人机轨迹...")
    
    # 定义不同无人机的颜色（红、绿、蓝）
    colors = [
        (1.0, 0.0, 0.0, 1.0),  # 红色
        (0.0, 1.0, 0.0, 1.0),  # 绿色
        (0.0, 0.0, 1.0, 1.0),  # 蓝色
        (1.0, 1.0, 0.0, 1.0),  # 黄色
        (1.0, 0.0, 1.0, 1.0),  # 紫色
        (0.0, 1.0, 1.0, 1.0)   # 青色
    ]
    
    # 检查drone_trace_data目录是否存在
    if not os.path.exists(data_dir):
        print(f"错误: 轨迹数据目录 {data_dir} 不存在")
        return
    
    # 获取目录中的所有轨迹CSV文件
    trajectory_files = [f for f in os.listdir(data_dir) if f.endswith('_trajectory.csv')]
    
    if not trajectory_files:
        print(f"错误: 在 {data_dir} 目录中没有找到轨迹CSV文件")
        return
    
    # 为每个轨迹文件调用show_trajectory_in_unreal函数
    for i, filename in enumerate(trajectory_files):
        # 构建完整的文件路径
        filepath = os.path.join(data_dir, filename)
        
        # 选择颜色（如果颜色数量不够，循环使用）
        color = colors[i % len(colors)]
        
        print(f"显示无人机轨迹，文件: {filename}，颜色: {color[:3]}")
        
        # 调用show_trajectory_in_unreal函数显示轨迹
        show_trajectory_in_unreal(client, filepath, color_rgba=color)
    
    print(f"所有 {len(trajectory_files)} 个无人机轨迹已在Unreal中显示")


