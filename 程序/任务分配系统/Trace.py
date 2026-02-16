import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import airsim
import time
import threading


# 全局变量，用于控制轨迹记录线程
_stop_recording = False
_trajectory_data = {}

def _record_trajectory(drone_name, csv_file):
    """
    记录无人机轨迹到CSV文件（在后台线程中运行）
    Args:
        drone_name (str): 无人机名称
        csv_file (str): 输出的CSV文件名
    """
    global _stop_recording, _trajectory_data
    
    # 初始化当前无人机的数据列表
    _trajectory_data[drone_name] = []
    client = airsim.MultirotorClient()
    
    print(f"开始记录无人机 {drone_name} 的轨迹...")
    
    try:
        while not _stop_recording:
            # 获取当前位置
            position = client.getMultirotorState(vehicle_name=drone_name).kinematics_estimated.position
            _trajectory_data[drone_name].append([position.x_val, position.y_val, position.z_val])
            
            # 等待一小段时间，避免采样过于频繁
            time.sleep(0.5)  # 每0.5秒采样一次
    except Exception as e:
        print(f"记录轨迹时发生错误: {e}")
    finally:
        # 确保数据被保存
        if drone_name in _trajectory_data and _trajectory_data[drone_name]:
            df = pd.DataFrame(_trajectory_data[drone_name], columns=['x', 'y', 'z'])
            df.to_csv(csv_file, index=False)
            print(f"无人机 {drone_name} 的轨迹已保存到 {csv_file}")

def start_recording(drone_name, csv_file):
    """
    启动轨迹记录线程
    """
    thread = threading.Thread(target=_record_trajectory, args=(drone_name, csv_file))
    thread.daemon = True  # 设置为守护线程，主程序结束时自动终止
    thread.start()
    return thread

def stop_all_recording():
    """
    停止所有轨迹记录
    """
    global _stop_recording
    _stop_recording = True
    # 给线程一点时间来保存数据
    time.sleep(0.5)  



def plot_trajectory(csv_file):
    # 读取CSV文件
    df = pd.read_csv(csv_file)
    
    # 提取坐标数据（注意AirSim中Z轴向下，绘图时取反转为向上）
    x = df['x']
    y = df['y']
    z = -df['z']  # 转换Z轴方向为常规视觉习惯
    
    # 创建3D图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制轨迹线
    ax.plot(x, y, z, label='Trajectory', linewidth=2, color='blue')
    
    # 标记起点（绿色）和终点（红色）
    ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], 
               c='green', s=100, label='Start', edgecolors='black')
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], 
               c='red', s=100, label='End', edgecolors='black')
    
    # 设置坐标轴标签和标题
    ax.set_xlabel('X Position (m)', fontsize=10)
    ax.set_ylabel('Y Position (m)', fontsize=10)
    ax.set_zlabel('Z Position (m)', fontsize=10)  # 已转换为向上
    ax.set_title('3D Trajectory Visualization', fontsize=12, pad=20)
    
    # 添加图例
    ax.legend()
    
    # 调整视角（可根据需要修改角度）
    ax.view_init(elev=30, azim=45)  # 仰角30度，方位角45度
    
    # 显示网格
    ax.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.show()
    
    
    
if __name__ == "__main__":
    plot_trajectory("trajectory/drone0_trajectory.csv")