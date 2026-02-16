import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
               color='green', s=100, label='Start', edgecolors='black')
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], 
               color='red', s=100, label='End', edgecolors='black')
    
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
    # 替换为你的CSV文件路径
    trajectory_file = "trajectory.csv"  # 若文件在其他目录，填写完整路径如"../data/trajectory.csv"
    plot_trajectory(trajectory_file)