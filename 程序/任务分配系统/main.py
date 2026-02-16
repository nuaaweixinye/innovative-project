import CLIENT
import airsim
import time
import Trace
import matplotlib
# 强制matplotlib使用Agg后端，避免启动时的问题
matplotlib.use('Agg')


client=airsim.MultirotorClient()



try:
    swarm=CLIENT.Swarm()
    swarm.takeoff()
    
    drones = [
        CLIENT.Vehicle(drone_name="Drone0"),
        CLIENT.Vehicle(drone_name="Drone1"),
        CLIENT.Vehicle(drone_name="Drone2")
    ]
    try:
       # 启动多无人机状态可视化
        visualizer_thread = CLIENT.start_multi_drone_visualization(drones)
        
        # 启动轨迹记录线程
        Trace.start_recording("Drone0", "trajectory/drone0_trajectory.csv")
        
        client.moveToPositionAsync(0, 0, -10, 1).join()  
        client.moveByVelocityAsync(0, 1, 0, 10).join() 
    finally: 
         # 确保轨迹记录线程停止
        Trace.stop_all_recording()
        print("轨迹记录线程已停止")
        # 确保无人机可视化线程停止
        CLIENT.stop_multi_drone_visualization()
        print("多无人机可视化已停止")
          
        
        
finally:
    # 确保无人机降落
    swarm.land()
