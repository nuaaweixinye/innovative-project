import Task
import config
import time
import airsim

class Drone:
    def __init__(self, id):
        self.id = id
        self.TaskManager = Task.TaskManager()
        self.resorces = config.DRONE_MAX_RESOURCES  # 初始化资源
        self.costs=[]  #(resource,start_time,duration)记录每次执行任务消耗的资源和时间 
        
        #连接到AirSim仿真环境
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        # 将数字ID转换为AirSim中使用的无人机名称格式
        self.vehicle_name = f"Drone{self.id}"
        print(f"drone{self.id}已连接")
        
        
    #更新资源信息
    def update_resources(self):
        current_time = time.time()
        for cost in self.costs:
            if current_time - cost[1] >= cost[2]:  # 如果任务执行时间已到
                self.resorces += cost[0]  # 恢复资源
                self.costs.remove(cost)  # 从记录中移除已完成的任务
                                                         
                                                            
    def add_task(self, task):
        self.TaskManager.add_task(task)
    


    def excute_tasks(self):
        self.update_resources()  # 更新资源信息
        res = self.TaskManager.execute(self.resorces)  # 执行任务并获取结果
        while res is not None:  # 继续执行任务直到没有可执行的任务
            if res is not None:
                self.resorces -= res[0]  # 更新资源
                self.costs.append((res[0], time.time(), res[1]))  # 记录消耗的资源和时间
            self.update_resources()  # 再次更新资源信息
            res = self.TaskManager.execute(self.resorces)  # 执行任务并获取结果
        
    
    #是否还有任务可执行
    def has_task(self):
        return not self.TaskManager.is_empty()
    
    def get_resources(self):
        self.update_resources()  # 更新资源信息
        return self.resorces  # 返回当前资源