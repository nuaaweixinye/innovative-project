# 控制器类，负责管理无人机的任务执行和资源分配
import Drone
from Drone import Drone
import config
import Task

class Controller:
    def __init__(self):
        self.drones = [Drone(id+1) for id in range(config.DRONE_NUM)]
        self.tasks = Task.TaskManager()  # 任务管理器，负责管理所有任务


    # 起飞所有无人机或指定无人机
    def takeoff(self, drone_id=None):
        if drone_id is None:  # 如果没有指定无人机id，则让所有无人机起飞
            takeoff_futures = []
            for drone in self.drones:
                future = drone.client.takeoffAsync(vehicle_name=f"Drone{drone.id}")
                takeoff_futures.append(future)
                print(f"Drone{drone.id} 正在起飞...")
            # 等待所有无人机起飞完成
            for future in takeoff_futures:
                future.join()
        else:  # 如果指定了无人机id，则只让指定的无人机起飞
            for drone in self.drones:
                if drone.id == drone_id:
                    future = drone.client.takeoffAsync(vehicle_name=f"Drone{drone.id}")
                    print(f"Drone{drone.id} 正在起飞...")
                    # 等待指定无人机起飞完成
                    future.join()
                    break

    # 降落所有无人机或指定无人机
    def land(self, drone_id=None):
        if drone_id is None:  # 如果没有指定无人机id，则让所有无人机降落
            land_futures = []
            for drone in self.drones:
                future = drone.client.landAsync(vehicle_name=f"Drone{drone.id}")
                land_futures.append(future)
                print(f"Drone{drone.id} 正在降落...")
            # 等待所有无人机降落完成
            for future in land_futures:
                future.join()
        else:  # 如果指定了无人机id，则只让指定的无人机降落
            for drone in self.drones:
                if drone.id == drone_id:
                    future = drone.client.landAsync(vehicle_name=f"Drone{drone.id}")
                    print(f"Drone{drone.id} 正在降落...")
                    # 等待指定无人机降落完成
                    future.join()
                    break    
    
    # 移动无人机指定速度和时间
    def move_by_velocity_for_duration(self, velocity, duration, drone_id=None):
        if drone_id is None:  # 如果没有指定无人机id，则让所有无人机按速度移动
            for drone in self.drones:
                drone.client.moveByVelocityAsync(
                    velocity[0], velocity[1], velocity[2], duration,
                    vehicle_name=f"Drone{drone.id}"
                )
                print(f"无人机{drone.id}以速度{velocity}移动{duration}秒")
        else:  # 如果指定了无人机id，则只让指定的无人机按速度移动
            for drone in self.drones:
                if drone.id == drone_id:
                    drone.client.moveByVelocityAsync(
                        velocity[0], velocity[1], velocity[2], duration,
                        vehicle_name=f"Drone{drone.id}"
                    )
                    print(f"无人机{drone.id}以速度{velocity}移动{duration}秒")
                    break
     
    # 移动无人机到指定位置
    def move_to_position(self, position, drone_id=None):
        if drone_id is None:  # 如果没有指定无人机id，则移动所有无人机
            move_futures = []
            for drone in self.drones:
                future = drone.client.moveToPositionAsync(position[0], position[1], position[2], 5, vehicle_name=f"Drone{drone.id}")
                move_futures.append(future)
            # 等待所有无人机移动完成
            for future in move_futures:
                future.join()
        else:  # 如果指定了无人机id，则只移动指定的无人机
            for drone in self.drones:
                if drone.id == drone_id:
                    future = drone.client.moveToPositionAsync(position[0], position[1], position[2], 5, vehicle_name=f"Drone{drone.id}")
                    # 等待指定无人机移动完成
                    future.join()
                    break
                
    # 分配计算任务给无人机
    def assign_task_to_drone(self, task, drone_id=None):
        if drone_id is None:  # 如果没有指定无人机id，则执行分配策略
            print("分配策略未实现")
        else:  # 如果指定了无人机id，则只分配给指定的无人机
            for drone in self.drones:
                if drone.id == drone_id:
                    drone.add_task(task)
                    print(f"任务分配给无人机{drone.id}")
                    break
    
    # 创建简单任务
    def create_simple_task(self, cost, time):
        from Task import SimpleTask
        task = SimpleTask(cost=cost, time=time)
        self.tasks.add_task(task)
        print(f"创建简单任务: cost={cost}, time={time}")
        return task
    
    # 创建复杂任务
    def create_complex_task(self, task_list):
        from Task import ComplexTask
        task = ComplexTask()
        task.cost = []  # 初始化cost为列表
        
        def process_task_list(tasks):
            processed_tasks = []
            for item in tasks:
                if isinstance(item, tuple) and len(item) == 2:
                    # 这是一个简单任务 (cost, time)
                    from Task import SimpleTask
                    processed_tasks.append(SimpleTask(cost=item[0], time=item[1]))
                elif isinstance(item, list):
                    # 这是一个嵌套的任务列表，递归处理
                    sub_complex_task = ComplexTask()
                    sub_complex_task.cost = []
                    sub_complex_task.cost.extend(process_task_list(item))
                    processed_tasks.append(sub_complex_task)
            return processed_tasks
        
        task.cost.extend(process_task_list(task_list))
        self.tasks.add_task(task)
        print(f"创建复杂任务: {task_list}")
        return task
    
    # 从全局任务列表中移除任务
    def remove_task_from_global_list(self, task_index):
        if 0 <= task_index < len(self.tasks.tasks):
            removed_task = self.tasks.tasks.pop(task_index)
            return removed_task
        else:
            print(f"错误: 任务索引 {task_index} 超出范围")
            return None