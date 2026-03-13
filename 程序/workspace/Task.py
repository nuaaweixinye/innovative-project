# 任务类:描述、处理计算任务
'''
返回值说明:
None:资源不足，无法执行任务
cost,time:执行任务消耗的资源和时间

'''

from config import DRONE_MAX_RESOURCES


class Task:
    def __init__(self, name="",cost=None,time=0):

        self.name = name
        self.cost = cost
        self.time = time

#基础计算任务类:继承自Task，包含具体的计算逻辑
class SimpleTask(Task):
    def __init__(self,  name="", cost=0, time=0):
        super().__init__(name,cost,time)

         
    def execute(self,resources=DRONE_MAX_RESOURCES):
        if resources >= self.cost:
            return self.cost, self.time  # 返回计算结果
        else:
            return None  # 资源不足，返回None
    


#复杂计算任务类:继承自Task，由简单计算任务组合而成
class ComplexTask(Task):
    def __init__(self, name="", cost=None, time=0):
        super().__init__(name,cost,time)
        
    def add_subtask(self, subtask:Task):
        self.cost.append(subtask)  # 添加子任务到列表
    
    def remove_subtask(self, subtask:Task):
        self.cost.remove(subtask)  # 从列表中移除子任务
        

        
    def execute(self,resources=DRONE_MAX_RESOURCES):#从tasks里挑一个子任务执行，返回消耗的资源
        if not self.cost:
            return False  # 没有子任务可执行，返回False结果
        for task in self.cost:
            if isinstance(task, SimpleTask):
                res=task.execute(resources)  # 执行简单子任务
                if res is not None:  # 如果子任务执行成功
                    self.cost.remove(task)  # 从列表中移除已执行的子任务
                    return res  # 返回子任务结果
            else:
                res=task.execute(resources)  # 递归执行复杂子任务
                if res == False:  # 如果复杂子任务执行完毕
                    self.cost.remove(task)  # 从列表中移除已执行的复杂子任务
                    continue  # 继续执行下一个子任务
                elif res is not None:  # 如果复杂子任务执行成功
                    return res  # 返回复杂子任务结果
        return None  # 没有可执行的子任务，返回None结果和0资源消耗
    

class TaskManager:
    def __init__(self,  tasks=None):
        if tasks is None:
            self.tasks = [] # 默认参数
        else:
            self.tasks = tasks
    
    def is_empty(self):
        return len(self.tasks) == 0  # 判断任务列表是否为空
    
    def add_task(self, task):
        self.tasks.append(task)  # 添加任务到列表
    
    def execute(self,resources):#选一个任务执行，返回消耗的资源
        if not self.tasks:
            return 0,0
        for task in self.tasks:
            if isinstance(task, SimpleTask):
                res=task.execute(resources)  # 执行简单任务
                if res is not None:  # 如果任务执行成功
                    self.tasks.remove(task)  # 从列表中移除已执行的任务
                    return res  # 返回任务结果
            else:
                res=task.execute(resources)  # 递归执行复杂任务
                if res == False:  # 如果复杂任务执行完毕
                    self.tasks.remove(task)  # 从列表中移除已执行的复杂任务
                    continue  # 继续执行下一个任务
                elif res is not None:  # 如果复杂任务执行成功
                    return res  # 返回复杂任务结果
        return None  # 没有可执行的任务，返回None结果
    
    def create_simple_task(self, cost, time):
        print(f"创建简单任务: cost={cost}, time={time}")  
        
    def create_complex_task(self, subtasks):
        print(f"创建复杂任务:{subtasks}")
    
    def delete_task(self, name):
        self.tasks = [task for task in self.tasks if task.name != name]  # 删除指定任务

    def clear_tasks(self):
        self.tasks = []  # 清空所有任务
        
    def get_task(self, name):
        for task in self.tasks:
            if task.name == name:
                return task  # 返回指定任务
        return None  # 未找到任务返回None
    