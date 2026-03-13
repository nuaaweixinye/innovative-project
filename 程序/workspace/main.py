import tkinter as tk
from tkinter import ttk, messagebox
import config
from Drone import Drone
from Contoller import Controller
import ast
import threading
import sys
import time
import airsim

# 单独的实时控制窗口
class RealTimeControlWindow:
    def __init__(self, master, drone_controller):
        self.master = master
        self.drone_controller = drone_controller
        self.window = tk.Toplevel(master)
        self.window.title("实时控制")
        self.window.geometry("400x300")
        
        # 速度输入框
        speed_frame = ttk.Frame(self.window)
        speed_frame.pack(pady=10)
        
        tk.Label(speed_frame, text="速度:").pack(side=tk.LEFT)
        self.speed_entry = tk.Entry(speed_frame, width=10)
        self.speed_entry.insert(0, "5.0")
        self.speed_entry.pack(side=tk.LEFT, padx=5)
        
        set_speed_btn = tk.Button(speed_frame, text="设置速度", command=self.set_speed, bg="#2196F3", fg="white")
        set_speed_btn.pack(side=tk.LEFT, padx=5)
        
        # 控制说明
        control_info = tk.Text(self.window, height=8, width=50)
        control_info.pack(pady=10)
        control_info.insert(tk.END, "控制说明：\n")
        control_info.insert(tk.END, "W/S: 前进/后退\n")
        control_info.insert(tk.END, "A/D: 左移/右移\n")
        control_info.insert(tk.END, "↑/↓: 上升/下降\n")
        control_info.insert(tk.END, "←/→: 左转/右转\n")
        control_info.config(state=tk.DISABLED)
        
        # 无人机选择
        drone_frame = ttk.Frame(self.window)
        drone_frame.pack(pady=10)
        
        tk.Label(drone_frame, text="选择无人机:").pack(side=tk.LEFT)
        self.selected_drone = tk.StringVar(value="Drone1")
        drone_options = [f"Drone{i+1}" for i in range(config.DRONE_NUM)]
        drone_menu = tk.OptionMenu(drone_frame, self.selected_drone, *drone_options)
        drone_menu.pack(side=tk.LEFT, padx=5)
        
        # 起飞和降落按钮
        flight_frame = ttk.Frame(self.window)
        flight_frame.pack(pady=10)
        
        takeoff_btn = tk.Button(flight_frame, text="起飞", command=self.takeoff, bg="#4CAF50", fg="white")
        takeoff_btn.pack(side=tk.LEFT, padx=5)
        
        land_btn = tk.Button(flight_frame, text="降落", command=self.land, bg="#f44336", fg="white")
        land_btn.pack(side=tk.LEFT, padx=5)
        
        # 退出控制按钮
        quit_btn = tk.Button(self.window, text="退出控制", command=self.quit_control, bg="#f44336", fg="white")
        quit_btn.pack(pady=10)
        
        # 绑定键盘事件
        self.window.focus_set()
        self.window.bind("<KeyPress>", self.on_key_press)
        self.window.bind("<KeyRelease>", self.on_key_release)
        
        # 控制状态
        self.keys_pressed = set()
        self.control_active = True
        
        # 启动控制循环
        self.control_loop()
    
    def set_speed(self):
        try:
            speed = float(self.speed_entry.get())
            self.drone_controller.set_speed(speed)
        except ValueError:
            print("错误: 请输入有效的速度值")
    
    def takeoff(self):
        drone_id = int(self.selected_drone.get().replace("Drone", ""))
        self.drone_controller.takeoff(drone_id)
    
    def land(self):
        drone_id = int(self.selected_drone.get().replace("Drone", ""))
        self.drone_controller.land(drone_id)
    
    def on_key_press(self, event):
        key = event.keysym.lower()
        self.keys_pressed.add(key)
    
    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
    
    def control_loop(self):
        if not self.control_active:
            return
            
        # 更新无人机ID
        drone_id = int(self.selected_drone.get().replace("Drone", ""))
        self.drone_controller.update_drone_id(drone_id)
        
        # 处理按键
        if 'w' in self.keys_pressed:
            self.drone_controller.move_forward()
        elif 's' in self.keys_pressed:
            self.drone_controller.move_backward()
        
        if 'a' in self.keys_pressed:
            self.drone_controller.move_left()
        elif 'd' in self.keys_pressed:
            self.drone_controller.move_right()
        
        if 'up' in self.keys_pressed:
            self.drone_controller.move_up()
        elif 'down' in self.keys_pressed:
            self.drone_controller.move_down()
        
        if 'left' in self.keys_pressed:
            self.drone_controller.rotate_left()
        elif 'right' in self.keys_pressed:
            self.drone_controller.rotate_right()
        
        # 每隔一段时间检查一次
        if self.control_active:
            self.window.after(100, self.control_loop)
    
    def quit_control(self):
        self.control_active = False
        self.window.destroy()
        print("退出实时控制")

class SingleDroneController:
    def __init__(self, drone_id=1):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.update_drone_id(drone_id)
        self.speed = 5.0  # 默认速度
        print(f"单无人机控制器已初始化，无人机: {self.vehicle_name}")
        
        # 启用API控制并解锁
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)
    
    def update_drone_id(self, drone_id):
        """更新无人机ID"""
        self.vehicle_name = f"Drone{drone_id}"
        # 为新无人机启用API控制
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)
    
    def set_speed(self, speed):
        """设置控制速度"""
        self.speed = speed
        print(f"速度已设置为: {speed}")
    
    def takeoff(self, drone_id=None):
        """起飞无人机"""
        if drone_id:
            vehicle_name = f"Drone{drone_id}"
        else:
            vehicle_name = self.vehicle_name
        
        print(f"{vehicle_name} 起飞...")
        self.client.takeoffAsync(vehicle_name=vehicle_name)
    
    def land(self, drone_id=None):
        """降落无人机"""
        if drone_id:
            vehicle_name = f"Drone{drone_id}"
        else:
            vehicle_name = self.vehicle_name
        
        print(f"{vehicle_name} 降落...")
        self.client.landAsync(vehicle_name=vehicle_name)
    
    def move_forward(self):
        """向前移动"""
        print("向前移动")
        self.client.moveByVelocityBodyFrameAsync(self.speed, 0, 0, 0.1, vehicle_name=self.vehicle_name)
    
    def move_backward(self):
        """向后移动"""
        print("向后移动")
        self.client.moveByVelocityBodyFrameAsync(-self.speed, 0, 0, 0.1, vehicle_name=self.vehicle_name)
    
    def move_left(self):
        """向左移动"""
        print("向左移动")
        self.client.moveByVelocityBodyFrameAsync(0, -self.speed, 0, 0.1, vehicle_name=self.vehicle_name)
    
    def move_right(self):
        """向右移动"""
        print("向右移动")
        self.client.moveByVelocityBodyFrameAsync(0, self.speed, 0, 0.1, vehicle_name=self.vehicle_name)
    
    def move_up(self):
        """向上移动"""
        print("向上移动")
        # AirSim中Z轴向下为正，所以要传入负值才能上升
        self.client.moveByVelocityBodyFrameAsync(0, 0, -abs(self.speed), 0.1, vehicle_name=self.vehicle_name)
    
    def move_down(self):
        """向下移动"""
        print("向下移动")
        # AirSim中Z轴向下为正，所以要传入正值才能下降
        self.client.moveByVelocityBodyFrameAsync(0, 0, abs(self.speed), 0.1, vehicle_name=self.vehicle_name)
    
    def rotate_left(self):
        """向左旋转"""
        print("向左旋转")
        # 获取当前朝向并旋转
        yaw_rate = 30  # 度/秒
        duration = 0.5  # 秒
        self.client.rotateByYawRateAsync(yaw_rate, duration, vehicle_name=self.vehicle_name)
    
    def rotate_right(self):
        """向右旋转"""
        print("向右旋转")
        # 获取当前朝向并旋转
        yaw_rate = -30  # 度/秒
        duration = 0.5  # 秒
        self.client.rotateByYawRateAsync(yaw_rate, duration, vehicle_name=self.vehicle_name)

class DroneControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("无人机控制系统")
        self.root.geometry("600x500")
        
        # 创建控制器实例
        self.controller = Controller()
        self.single_drone_controller = SingleDroneController()
        
        # 创建主标签页控件
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建任务生成页面
        self.task_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.task_frame, text="任务生成")
        self.create_task_page()
        
        # 创建无人机控制页面
        self.control_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.control_frame, text="无人机控制")
        self.create_control_page()
        
        # 创建实时控制页面
        self.realtime_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.realtime_frame, text="实时控制")
        self.create_realtime_control_page()
    
    def create_task_page(self):
        # 任务类型选择
        task_type_frame = ttk.LabelFrame(self.task_frame, text="任务类型", padding=(10, 5))
        task_type_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.task_type_var = tk.StringVar(value="simple")
        tk.Radiobutton(task_type_frame, text="简单任务", variable=self.task_type_var, 
                      value="simple", command=self.toggle_task_fields).pack(side=tk.LEFT)
        tk.Radiobutton(task_type_frame, text="复杂任务", variable=self.task_type_var, 
                      value="complex", command=self.toggle_task_fields).pack(side=tk.LEFT, padx=(10, 0))
        
        # 任务输入区域
        input_frame = ttk.LabelFrame(self.task_frame, text="任务输入", padding=(10, 5))
        input_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 简单任务输入字段
        self.simple_frame = ttk.Frame(input_frame)
        self.simple_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(self.simple_frame, text="Cost:").grid(row=0, column=0, sticky=tk.W)
        self.cost_entry = tk.Entry(self.simple_frame, width=10)
        self.cost_entry.grid(row=0, column=1, padx=5)
        
        tk.Label(self.simple_frame, text="Time:").grid(row=0, column=2, sticky=tk.W)
        self.time_entry = tk.Entry(self.simple_frame, width=10)
        self.time_entry.grid(row=0, column=3, padx=5)
        
        # 复杂任务输入字段
        self.complex_frame = ttk.Frame(input_frame)
        
        tk.Label(self.complex_frame, text="任务列表 (例如: [(1,2), [(2,2), (30,8)]])").pack(anchor=tk.W)
        self.task_list_text = tk.Text(self.complex_frame, height=8, width=60)
        self.task_list_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 按钮框架
        button_frame = ttk.Frame(self.task_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        generate_btn = tk.Button(button_frame, text="生成任务", command=self.generate_task, bg="#4CAF50", fg="white")
        generate_btn.pack(side=tk.LEFT, padx=5)
        
        # 分配任务按钮
        assign_btn = tk.Button(button_frame, text="分配任务", command=self.assign_task, bg="#FF9800", fg="white")
        assign_btn.pack(side=tk.LEFT, padx=5)
        
        # 默认显示简单任务输入
        self.complex_frame.pack_forget()
        
    def toggle_task_fields(self):
        if self.task_type_var.get() == "simple":
            self.complex_frame.pack_forget()
            self.simple_frame.pack(fill=tk.X, pady=5)
        else:
            self.simple_frame.pack_forget()
            self.complex_frame.pack(fill=tk.BOTH, expand=True, pady=5)
    
    def generate_task(self):
        task_type = self.task_type_var.get()
        
        if task_type == "simple":
            # 获取简单任务输入
            try:
                cost = int(self.cost_entry.get())
                time = int(self.time_entry.get())
                
                if cost >= 100:
                    print("错误: Cost必须小于100")
                    return
                    
                # 调用Controller接口生成简单任务
                task = self.controller.create_simple_task(cost, time)
                print(f"简单任务已生成: Cost={cost}, Time={time}")
                
            except ValueError:
                print("错误: 请输入有效的数字")
        else:
            # 获取复杂任务输入
            task_list_str = self.task_list_text.get("1.0", tk.END).strip()
            if not task_list_str:
                print("错误: 请输入任务列表")
                return
                
            try:
                # 解析任务列表
                task_list = ast.literal_eval(task_list_str)
                
                # 验证cost值
                def validate_costs(tasks):
                    if isinstance(tasks, tuple) and len(tasks) == 2:
                        cost, time = tasks
                        if not isinstance(cost, int) or cost >= 100:
                            raise ValueError("Cost必须是小于100的整数")
                    elif isinstance(tasks, list):
                        for item in tasks:
                            validate_costs(item)
                
                validate_costs(task_list)
                
                # 调用Controller接口生成复杂任务
                task = self.controller.create_complex_task(task_list)
                print(f"复杂任务已生成: {task_list}")
                
            except (ValueError, SyntaxError):
                print("错误: 无效的任务列表格式")
    
    def assign_task(self):
        # 获取所有任务并显示供选择
        task_names = [f"Task_{i}" for i in range(len(self.controller.tasks.tasks))]  # 获取任务列表
        if not task_names:
            print("警告: 没有可分配的任务")
            return
        
        # 创建分配窗口
        assign_window = tk.Toplevel(self.root)
        assign_window.title("分配任务")
        assign_window.geometry("300x200")
        
        # 任务选择
        tk.Label(assign_window, text="选择任务:").pack(anchor=tk.W, padx=10, pady=5)
        task_var = tk.StringVar(value=task_names[0] if task_names else "")
        task_menu = tk.OptionMenu(assign_window, task_var, *task_names)
        task_menu.pack(fill=tk.X, padx=10, pady=5)
        
        # 无人机选择
        tk.Label(assign_window, text="选择无人机:").pack(anchor=tk.W, padx=10, pady=5)
        drone_var = tk.StringVar(value="所有无人机")
        drone_options = ["所有无人机"] + [f"无人机{i+1}" for i in range(config.DRONE_NUM)]
        drone_menu = tk.OptionMenu(assign_window, drone_var, *drone_options)
        drone_menu.pack(fill=tk.X, padx=10, pady=5)
        
        # 确认分配按钮
        def confirm_assignment():
            selected_task_index = task_names.index(task_var.get()) if task_var.get() in task_names else None
            selected_drone = drone_var.get()
            
            if selected_task_index is not None:
                task = self.controller.tasks.tasks[selected_task_index]
                drone_id = None
                if selected_drone != "所有无人机":
                    drone_id = int(selected_drone.replace("无人机", ""))
                
                # 调用Controller接口分配任务
                self.controller.assign_task_to_drone(task, drone_id)
                print(f"任务已分配给{selected_drone}")
                assign_window.destroy()
            else:
                print("错误: 请选择有效任务")
        
        tk.Button(assign_window, text="确认分配", command=confirm_assignment, bg="#4CAF50", fg="white").pack(pady=10)
    
    def create_control_page(self):
        # 无人机选择
        select_frame = ttk.LabelFrame(self.control_frame, text="选择无人机", padding=(10, 5))
        select_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.drone_selection = tk.StringVar(value="所有无人机")
        drone_options = ["所有无人机"] + [f"无人机{i+1}" for i in range(config.DRONE_NUM)]
        
        tk.OptionMenu(select_frame, self.drone_selection, *drone_options).pack(side=tk.LEFT)
        
        # 控制方式选择
        control_type_frame = ttk.LabelFrame(self.control_frame, text="控制方式", padding=(10, 5))
        control_type_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.control_type_var = tk.StringVar(value="to_position")
        tk.Radiobutton(control_type_frame, text="移动到指定位置", variable=self.control_type_var, 
                      value="to_position", command=self.toggle_control_fields).pack(side=tk.LEFT)
        tk.Radiobutton(control_type_frame, text="按速度移动指定时间", variable=self.control_type_var, 
                      value="by_speed", command=self.toggle_control_fields).pack(side=tk.LEFT, padx=(10, 0))
        
        # 控制参数输入区域
        param_frame = ttk.LabelFrame(self.control_frame, text="控制参数", padding=(10, 5))
        param_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 移动到指定位置参数
        self.to_position_frame = ttk.Frame(param_frame)
        self.to_position_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(self.to_position_frame, text="目标位置 X:").grid(row=0, column=0, sticky=tk.W)
        self.pos_x_entry = tk.Entry(self.to_position_frame, width=10)
        self.pos_x_entry.grid(row=0, column=1, padx=5)
        
        tk.Label(self.to_position_frame, text="Y:").grid(row=0, column=2, sticky=tk.W)
        self.pos_y_entry = tk.Entry(self.to_position_frame, width=10)
        self.pos_y_entry.grid(row=0, column=3, padx=5)
        
        tk.Label(self.to_position_frame, text="Z:").grid(row=0, column=4, sticky=tk.W)
        self.pos_z_entry = tk.Entry(self.to_position_frame, width=10)
        self.pos_z_entry.grid(row=0, column=5, padx=5)
        
        # 按速度移动参数
        self.by_speed_frame = ttk.Frame(param_frame)
        
        tk.Label(self.by_speed_frame, text="速度 X:").grid(row=0, column=0, sticky=tk.W)
        self.vel_x_entry = tk.Entry(self.by_speed_frame, width=10)
        self.vel_x_entry.grid(row=0, column=1, padx=5)
        
        tk.Label(self.by_speed_frame, text="Y:").grid(row=0, column=2, sticky=tk.W)
        self.vel_y_entry = tk.Entry(self.by_speed_frame, width=10)
        self.vel_y_entry.grid(row=0, column=3, padx=5)
        
        tk.Label(self.by_speed_frame, text="Z:").grid(row=0, column=4, sticky=tk.W)
        self.vel_z_entry = tk.Entry(self.by_speed_frame, width=10)
        self.vel_z_entry.grid(row=0, column=5, padx=5)
        
        tk.Label(self.by_speed_frame, text="持续时间:").grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        self.duration_entry = tk.Entry(self.by_speed_frame, width=10)
        self.duration_entry.grid(row=1, column=1, padx=5, pady=(5, 0), columnspan=2)
        
        # 控制按钮
        button_frame = ttk.Frame(self.control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        move_btn = tk.Button(button_frame, text="执行移动", command=self.execute_move, bg="#2196F3", fg="white")
        move_btn.pack(side=tk.LEFT, padx=5)
        
        # 默认显示移动到指定位置
        self.by_speed_frame.pack_forget()
        
    def toggle_control_fields(self):
        if self.control_type_var.get() == "to_position":
            self.by_speed_frame.pack_forget()
            self.to_position_frame.pack(fill=tk.X, pady=5)
        else:
            self.to_position_frame.pack_forget()
            self.by_speed_frame.pack(fill=tk.BOTH, expand=True, pady=5)
    
    def execute_move(self):
        # 获取所选无人机
        selection = self.drone_selection.get()
        drone_id = None
        if selection != "所有无人机":
            # 提取无人机ID (例如 "无人机1" -> 1)
            drone_id = int(selection.replace("无人机", ""))
        
        control_type = self.control_type_var.get()
        
        if control_type == "to_position":
            # 获取目标位置
            try:
                x = float(self.pos_x_entry.get())
                y = float(self.pos_y_entry.get())
                z = float(self.pos_z_entry.get())
                
                position = (x, y, z)
                
                # 调用Controller接口移动无人机到指定位置
                self.controller.move_to_position(position, drone_id)
                
                # 打印移动信息
                if drone_id is None:
                    print(f"移动所有无人机到位置: {position}")
                else:
                    print(f"移动无人机{drone_id}到位置: {position}")
                
            except ValueError:
                print("错误: 请输入有效的坐标值")
        else:
            # 获取速度和持续时间
            try:
                vel_x = float(self.vel_x_entry.get())
                vel_y = float(self.vel_y_entry.get())
                vel_z = float(self.vel_z_entry.get())
                duration = float(self.duration_entry.get())
                
                velocity = (vel_x, vel_y, vel_z)
                
                # 调用Controller接口按速度移动指定时间
                self.controller.move_by_velocity_for_duration(velocity, duration, drone_id)
                
                # 打印移动信息
                if drone_id is None:
                    print(f"以速度{velocity}移动所有无人机{duration}秒")
                else:
                    print(f"以速度{velocity}移动无人机{drone_id}{duration}秒")
                
            except ValueError:
                print("错误: 请输入有效的数值")
    
    def create_realtime_control_page(self):
        # 实时控制说明
        info_label = tk.Label(self.realtime_frame, text="点击下方按钮开启实时控制模式", font=("Arial", 12))
        info_label.pack(pady=20)
        
        # 开始实时控制按钮
        start_btn = tk.Button(self.realtime_frame, text="开始实时控制", 
                             command=self.start_realtime_control, 
                             bg="#9C27B0", fg="white", font=("Arial", 12))
        start_btn.pack(pady=10)
        
        # 控制说明
        control_desc = tk.Text(self.realtime_frame, height=10, width=60)
        control_desc.pack(pady=10)
        control_desc.insert(tk.END, "控制说明：\n")
        control_desc.insert(tk.END, "W/S: 前进/后退\n")
        control_desc.insert(tk.END, "A/D: 左移/右移\n")
        control_desc.insert(tk.END, "↑/↓: 上升/下降\n")
        control_desc.insert(tk.END, "←/→: 左转/右转\n")
        control_desc.insert(tk.END, "起飞前请先点击起飞按钮\n")
        control_desc.config(state=tk.DISABLED)
    
    def start_realtime_control(self):
        # 创建实时控制窗口
        RealTimeControlWindow(self.root, self.single_drone_controller)

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneControlGUI(root)
    root.mainloop()
    
    
    