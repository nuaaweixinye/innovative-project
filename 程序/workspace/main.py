import tkinter as tk
from tkinter import ttk, messagebox
import config
from Drone import Drone
from Contoller import Controller
import ast

class DroneControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("无人机控制系统")
        self.root.geometry("600x500")
        
        # 创建控制器实例
        self.controller = Controller()
        
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
                    messagebox.showerror("错误", "Cost必须小于100")
                    return
                    
                # 调用Controller接口生成简单任务
                task = self.controller.create_simple_task(cost, time)
                messagebox.showinfo("成功", f"简单任务已生成: Cost={cost}, Time={time}")
                
            except ValueError:
                messagebox.showerror("错误", "请输入有效的数字")
        else:
            # 获取复杂任务输入
            task_list_str = self.task_list_text.get("1.0", tk.END).strip()
            if not task_list_str:
                messagebox.showerror("错误", "请输入任务列表")
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
                messagebox.showinfo("成功", f"复杂任务已生成: {task_list}")
                
            except (ValueError, SyntaxError):
                messagebox.showerror("错误", "无效的任务列表格式")
    
    def assign_task(self):
        # 获取所有任务并显示供选择
        task_names = [f"Task_{i}" for i in range(len(self.controller.tasks.tasks))]  # 获取任务列表
        if not task_names:
            messagebox.showwarning("警告", "没有可分配的任务")
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
                messagebox.showinfo("成功", f"任务已分配给{selected_drone}")
                assign_window.destroy()
            else:
                messagebox.showerror("错误", "请选择有效任务")
        
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
                    
                messagebox.showinfo("移动指令", f"已发送移动指令到{'所有无人机' if drone_id is None else f'无人机{drone_id}'}")
                
            except ValueError:
                messagebox.showerror("错误", "请输入有效的坐标值")
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
                    
                messagebox.showinfo("移动指令", f"已发送速度移动指令到{'所有无人机' if drone_id is None else f'无人机{drone_id}'}")
                
            except ValueError:
                messagebox.showerror("错误", "请输入有效的数值")

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneControlGUI(root)
    root.mainloop()