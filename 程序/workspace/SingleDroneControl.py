#包括前后左右移动，升降，旋转等功能，速度手动设置
import airsim
import time

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
        
