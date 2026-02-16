from shapely.geometry import Point



class Config:

    def __init__(self):

        # self.source = Point(-1172.0, -1168) good for local minima
        self.source = Point(-30.0, -50.0)
        self.destination = Point(-10.0, 50.0)
        self.height = -18
        self.velocity = 1.5
        self.ltf_velocity = 3
        self.load_from_json = False #加载json文件，障碍物信息
        self.buffer_size = 8.0
        self.lidar_theta_resolution = 10
        self.lidar_scans_number = 1

config = Config()
