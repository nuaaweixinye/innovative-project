import multiprocessing
import time
import datetime
import os

class TrajectoryLogger:
    def __init__(self, log_file=None):
        # 创建用于进程间通信的队列
        self.queue = multiprocessing.Queue()
        # 默认日志文件名使用当前时间
        if log_file is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = f"trajectory.csv"
        self.log_file = log_file
        # 跟踪记录器进程
        self.logger_process = None
        
    def start(self):
        """启动轨迹记录器进程"""
        if self.logger_process is not None and self.logger_process.is_alive():
            print("轨迹记录器已经在运行中")
            return
        
        # 创建并启动记录器进程
        self.logger_process = multiprocessing.Process(target=self._log_trajectory)
        self.logger_process.daemon = True  # 设置为守护进程，主进程结束时自动终止
        self.logger_process.start()
        print(f"轨迹记录器已启动，日志文件: {self.log_file}")
        
    def stop(self):
        """停止轨迹记录器进程"""
        if self.logger_process is not None and self.logger_process.is_alive():
            # 发送终止信号
            self.queue.put(None)  # None 作为终止信号
            self.logger_process.join(timeout=2.0)  # 等待进程结束，最多等待2秒
            if self.logger_process.is_alive():
                self.logger_process.terminate()  # 强制终止
            self.logger_process = None
            print("轨迹记录器已停止")
        
    def log_position(self, x, y, z, timestamp=None):
        """记录当前位置信息"""
        if self.logger_process is None or not self.logger_process.is_alive():
            return
        
        if timestamp is None:
            timestamp = time.time()
        
        # 将位置信息放入队列
        self.queue.put((timestamp, x, y, z))
    
    def _log_trajectory(self):
        """轨迹记录器的主函数，在独立进程中运行"""
        try:
            # 检查目录是否存在，如果不存在则创建
            log_dir = os.path.dirname(self.log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            
            # 打开日志文件并写入表头
            with open(self.log_file, 'w', newline='') as f:
                f.write("timestamp,x,y,z\n")
                
                # 循环读取队列中的位置信息
                while True:
                    data = self.queue.get()
                    if data is None:  # 接收到终止信号
                        break
                    
                    timestamp, x, y, z = data
                    # 写入CSV格式的位置信息
                    f.write(f"{timestamp},{x},{y},{z}\n")
                    f.flush()  # 确保数据实时写入文件
                    
                    # 稍微休眠一下，避免CPU占用过高
                    time.sleep(0.01)
        except Exception as e:
            print(f"轨迹记录器发生错误: {e}")

# 使用示例（在主程序中集成时使用）
if __name__ == "__main__":
    # 这个示例展示了如何使用轨迹记录器
    logger = TrajectoryLogger()
    logger.start()
    
    try:
        # 模拟记录一些位置数据
        for i in range(10):
            logger.log_position(i, i*2, i*3)
            time.sleep(0.5)
    finally:
        logger.stop()