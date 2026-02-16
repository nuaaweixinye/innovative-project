import airsim
import cv2
import numpy as np
from ultralytics import YOLO
import time
from threading import Thread, Lock
from collections import defaultdict

# --------------------------
# 配置参数（可根据需求修改）
# --------------------------
YOLO_MODEL = "yolov8n.pt"  # YOLO模型
TARGET_CLASSES = ["person", "car"]  # 可跟踪的目标类别（多目标支持）
CAMERA_NAME = "front_center"  # 摄像头名称
IMAGE_SIZE = (640, 480)  # 图像尺寸（宽x高）
CONFIDENCE_THRESHOLD = 0.5  # 检测置信度阈值
MAX_YAW_RATE = 0.8  # 最大偏航角速度（rad/s）
MAX_SPEED = 3.0  # 最大线速度（m/s）
DESIRED_TARGET_WIDTH = 180  # 目标期望宽度（像素）
DRONE_NAMES = ["Drone1", "Drone2", "Drone3"]  # 无人机名称列表（需与AirSim一致）
UPDATE_INTERVAL = 2.0  # 目标分配更新间隔（秒，避免频繁切换）

# --------------------------
# 全局变量与锁（线程安全）
# --------------------------
detection_results = defaultdict(list)  # 存储所有无人机的检测结果 {无人机名: [(目标类别, bbox, 置信度), ...]}
assigned_targets = {}  # 目标分配结果 {无人机名: (目标类别, bbox)}
lock = Lock()  # 线程锁（保护共享变量）


class DroneTracker:
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self.client = airsim.MultirotorClient()  # 每个无人机独立连接（实际共享底层连接）
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle_name=drone_name)
        self.client.armDisarm(True, vehicle_name=drone_name)
        self.model = YOLO(YOLO_MODEL)  # 每个无人机加载独立模型实例
        self.center_x, self.center_y = IMAGE_SIZE[0] // 2, IMAGE_SIZE[1] // 2
        self.running = True  # 线程运行标志

    def takeoff(self):
        """无人机起飞并上升到指定高度"""
        self.client.takeoffAsync(vehicle_name=self.drone_name).join()
        self.client.moveToZAsync(-1, 1.0, vehicle_name=self.drone_name).join()  # 高度1米
        print(f"[{self.drone_name}] 起飞完成，准备跟踪")

    def get_image(self):
        """从无人机摄像头获取图像并预处理"""
        response = self.client.simGetImage(CAMERA_NAME, airsim.ImageType.Scene, vehicle_name=self.drone_name)
        if not response:
            return None
        
        try:
            img_buf = np.frombuffer(response, dtype=np.uint8)
            img = cv2.imdecode(img_buf, cv2.IMREAD_COLOR)
            return cv2.resize(img, IMAGE_SIZE) if img.shape[:2] != IMAGE_SIZE[::-1] else img
        except:
            # 处理原始RGB数据
            expected_size = IMAGE_SIZE[0] * IMAGE_SIZE[1] * 3
            if len(response) != expected_size:
                return None
            img = np.frombuffer(response, dtype=np.uint8).reshape((IMAGE_SIZE[1], IMAGE_SIZE[0], 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def detect_targets(self, img):
        """检测图像中的目标并更新全局检测结果"""
        results = self.model(img, conf=CONFIDENCE_THRESHOLD, verbose=False)
        detected = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)
                class_name = self.model.names[class_id]
                if class_name in TARGET_CLASSES:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = box.conf.item()
                    detected.append((class_name, (x1, y1, x2, y2), conf))
        # 线程安全地更新全局检测结果
        with lock:
            detection_results[self.drone_name] = detected
        return detected

    def track_target(self, img, target):
        """根据分配的目标控制无人机运动"""
        if not target:
            self.client.hoverAsync(vehicle_name=self.drone_name).join()
            return img
        
        class_name, bbox = target
        x1, y1, x2, y2 = bbox
        # 计算目标中心与偏移
        target_center_x = (x1 + x2) / 2
        horizontal_offset = target_center_x - self.center_x
        target_width = x2 - x1

        # 计算偏航角速度（转向控制）
        yaw_rate = -horizontal_offset * 0.003
        yaw_rate = np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)

        # 计算前进/后退速度（距离控制）
        speed = (DESIRED_TARGET_WIDTH - target_width) * 0.015
        speed = np.clip(speed, -MAX_SPEED, MAX_SPEED)

        # 发送控制指令
        self.client.moveByVelocityAsync(
            vx=speed, vy=0, vz=0, duration=0.1,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
            vehicle_name=self.drone_name
        ).join()

        # 绘制跟踪目标框
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        cv2.putText(img, f"Tracking: {class_name}", 
                    (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
        return img

    def run(self):
        """无人机跟踪主循环（线程执行）"""
        while self.running:
            start_time = time.time()
            img = self.get_image()
            if img is None:
                time.sleep(0.1)
                continue

            # 检测目标并更新全局结果
            self.detect_targets(img)

            # 获取当前分配的目标（线程安全）
            with lock:
                current_target = assigned_targets.get(self.drone_name, None)

            # 跟踪目标并绘制图像
            img = self.track_target(img, current_target)
            cv2.putText(img, f"Drone: {self.drone_name}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.imshow(f"Tracking View - {self.drone_name}", img)

            # 处理退出指令
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                break

            # 控制帧率（避免CPU过载）
            elapsed = time.time() - start_time
            if elapsed < 0.05:  # 最低20FPS
                time.sleep(0.05 - elapsed)

    def stop(self):
        """停止无人机并释放资源"""
        self.running = False
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
        self.client.armDisarm(False, vehicle_name=self.drone_name)
        self.client.enableApiControl(False, vehicle_name=self.drone_name)
        print(f"[{self.drone_name}] 已停止")


def target_assigner():
    """目标分配器（独立线程）：根据检测结果分配目标，避免重复跟踪"""
    last_update = time.time()
    while True:
        # 定时更新分配结果（避免频繁切换）
        if time.time() - last_update < UPDATE_INTERVAL:
            time.sleep(0.1)
            continue
        last_update = time.time()

        with lock:
            # 收集所有检测到的目标（去重，按置信度排序）
            all_targets = []
            for drone_name, dets in detection_results.items():
                for cls, bbox, conf in dets:
                    # 用目标中心和类别作为唯一标识（简单去重）
                    center = ((bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2)
                    all_targets.append((-conf, cls, bbox, drone_name, center))  # 负号用于降序

            # 按置信度降序排序（优先分配高置信度目标）
            all_targets.sort()
            assigned_centers = set()  # 已分配的目标中心（避免重复）
            new_assigned = {}

            # 为每个无人机分配一个未被跟踪的目标
            for drone_name in DRONE_NAMES:
                for t in all_targets:
                    conf_neg, cls, bbox, det_drone, center = t
                    # 优先分配无人机自己检测到的目标（视野内更可靠）
                    if det_drone == drone_name and center not in assigned_centers:
                        new_assigned[drone_name] = (cls, bbox)
                        assigned_centers.add(center)
                        break
                # 若未分配到目标，保持原分配或空
                if drone_name not in new_assigned:
                    new_assigned[drone_name] = assigned_targets.get(drone_name, None)

            # 更新全局分配结果
            assigned_targets.update(new_assigned)
            print(f"目标分配更新: { {k: v[0] if v else None for k, v in new_assigned.items()} }")

        # 检查退出信号
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




# 自动获取所有连接的无人机名称
client = airsim.MultirotorClient()
client.confirmConnection()
DRONE_NAMES = client.listVehicles()

print(f"发现 {len(DRONE_NAMES)} 个无人机: {DRONE_NAMES}")
# 初始化所有无人机
drones = [DroneTracker(name) for name in DRONE_NAMES]
for drone in drones:
    drone.takeoff()
    time.sleep(0.8)  # 错开起飞时间


if __name__ == "__main__":
    try:
        # 启动目标分配线程
        assigner_thread = Thread(target=target_assigner, daemon=True)
        assigner_thread.start()

        # 启动所有无人机的跟踪线程
        drone_threads = []
        for drone in drones:
            t = Thread(target=drone.run)
            t.start()
            drone_threads.append(t)

        # 等待所有线程结束
        for t in drone_threads:
            t.join()

    except KeyboardInterrupt:
        print("用户手动中断")
    finally:
        # 确保所有无人机停止
        for drone in drones:
            drone.stop()
        cv2.destroyAllWindows()
        print("程序结束")