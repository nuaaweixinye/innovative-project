import airsim
import cv2
import numpy as np
from ultralytics import YOLO
import time
from threading import Thread, Lock
from collections import defaultdict

# --------------------------
# 配置参数（新增主从协同参数）
# --------------------------
YOLO_MODEL = "yolov8n.pt"  # YOLO模型
TARGET_CLASSES = ["person", "car"]  # 可跟踪的目标类别
CAMERA_NAME = "front_center"  # 摄像头名称
IMAGE_SIZE = (640, 480)  # 图像尺寸（宽x高）
CONFIDENCE_THRESHOLD = 0.5  # 检测置信度阈值
MAX_YAW_RATE = 0.8  # 最大偏航角速度（rad/s）
MAX_SPEED = 3.0  # 最大线速度（m/s）
DESIRED_TARGET_WIDTH = 180  # 目标期望宽度（像素）
DRONE_NAMES = ["Drone1", "Drone2", "Drone3"]  # 初始无人机名称（实际自动获取）
UPDATE_INTERVAL = 2.0  # 目标分配更新间隔（秒）
# 新增主从协同参数
MASTER_DRONES_NUM = 1  # 主跟踪无人机数量（默认1架核心跟踪）
SLAVE_DISTANCE = 10.0  # 从无人机与主无人机的协同距离（米）
SLAVE_ANGLE_OFFSET = [90, 270]  # 从无人机相对主无人机的角度偏移（度）：左右侧方

# --------------------------
# 全局变量与锁（新增主从标识）
# --------------------------
detection_results = defaultdict(list)  # {无人机名: [(类别, bbox, 置信度), ...]}
assigned_targets = {}  # {无人机名: (目标类别, bbox, 角色)} 角色："master"/"slave"
global_best_target = None  # 全局最高置信度目标：(类别, bbox, 置信度, 检测无人机)
lock = Lock()  # 线程锁


class DroneTracker:
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle_name=drone_name)
        self.client.armDisarm(True, vehicle_name=drone_name)
        self.model = YOLO(YOLO_MODEL)
        self.center_x, self.center_y = IMAGE_SIZE[0] // 2, IMAGE_SIZE[1] // 2
        self.running = True
        # 新增主从状态与位置记录
        self.role = None  # "master"/"slave"/None
        self.master_position = None  # 主无人机位置（用于从无人机协同）

    def takeoff(self):
        """无人机起飞并上升到指定高度"""
        self.client.takeoffAsync(vehicle_name=self.drone_name).join()
        self.client.moveToZAsync(-1, 1.0, vehicle_name=self.drone_name).join()  # 高度1米
        print(f"[{self.drone_name}] 起飞完成，角色：{self.role if self.role else '待分配'}")

    def get_image(self):
        """从无人机摄像头获取图像并预处理（保持原逻辑）"""
        response = self.client.simGetImage(CAMERA_NAME, airsim.ImageType.Scene, vehicle_name=self.drone_name)
        if not response:
            return None
        
        try:
            img_buf = np.frombuffer(response, dtype=np.uint8)
            img = cv2.imdecode(img_buf, cv2.IMREAD_COLOR)
            return cv2.resize(img, IMAGE_SIZE) if img.shape[:2] != IMAGE_SIZE[::-1] else img
        except:
            expected_size = IMAGE_SIZE[0] * IMAGE_SIZE[1] * 3
            if len(response) != expected_size:
                return None
            img = np.frombuffer(response, dtype=np.uint8).reshape((IMAGE_SIZE[1], IMAGE_SIZE[0], 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def detect_targets(self, img):
        """检测图像中的目标并更新全局检测结果（保持原逻辑）"""
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
        with lock:
            detection_results[self.drone_name] = detected
        return detected

    def get_drone_position(self):
        """获取无人机当前位置（新增：用于主从协同）"""
        pose = self.client.simGetVehiclePose(vehicle_name=self.drone_name)
        return (pose.position.x_val, pose.position.y_val, pose.position.z_val)

    def track_target(self, img, target_info):
        """修改跟踪逻辑：区分主从无人机控制"""
        if not target_info:
            self.client.hoverAsync(vehicle_name=self.drone_name).join()
            return img
        
        class_name, bbox, role = target_info
        self.role = role  # 更新当前角色
        x1, y1, x2, y2 = bbox

        # 1. 主无人机：精准跟踪目标（基于原逻辑优化）
        if role == "master":
            # 记录主无人机位置（供从无人机参考）
            self.master_position = self.get_drone_position()
            # 计算目标偏移与控制量
            target_center_x = (x1 + x2) / 2
            horizontal_offset = target_center_x - self.center_x
            target_width = x2 - x1

            # 偏航角控制（转向）
            yaw_rate = -horizontal_offset * 0.003
            yaw_rate = np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)
            # 线速度控制（距离）
            speed = (DESIRED_TARGET_WIDTH - target_width) * 0.015
            speed = np.clip(speed, -MAX_SPEED, MAX_SPEED)

            # 发送控制指令
            self.client.moveByVelocityAsync(
                vx=speed, vy=0, vz=0, duration=0.1,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                vehicle_name=self.drone_name
            ).join()

            # 绘制主跟踪标识
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
            cv2.putText(img, f"MASTER: Track {class_name}", 
                        (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)

        # 2. 从无人机：跟随主无人机协同（新增逻辑）
        elif role == "slave":
            if not self.master_position:
                self.client.hoverAsync(vehicle_name=self.drone_name).join()
                return img
            
            # 获取从无人机当前位置
            slave_x, slave_y, slave_z = self.get_drone_position()
            master_x, master_y, master_z = self.master_position
            # 计算从无人机目标位置（基于主无人机位置+角度偏移）
            # 分配从无人机角度（按索引分配：0→90度，1→270度，循环）
            slave_idx = list(assigned_targets.keys()).index(self.drone_name) - MASTER_DRONES_NUM
            angle = SLAVE_ANGLE_OFFSET[slave_idx % len(SLAVE_ANGLE_OFFSET)] * np.pi / 180
            # 计算目标坐标（极坐标转直角坐标）
            target_x = master_x + SLAVE_DISTANCE * np.cos(angle)
            target_y = master_y + SLAVE_DISTANCE * np.sin(angle)
            target_z = master_z  # 与主无人机同高度

            # 移动到协同位置
            self.client.moveToPositionAsync(
                target_x, target_y, target_z, MAX_SPEED,
                yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0),  # 朝向目标
                vehicle_name=self.drone_name
            ).join()

            # 绘制从协同标识
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, f"SLAVE: Assist {class_name}", 
                        (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

        # 绘制无人机与角色信息
        cv2.putText(img, f"Drone: {self.drone_name} ({self.role})", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        return img

    def run(self):
        """无人机跟踪主循环（保持原逻辑，适配新跟踪函数）"""
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
            cv2.imshow(f"Tracking View - {self.drone_name}", img)

            # 处理退出指令
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                break

            # 控制帧率（最低20FPS）
            elapsed = time.time() - start_time
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

    def stop(self):
        """停止无人机并释放资源（保持原逻辑）"""
        self.running = False
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
        self.client.armDisarm(False, vehicle_name=self.drone_name)
        self.client.enableApiControl(False, vehicle_name=self.drone_name)
        print(f"[{self.drone_name}] 已停止（角色：{self.role}）")


def target_assigner():
    """修改目标分配逻辑：聚焦全局最高置信度目标+主从角色分配"""
    last_update = time.time()
    while True:
        # 定时更新（避免频繁切换）
        if time.time() - last_update < UPDATE_INTERVAL:
            time.sleep(0.1)
            continue
        last_update = time.time()

        with lock:
            # 步骤1：汇总所有无人机的检测结果，筛选有效目标
            all_valid_targets = []
            for drone_name, dets in detection_results.items():
                for cls, bbox, conf in dets:
                    # 计算目标中心（用于去重）
                    target_center = ((bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2)
                    all_valid_targets.append(
                        (-conf, cls, bbox, conf, drone_name, target_center)  # 负号用于降序排序
                    )

            # 步骤2：筛选全局最高置信度目标（去重）
            global global_best_target
            if not all_valid_targets:
                global_best_target = None
                assigned_targets.clear()
                print("目标分配更新：无有效目标，所有无人机悬停")
                continue

            # 按置信度降序排序，取第一个目标（最高置信度）
            all_valid_targets.sort()
            best_target = all_valid_targets[0]
            best_neg_conf, best_cls, best_bbox, best_conf, best_det_drone, best_center = best_target

            # 目标去重：检查是否有其他无人机检测到同一目标（中心距离<30像素）
            for target in all_valid_targets[1:]:
                neg_conf, cls, bbox, conf, det_drone, center = target
                if cls == best_cls and abs(center[0]-best_center[0]) < 30 and abs(center[1]-best_center[1]) < 30:
                    # 若存在同一目标且置信度更高，更新最佳目标
                    if -neg_conf > best_conf:
                        best_conf = -neg_conf
                        best_bbox = bbox
                        best_det_drone = det_drone
                        best_center = center
            # 更新全局最佳目标
            global_best_target = (best_cls, best_bbox, best_conf, best_det_drone)

            # 步骤3：分配主从角色与目标
            new_assigned = {}
            drone_list = list(DRONE_NAMES)
            if not drone_list:
                continue

            # 分配主无人机（默认1架，优先选择检测到最佳目标的无人机）
            master_drone = best_det_drone if best_det_drone in drone_list else drone_list[0]
            new_assigned[master_drone] = (best_cls, best_bbox, "master")

            # 分配从无人机（剩余无人机，目标与主无人机一致，角色为slave）
            slave_drones = [d for d in drone_list if d != master_drone]
            for slave_drone in slave_drones:
                new_assigned[slave_drone] = (best_cls, best_bbox, "slave")

            # 更新全局分配结果
            assigned_targets.update(new_assigned)
            # 打印分配结果（简化显示）
            assign_log = {d: f"{r} (track {c})" for d, (c, _, r) in new_assigned.items()}
            print(f"目标分配更新：全局最佳目标（{best_cls}, 置信度{best_conf:.2f}），分配结果：{assign_log}")

        # 检查退出信号
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# --------------------------
# 系统启动入口（保持原逻辑）
# --------------------------
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