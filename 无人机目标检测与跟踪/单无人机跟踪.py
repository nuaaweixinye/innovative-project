import airsim
import cv2
import numpy as np
from ultralytics import YOLO
import time

# --------------------------
# 配置参数（可根据需求修改）
# --------------------------
YOLO_MODEL = "yolov8n.pt"  # 官方轻量模型（nano版，速度快）
TARGET_CLASS = "person"    # 跟踪目标（可选：car, dog, bicycle等，见官方80类）
CAMERA_NAME = "front_center"  # AirSim摄像头名称（与AirSim设置一致）
IMAGE_SIZE = (640, 480)    # 图像尺寸（宽x高）
CONFIDENCE_THRESHOLD = 0.5  # 目标检测置信度阈值
MAX_YAW_RATE = 0.8          # 最大偏航角速度（rad/s，控制转向灵敏度）
MAX_SPEED = 3.0             # 最大前进/后退速度（m/s）
DESIRED_TARGET_WIDTH = 180  # 目标在图像中的期望宽度（像素，控制距离）

# --------------------------
# 初始化连接与无人机
# --------------------------
# 连接AirSim仿真环境
client = airsim.MultirotorClient()
client.confirmConnection()  # 验证连接
client.enableApiControl(True)  # 启用API控制
client.armDisarm(True)       # 解锁电机

# 起飞并上升到1米高度
client.takeoffAsync().join()
client.moveToZAsync(-1, 1.0).join()  # Z为负表示高度（单位：米）
print("无人机准备就绪，开始跟踪...")

# --------------------------
# 加载YOLO官方模型
# --------------------------
model = YOLO(YOLO_MODEL)  # 首次运行会自动下载模型到本地
print(f"加载YOLO模型成功：{YOLO_MODEL}")
print(f"支持跟踪的目标类别：{model.names}（当前跟踪：{TARGET_CLASS}）")

# 图像中心坐标（用于计算目标偏移）
center_x, center_y = IMAGE_SIZE[0] // 2, IMAGE_SIZE[1] // 2

# --------------------------
# 主循环：检测与跟踪
# --------------------------
try:
    while True:
        start_time = time.time()  # 计时，用于计算帧率

        # 1. 从AirSim获取摄像头图像
        response = client.simGetImage(CAMERA_NAME, airsim.ImageType.Scene)
        if not response:
            print("未获取到图像，重试...")
            time.sleep(0.5)
            continue

        try:
            # 尝试解码压缩格式图像（PNG/JPG）
            # 将二进制数据转为OpenCV图像（自动处理压缩格式）
            img_buf = np.frombuffer(response, dtype=np.uint8)
            img = cv2.imdecode(img_buf, cv2.IMREAD_COLOR)  # 解码为BGR格式
            
            # 确保图像尺寸与预设一致（缩放至目标尺寸）
            if img.shape[1] != IMAGE_SIZE[0] or img.shape[0] != IMAGE_SIZE[1]:
                img = cv2.resize(img, IMAGE_SIZE)
                print(f"图像尺寸自动调整为{IMAGE_SIZE}")

        except Exception as e:
            # 若解码失败，尝试解析为原始RGB数据
            expected_size = IMAGE_SIZE[0] * IMAGE_SIZE[1] * 3
            if len(response) != expected_size:
                print(f"图像尺寸不匹配：期望{expected_size}字节，实际{len(response)}字节，跳过此帧")
                time.sleep(0.1)
                continue
            
            # 处理原始RGB数据
            img = np.frombuffer(response, dtype=np.uint8)
            img = img.reshape((IMAGE_SIZE[1], IMAGE_SIZE[0], 3))  # 高x宽x通道
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # 转为BGR格式
            print("使用原始RGB格式图像")

        # 2. YOLO目标检测
        results = model(img, conf=CONFIDENCE_THRESHOLD, verbose=False)
        target_detected = False
        target_bbox = None  # 目标边界框 (x1, y1, x2, y2)

        # 解析检测结果，筛选目标类别
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)
                class_name = model.names[class_id]  # 获取类别名称
                if class_name == TARGET_CLASS:
                    target_detected = True
                    # 提取边界框坐标（像素值）
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    target_bbox = (x1, y1, x2, y2)
                    # 在图像上绘制边界框和标签
                    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    # 将 box.conf 从张量转换为标量值
                    conf_value = box.conf.item()
                    cv2.putText(img, f"{class_name} {conf_value:.2f}",
                                (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)


        # 3. 根据目标位置控制无人机
        if target_detected and target_bbox:
            x1, y1, x2, y2 = target_bbox
            # 计算目标中心坐标
            target_center_x = (x1 + x2) / 2
            target_center_y = (y1 + y2) / 2
            # 计算与图像中心的水平偏移（控制转向）
            horizontal_offset = target_center_x - center_x
            # 计算目标宽度（反映距离：宽度越大，距离越近）
            target_width = x2 - x1

            # 计算偏航角速度（转向控制）
            # 增益系数0.003：数值越大，转向越灵敏（需根据场景调试）
            yaw_rate = -horizontal_offset * 0.003
            yaw_rate = np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)  # 限制最大角速度

            # 计算前进/后退速度（距离控制）
            # 目标宽度 < 期望宽度：前进；反之：后退
            speed = (DESIRED_TARGET_WIDTH - target_width) * 0.015
            speed = np.clip(speed, -MAX_SPEED, MAX_SPEED)  # 限制最大速度

            # 发送速度控制指令（vx: 前进/后退, vy: 左右, vz: 升降, yaw_rate: 偏航）
            print(f"跟踪中 | 偏航角速度: {yaw_rate:.2f} rad/s | 速度: {speed:.2f} m/s")
            client.moveByVelocityAsync(
                vx=speed,
                vy=0,  # 暂时不控制左右
                vz=0,  # 暂时不控制升降
                duration=0.1,  # 指令持续时间（秒）
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)
            ).join()
            
            # 显示处理后的图像
            cv2.imshow("YOLO Detection & Tracking", img)

            # 计算并显示帧率
            fps = 1 / (time.time() - start_time)
            #print(f"帧率: {fps:.1f} FPS | 按 'q' 退出\n")
            
            # 按'q'键退出程序
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break     
           
        else:
            # 显示处理后的图像
            cv2.imshow("YOLO Detection & Tracking", img)

            # 计算并显示帧率
            fps = 1 / (time.time() - start_time)
            #print(f"帧率: {fps:.1f} FPS | 按 'q' 退出\n")
                
            # 未检测到目标时悬停
            client.hoverAsync().join()
            time.sleep(1)
            print(f"未检测到{TARGET_CLASS}，悬停1秒")
            # 按'q'键退出程序
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("用户手动中断程序")

finally:
    # 程序结束时的清理工作
    client.armDisarm(False)  # 锁定电机
    client.enableApiControl(False)  # 关闭API控制
    cv2.destroyAllWindows()  # 关闭图像窗口
    print("程序结束")