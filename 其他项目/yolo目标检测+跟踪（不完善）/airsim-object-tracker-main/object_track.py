import airsim
import numpy as np
import math
import time
import pandas as pd
import cv2
import torch
import threading
from airsim import Vector3r, to_quaternion, Pose
import atexit
from ultralytics import YOLO

from deep_sort_realtime.deepsort_tracker import DeepSort
from filterpy.kalman import KalmanFilter
from collections import defaultdict

# === 模型设置 ===
model = YOLO(
   "weigths/yolov11m_myeong_best_airsim100.pt"
)  # yolov11m_myeong_best模型权重路径

"""
# 水平距离计算函数
# 利用摄像头下俯角和图像中目标位置，通过三角函数（正切）估算水平距离
# -------------------------
# def estimate_horizontal_distance_from_object(
#     drone_altitude_m,
#     camera_pitch_deg,
#     object_center_y_px,
#     image_height_px,
#     vertical_fov_deg,
# ):
#     deg_per_pixel = vertical_fov_deg / image_height_px  # 每像素对应的角度（垂直方向）
#     offset_pixel = object_center_y_px - (image_height_px / 2)  # 目标与图像中心的垂直像素偏移
#     offset_angle_deg = offset_pixel * deg_per_pixel  # 垂直角度偏移
#     total_angle_deg = camera_pitch_deg + offset_angle_deg  # 总垂直角度
#     total_angle_rad = math.radians(total_angle_deg)  # 角度转弧度
#     if total_angle_rad <= 0:
#         return None  # 或返回无穷大，避免除零错误
#     horizontal_distance_m = drone_altitude_m / math.tan(total_angle_rad)  # 水平距离计算
#     return round(horizontal_distance_m, 2)
"""

# 下方是考虑水平方向（左右）的完整距离计算函数
def estimate_horizontal_distance_from_object(
    drone_altitude_m,          # 无人机高度（米）
    camera_pitch_deg,          # 摄像头俯仰角（度）
    camera_yaw_deg,            # 摄像头偏航角（度）
    object_center_x_px,        # 目标中心X坐标（像素）
    object_center_y_px,        # 目标中心Y坐标（像素）
    image_width_px,            # 图像宽度（像素）
    image_height_px,           # 图像高度（像素）
    horizontal_fov_deg,        # 水平视场角（度）
    vertical_fov_deg,          # 垂直视场角（度）
):
    # 根据视场角计算每像素对应的角度
    deg_per_px_y = vertical_fov_deg / image_height_px    # 垂直方向每像素角度
    deg_per_px_x = horizontal_fov_deg / image_width_px    # 水平方向每像素角度

    # 计算目标与图像中心的像素偏移
    offset_y_px = object_center_y_px - (image_height_px / 2)  # 垂直偏移
    offset_x_px = object_center_x_px - (image_width_px / 2)   # 水平偏移

# 计算目标对应的俯仰角、偏航角偏移
    offset_pitch_deg = offset_y_px * deg_per_px_y    # 俯仰角偏移
    offset_yaw_deg = offset_x_px * deg_per_px_x      # 偏航角偏移

    # 计算目标对应的总俯仰角、总偏航角
    total_pitch_deg = camera_pitch_deg + offset_pitch_deg  # 总俯仰角
    total_yaw_deg = camera_yaw_deg + offset_yaw_deg        # 总偏航角

    # 根据俯仰角计算基础水平距离
    total_pitch_rad = math.radians(total_pitch_deg)  # 俯仰角转弧度
    if total_pitch_rad <= 0:
        # raise ValueError("总俯仰角小于等于0度，请检查摄像头角度")
        return None  # 俯仰角异常时返回None，跳过该目标

    base_distance = drone_altitude_m / math.tan(total_pitch_rad)  # 基础水平距离

    # 考虑偏航角修正（目标越靠左右两侧，实际距离越远）
    yaw_correction = math.cos(math.radians(total_yaw_deg))  # 偏航角修正系数
    if yaw_correction <= 0.01:  # 避免过度侧移导致修正系数过小
        yaw_correction = 0.01

    corrected_distance = base_distance / yaw_correction  # 修正后的实际水平距离
    return round(corrected_distance, 2)


# 待跟踪的特定目标ID（初始值：None，未选择目标）
target_track_id = None
# 鼠标点击选择特定目标（人/车）
clicked_point = None  # 存储用户点击的图像坐标


### "再次点击取消跟踪"功能
def select_target(event, x, y, flags, param):
    global clicked_point, target_track_id
    if event == cv2.EVENT_LBUTTONDOWN:  # 检测鼠标左键点击
        if target_track_id is not None:
            # 👉 已处于跟踪状态时，点击取消跟踪
            print(f"❌ 取消跟踪（目标ID：{target_track_id}）")
            target_track_id = None
            clicked_point = None
        else:
            # 👉 未跟踪目标时，记录点击坐标以选择目标
            clicked_point = (x, y)
# 线程停止信号（用于优雅关闭YOLO检测线程）
stop_event = threading.Event()


def yolo_worker_ultralytics():
    global clicked_point
    print("[YOLO] Ultralytics YOLO目标检测线程启动")
    yolo_client = airsim.MultirotorClient()  # 初始化AirSim客户端
    yolo_client.confirmConnection()  # 确认与AirSim的连接
    
    # 创建OpenCV窗口并绑定鼠标点击事件（用于选择目标）
    cv2.namedWindow("YOLO + DeepSORT")
    cv2.setMouseCallback("YOLO + DeepSORT", select_target)

    # ✅ 视频保存设置
    output_path = "output_video/airsim无人机_驻地边界.mp4"  # 输出视频路径
    fps = 1  # 视频帧率
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 视频编码格式（MP4）
    out = cv2.VideoWriter(output_path, fourcc, fps, (1280, 720))  # 初始化视频写入器

    while not stop_event.is_set():  # 线程未收到停止信号时持续运行
        try:
            ## 核心代码：从AirSim获取摄像头图像 ###
            response = yolo_client.simGetImage(
                "front_center",  # 摄像头名称（前视中心摄像头）
                airsim.ImageType.Scene,  # 图像类型（场景图，彩色）
                vehicle_name="Drone1"  # 无人机名称
            )

            if response:  # 成功获取图像时
                # 解码图像（二进制→numpy数组→OpenCV格式）
                img_array = np.frombuffer(response, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                height, width, _ = frame.shape  # 获取图像尺寸

                ### 距离计算所需参数获取
                # 获取无人机当前状态（位置、姿态等）
                state = yolo_client.getMultirotorState(vehicle_name="Drone1")
                DRONE_ALTITUDE = -state.kinematics_estimated.position.z_val  # 无人机高度（AirSim中z负向为上方）
                
                # 自动获取摄像头参数（位置、姿态、视场角）
                cam_info = yolo_client.simGetCameraInfo("front_center")
                pitch_rad, _, yaw_rad = airsim.to_eularian_angles(
                    cam_info.pose.orientation  # 摄像头姿态（四元数→欧拉角）
                )

                # 摄像头角度转换：AirSim默认与三角函数方向相反，需取负
                CAMERA_YAW_DEG = math.degrees(yaw_rad)  # 偏航角（弧度→度）
                CAMERA_PITCH_DEG = -math.degrees(pitch_rad)  # 俯仰角（取负后转度）
                VERTICAL_FOV_DEG = cam_info.fov  # 垂直视场角（度）
                IMAGE_WIDTH_PX = width  # 图像宽度（像素）
                IMAGE_HEIGHT_PX = height  # 图像高度（像素）
                
                # 计算水平视场角（根据图像宽高比和垂直视场角推导）
                HORIZONTAL_FOV_DEG = 2 * math.degrees(
                    math.atan(
                        math.tan(math.radians(VERTICAL_FOV_DEG / 2))
                        * (IMAGE_WIDTH_PX / IMAGE_HEIGHT_PX)
                    )
                )

                FOCAL_LENGTH_PX = 800  # 焦距（像素，示例值，实际需校准）

                # 初始化DeepSORT目标跟踪器
                tracker = DeepSort(max_age=30, n_init=1, max_iou_distance=0.7)  # 添加这行代码
                
                # 初始化卡尔曼滤波器（用于目标位置预测）
                # 状态维度4（x位置、y位置、x速度、y速度），观测维度2（x位置、y位置）
                kf = KalmanFilter(dim_x=4, dim_z=2)  
                # 状态转移矩阵（匀速运动模型）
                kf.F = np.array(
                    [
                        [1, 0, 1, 0],  # x_new = x_old + vx*1
                        [0, 1, 0, 1],  # y_new = y_old + vy*1
                        [0, 0, 1, 0],  # vx保持不变
                        [0, 0, 0, 1],  # vy保持不变
                    ]
                )
                # 观测矩阵（仅观测位置，不观测速度）
                kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  
                kf.P *= 1000  # 初始状态协方差（不确定性）
                kf.R *= 10    # 观测噪声协方差


                if frame is not None:
                    # ⏩ 调用YOLO模型进行目标检测
                    results = model(frame,verbose=False,show=False)[0]  # 获取检测结果（取第一个批次）
                    print("[YOLO] 检测到的目标框数量：", len(results.boxes))
                    detections = []  # 存储符合跟踪条件的目标（用于DeepSORT）

                    # ✅ 指定需要跟踪的目标类别（COCO数据集类别ID）
                    track_classes = [0, 2, 7]  # 0:人(person)，2:汽车(car)，7:卡车(truck)

                    # ✅ 初始化目标信息存储列表（用于统计和打印）
                    detected_objects_info = []
                    class_counter = defaultdict(int)  # 统计各类别目标数量的字典
                    
                    # 存储目标框与距离的映射，用于后续自动选择最近目标
                    boxes_with_distance = []

                    ### 遍历检测结果，提取目标信息 ###
                    for result in results.boxes.data:
                        # 解析目标框坐标、置信度、类别ID（YOLO输出格式：x1,y1,x2,y2,score,class_id）
                        x1, y1, x2, y2, score, class_id = result.tolist()
                        center_x = int((x1 + x2) / 2)  # 目标中心X坐标
                        center_y = int((y1 + y2) / 2)  # 目标中心Y坐标
                        class_id = int(class_id)  # 类别ID（转为整数）
                        # 获取类别名称（如无对应名称则显示"Unknown"）
                        label = model.names[class_id] if class_id in model.names else "Unknown"
                        
                        # 计算目标与无人机的水平距离
                        distance_to_object = estimate_horizontal_distance_from_object(
                            DRONE_ALTITUDE,
                            CAMERA_PITCH_DEG,
                            CAMERA_YAW_DEG,
                            center_x,
                            center_y,
                            IMAGE_WIDTH_PX,
                            IMAGE_HEIGHT_PX,
                            HORIZONTAL_FOV_DEG,
                            VERTICAL_FOV_DEG,
                        )
                        if distance_to_object is None:
                            continue  # 距离估算异常时，跳过该目标

                        # 👉 存储当前目标的信息（用于统计）
                        detected_objects_info.append(
                            {
                                "类别": label,
                                "距离": distance_to_object,
                                "置信度": round(score, 2),
                            }
                        )
                        # ✅ 统计各类别目标数量
                        class_counter[label] += 1

                        # 筛选符合条件的目标（指定类别 + 置信度>0.5），用于DeepSORT跟踪
                        if class_id in track_classes and score > 0.5:
                            # DeepSORT要求输入格式：(目标框[x1,y1,w,h], 置信度, 类别ID)
                            detections.append(([x1, y1, x2 - x1, y2 - y1], score, class_id))
                            # 存储目标框和距离的映射
                            boxes_with_distance.append(([x1, y1, x2 - x1, y2 - y1], distance_to_object))
                    
                    # ✅ 打印检测结果统计信息
                    print(f"[YOLO] ✅ 有效检测目标数量：{len(detected_objects_info)}")
                    for label, count in class_counter.items():
                        print(f" - {label}：{count}个")

                    # 调用DeepSORT进行目标跟踪（输入检测结果和图像）
                    tracked_objects = tracker.update_tracks(detections, frame=frame)
                    print(f"[跟踪] 跟踪到的目标数量：{len(list(tracked_objects))}")

                    global target_track_id
                    # 若未选择跟踪目标
                    if target_track_id is None:
                        print(f"[选择] 当前跟踪目标ID：{target_track_id}")
                        print(f"[选择] 用户点击坐标：{clicked_point}")
                        
                        # 用户点击选择目标的情况
                        if clicked_point is not None:
                            min_dist = float("inf")  # 初始化最小距离为无穷大
                            for track in tracked_objects:
                                if not track.is_confirmed():
                                    continue
                                ltrb = track.to_ltrb()
                                x1, y1, x2, y2 = map(int, ltrb)
                                target_center = ((x1 + x2) // 2, (y1 + y2) // 2)  # 跟踪目标中心

                                # 计算点击坐标与跟踪目标中心的欧氏距离
                                distance = np.linalg.norm(
                                    np.array(target_center) - np.array(clicked_point)
                                )
                                # 找到距离点击坐标最近的跟踪目标
                                if distance < min_dist:
                                    min_dist = distance
                                    target_track_id = track.track_id  # 记录目标ID
                            
                            print(f"🔍 已选择跟踪目标，ID：{target_track_id}")
                            clicked_point = None  # 重置点击坐标，避免重复选择
                        
                        # 用户未点击，强制选择一个目标进行跟踪
                        else:
                            print("[自动选择] 用户未操作，强制选择一个目标进行跟踪")
                            
                            # 策略1：尝试使用已确认的跟踪目标
                            confirmed_tracks = [track for track in tracked_objects if track.is_confirmed()]
                            if confirmed_tracks:
                                target_track_id = confirmed_tracks[0].track_id
                                print(f"🔍 自动选择已确认的目标，ID：{target_track_id}")
                            
                            # 策略2：如果没有已确认的跟踪目标，但有跟踪目标，就直接选择第一个
                            elif len(list(tracked_objects)) > 0:
                                first_track = next(iter(tracked_objects), None)
                                if first_track:
                                    target_track_id = first_track.track_id
                                    print(f"🔍 自动选择第一个跟踪目标（未确认状态），ID：{target_track_id}")
                            
                            # 策略3：如果连跟踪目标都没有，但有检测到的目标，直接从YOLO检测结果中选择
                            elif len(detections) > 0:
                                # 这里我们创建一个虚拟的track_id，直接使用YOLO的第一个检测结果
                                target_track_id = 999  # 使用一个固定的ID表示直接从YOLO结果选择
                                print(f"🔍 直接从YOLO检测结果中选择目标，虚拟ID：{target_track_id}")
                            
                            # 删除默认跟踪ID 0的设置
                            # 不再设置默认值，保持target_track_id为None
                            else:
                                print("🔍 未检测到任何目标，不设置默认跟踪ID")
                        target_found = False  # 标记当前帧是否检测到跟踪目标

                    # 遍历所有跟踪目标，绘制边界框和信息
                    for track in tracked_objects:
                        # 为了确保一定能跟踪，我们放宽条件，不再检查is_confirmed()
                        # if not track.is_confirmed():
                        #     continue
                        track_id = track.track_id  # 跟踪目标ID
                        print(f"[跟踪] 当前跟踪目标ID：{track_id}")
                        # 边界框坐标转换（ltrb→整数）
                        ltrb = track.to_ltrb()
                        x1, y1, x2, y2 = map(int, ltrb)
                        target_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])  # 目标中心

                        if target_track_id is None:
                            # 未选择跟踪目标时：绘制所有目标的边界框（绿色，细线）
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                            # 绘制类别标签（绿色，小字体）
                            cv2.putText(
                                frame,
                                f"{label}",
                                (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1,
                            )
                        elif track_id == target_track_id:
                            # 已选择跟踪目标且当前目标为跟踪对象：绘制突出边界框
                            target_found = True  # 标记跟踪目标已找到
                            kf.update(target_center)  # 用当前目标位置更新卡尔曼滤波器

                            # 绘制跟踪目标边界框（绿色，粗线）
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            # 绘制跟踪信息（ID + 距离，绿色，大字体）
                            cv2.putText(
                                frame,
                                f"跟踪目标 ID {track_id} / 距离:{distance_to_object}m",
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 0),
                                2,
                            )
                    
                    # 特别处理：如果使用了虚拟ID（999），直接从YOLO结果中选择第一个目标
                    if target_track_id == 999 and len(detections) > 0:
                        # 获取第一个检测结果
                        bbox, score, class_id = detections[0]
                        x1, y1, w, h = bbox
                        x2 = x1 + w
                        y2 = y1 + h
                        
                        # 绘制边界框和信息
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            f"直接YOLO跟踪 / 置信度:{score:.2f}",
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )
                    # # 若跟踪目标在当前帧消失：用卡尔曼滤波器预测位置并绘制
                    # if not target_found and target_track_id is not None:
                    #     kf.predict()  # 预测目标下一帧位置
                    #     predicted_x, predicted_y = kf.x[:2]  # 提取预测的x、y位置
                    #     pred_point = int(predicted_x), int(predicted_y)  # 预测位置（整数化）

                    #     # 匹配预测位置附近的跟踪目标（重新关联消失的目标）
                    #     min_dist = float("inf")
                    #     for track in tracked_objects:
                    #         if not track.is_confirmed():
                    #             continue
                    #         ltrb = track.to_ltrb()
                    #         x1, y1, x2, y2 = map(int, ltrb)
                    #         target_center = ((x1 + x2) // 2, (y1 + y2) // 2)

                    #         # 计算预测位置与跟踪目标的距离
                    #         distance = np.linalg.norm(
                    #             np.array(target_center) - np.array(pred_point)
                    #         )
                    #         if distance < min_dist:
                    #             min_dist = distance
                    #             target_track_id = track.track_id  # 重新关联目标ID

                    #     # 绘制预测位置（红色实心圆）
                    #     cv2.circle(
                    #         frame,
                    #         (int(predicted_x), int(predicted_y)),
                    #         10,
                    #         (0, 0, 255),
                    #         -1,
                    #     )
                    #     # 绘制预测位置标签（红色，大字体）
                    #     cv2.putText(
                    #         frame,
                    #         "预测位置",
                    #         (int(predicted_x), int(predicted_y) - 10),
                    #         cv2.FONT_HERSHEY_SIMPLEX,
                    #         0.7,
                    #         (0, 0, 255),
                    #         2,
                    #     )

                    # 显示处理后的图像
                    cv2.imshow("YOLO + DeepSORT", frame)

                    # 将当前帧写入视频文件
                    out.write(frame)

                    # 按下ESC键（ASCII码27）退出循环
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

        except Exception as e:
            print(f"[YOLO] 检测过程出错：{e}")
        time.sleep(0.2)  # 控制检测频率，避免CPU占用过高

    # ❗ 资源释放：关闭视频写入器和OpenCV窗口
    out.release()
    cv2.destroyAllWindows()


def find_nearest_waypoint_index(current_xyz, waypoints):
    """找到当前位置距离最近的航点索引"""
    # 计算当前位置与所有航点的水平距离（忽略z轴，仅x、y方向）
    dists = np.linalg.norm(waypoints[:, :2] - current_xyz[:2], axis=1)
    return np.argmin(dists)  # 返回距离最小的航点索引


def run_path(airsim_path, velocity):
    """沿指定航点路径飞行（用于线程调用）"""
    local_client = airsim.MultirotorClient()
    local_client.confirmConnection()
    print(f"[路径飞行] 共{len(airsim_path)}个航点，开始飞行")
    # 调用AirSim的异步路径飞行接口（阻塞直到飞行完成）
    local_client.moveOnPathAsync(
        airsim_path,                # 航点列表（Vector3r类型）
        velocity=velocity,          # 飞行速度（米/秒）
        drivetrain=airsim.DrivetrainType.ForwardOnly,  # 驱动模式：仅向前
        yaw_mode=airsim.YawMode(False, 0),  # 偏航模式：不锁定偏航角
        vehicle_name="Drone1"       # 无人机名称
    ).join()
    print("[路径飞行] 路径飞行任务完成")


def patrol_loop():
    """无人机巡逻主循环：沿路径巡逻，检测到跟踪目标时暂停"""
    global waypoints, target_track_id
    while True:
        # 👉 若已选择跟踪目标（target_track_id不为None），暂停巡逻并悬停
        if target_track_id is not None:
            print(f"🛑 已选择跟踪目标（ID: {target_track_id}）— 无人机暂停并悬停")
            client.cancelLastTask(vehicle_name="Drone1")  # 取消当前飞行任务
            client.hoverAsync(vehicle_name="Drone1").join()  # 执行悬停
            time.sleep(0.5)  # 每隔0.5秒检查一次跟踪状态
            continue  # 继续循环，等待跟踪取消

        # 找到当前位置距离最近的航点索引
        state = client.getMultirotorState(vehicle_name="Drone1")
        drone_xyz = np.array(
            [
                state.kinematics_estimated.position.x_val,  # 无人机x坐标
                state.kinematics_estimated.position.y_val,  # 无人机y坐标
                state.kinematics_estimated.position.z_val,  # 无人机z坐标
            ]
        )
        start_idx = find_nearest_waypoint_index(drone_xyz, waypoints)

        # 重新排序航点：从最近航点开始，到最后一个航点后回到第一个
        reordered_waypoints = np.concatenate(
            (waypoints[start_idx:], waypoints[:start_idx]), axis=0
        )
        # 👉 若已取消跟踪，将航点转换为AirSim要求的Vector3r类型（叠加安全高度）
        airsim_path = [
            Vector3r(float(x), float(y), float(z) - safety_margin)
            for x, y, z in reordered_waypoints
        ]

        print(
            f"\n[路径飞行] 从第{start_idx}个航点开始，共{len(airsim_path)}个航点"
        )
        # 启动路径飞行线程（避免阻塞主循环）
        path_thread = threading.Thread(target=run_path, args=(airsim_path, velocity))
        path_thread.start()

        # 等待路径飞行完成，期间检查是否需要暂停（选择跟踪目标）
        while path_thread.is_alive():
            # 若选择了跟踪目标，中断路径飞行并悬停
            if target_track_id is not None:
                print("🔁 收到跟踪请求 — 中断路径飞行并悬停")
                client.cancelLastTask(vehicle_name="Drone1")
                client.hoverAsync(vehicle_name="Drone1").join()
                break

            time.sleep(0.1)  # 每隔0.1秒检查一次跟踪状态

        print("\n✅ 路径巡逻完成或已中断 — 重新开始巡逻循环")

        #     # 以下代码用于计算无人机当前朝向（指向下一航点）
        #     # 注：moveOnPathAsync会自动控制朝向，无需手动调整，保留代码供后续扩展
        #     state = client.getMultirotorState(vehicle_name="Drone1")
        #     drone_xyz = np.array(
        #         [
        #             state.kinematics_estimated.position.x_val,
        #             state.kinematics_estimated.position.y_val,
        #             state.kinematics_estimated.position.z_val,
        #         ]
        #     )
        #     # 找到当前最近的航点，确定下一航点
        #     prev_idx = find_nearest_waypoint_index(drone_xyz, waypoints)
        #     if prev_idx + 1 < len(waypoints):
        #         next_xyz = waypoints[prev_idx + 1]
        #     else:
        #         next_xyz = waypoints[0]  # 最后一个航点的下一航点为第一个
        #     # 计算当前位置到下一航点的向量差
        #     diff = next_xyz[:2] - drone_xyz[:2]
        #     # 计算朝向角度（弧度）
        #     heading_rad = np.arctan2(diff[1], diff[0])

        #     time.sleep(0.1)
        # print("\n✅ 全路径巡逻完成，重新开始巡逻")


def cleanup():
    """程序退出时的清理函数（关闭无人机控制、释放资源）"""
    client = airsim.MultirotorClient()
    # 锁定无人机、禁用API控制
    client.armDisarm(False, vehicle_name="Drone1")
    client.enableApiControl(False, vehicle_name="Drone1")
    print(f"[Drone1] 无人机已锁定，API控制已禁用")
    # 发送线程停止信号，关闭YOLO检测线程
    stop_event.set()



# --- AirSim无人机连接与初始化 ---
client = airsim.MultirotorClient()
client.confirmConnection()  # 确认与AirSim的连接
# 启用API控制（True：启用，False：禁用）
client.enableApiControl(True, vehicle_name="Drone1")
# 解锁无人机（True：解锁，False：锁定）
client.armDisarm(True, vehicle_name="Drone1")
# 执行起飞（阻塞直到起飞完成）
client.takeoffAsync(vehicle_name="Drone1").join()
print("[启动] 无人机连接成功并完成起飞")


# --- 从fence_path.csv读取航点数据（x, y, z）并转换 ---
# 读取CSV文件（表头为name, x, y, z）
df = pd.read_csv("trajectory.csv", header=0, names=["timestamp", "x", "y", "z"])
# 提取x、y、z列作为航点数据
waypoints = df[["x", "y", "z"]].values
# AirSim中z轴负向为上方，需反转z值符号
#waypoints[:, 2] = -waypoints[:, 2]
# 单位转换：CSV中为厘米（cm），转为米（m）
#waypoints = waypoints / 100
print(f"[加载] 航点数据示例（前10个）:\n{waypoints[:10]}")
print(f"[加载] 航点数据是否包含NaN值：{np.isnan(waypoints).any()}")



# 飞行参数设置
obstacle_dist = 0.8  # 障碍物判定距离（米）
fov_deg = 90         # 障碍物检测角度（前方90度）
safety_margin = 20   # 相对于边界的安全高度（米，飞行时高于边界该高度）
velocity = 2         # 飞行速度（米/秒）

# --- 无人机 teleport到起点并悬停 ---
start = waypoints[0]  # 第一个航点作为起点
start_z = start[2] - safety_margin  # 起点z坐标（叠加安全高度）
print(f"🔸  teleport到起点：x={start[0]:.2f}, y={start[1]:.2f}, z={start_z:.2f}")
# 设置无人机姿态（位置+朝向）
client.simSetVehiclePose(
    Pose(
        Vector3r(float(start[0]), float(start[1]), float(start_z)),  # 起点位置
        to_quaternion(0, 0, 0)  # 朝向（欧拉角0,0,0 → 四元数）
    ),
    ignore_collision=True,  # 忽略碰撞（teleport时）
    vehicle_name="Drone1"
)
time.sleep(1)  # 等待teleport完成
# 执行悬停（确保无人机稳定在起点）
client.hoverAsync(vehicle_name="Drone1").join()
print("[启动] 无人机已移动到起点并完成悬停")

# --- 启动YOLO目标检测线程 ---
yolo_thread = threading.Thread(target=yolo_worker_ultralytics, daemon=True)
yolo_thread.start()
print("[YOLO] 目标检测线程已启动")




# 注册程序退出时的清理函数（无论正常退出还是异常退出）
atexit.register(cleanup)

# --- 启动无人机巡逻 ---
try:
    print("[巡逻] 无人机单机组巡逻系统启动")
    patrol_loop()  # 进入巡逻主循环
except KeyboardInterrupt:
    print("🛑 用户手动中断程序")
    cleanup()  # 执行清理操作
    print("[退出] 程序已正常退出")
    exit(0)  # 正常退出