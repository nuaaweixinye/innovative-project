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

# === 모델 세팅 ===
model = YOLO(
    "E:/AirSim_vscode/weights/yolov11m_myeong_best_airsim100.pt"
)  # yolov11m_myeong_best


# 수평 거리 계산 함수
# 카메라 하향각과 이미지속 객체 위치로 삼각함수(tan)을 이용해서 수평거리 추정
# -------------------------
# def estimate_horizontal_distance_from_object(
#     drone_altitude_m,
#     camera_pitch_deg,
#     object_center_y_px,
#     image_height_px,
#     vertical_fov_deg,
# ):
#     deg_per_pixel = vertical_fov_deg / image_height_px
#     offset_pixel = object_center_y_px - (image_height_px / 2)
#     offset_angle_deg = offset_pixel * deg_per_pixel
#     total_angle_deg = camera_pitch_deg + offset_angle_deg
#     total_angle_rad = math.radians(total_angle_deg)
#     if total_angle_rad <= 0:
#         return None  # or float('inf')
#     horizontal_distance_m = drone_altitude_m / math.tan(total_angle_rad)
#     return round(horizontal_distance_m, 2)


# 아래는 수평까지 고려한 함수
def estimate_horizontal_distance_from_object(
    drone_altitude_m,
    camera_pitch_deg,
    camera_yaw_deg,
    object_center_x_px,
    object_center_y_px,
    image_width_px,
    image_height_px,
    horizontal_fov_deg,
    vertical_fov_deg,
):
    # FOV 기준으로 각 픽셀당 각도 계산
    deg_per_px_y = vertical_fov_deg / image_height_px
    deg_per_px_x = horizontal_fov_deg / image_width_px

    # 이미지 중앙으로부터의 오프셋
    offset_y_px = object_center_y_px - (image_height_px / 2)
    offset_x_px = object_center_x_px - (image_width_px / 2)

    # 해당 픽셀의 pitch, yaw 오프셋
    offset_pitch_deg = offset_y_px * deg_per_px_y
    offset_yaw_deg = offset_x_px * deg_per_px_x

    # 총 pitch, yaw 각도
    total_pitch_deg = camera_pitch_deg + offset_pitch_deg
    total_yaw_deg = camera_yaw_deg + offset_yaw_deg

    # pitch → 수평 거리
    total_pitch_rad = math.radians(total_pitch_deg)
    if total_pitch_rad <= 0:
        # raise ValueError("총 pitch가 0도 이하입니다. 카메라 각도를 확인하세요.")
        return None

    base_distance = drone_altitude_m / math.tan(total_pitch_rad)

    # yaw 각도 보정 (측면으로 갈수록 거리 증가)
    yaw_correction = math.cos(math.radians(total_yaw_deg))
    if yaw_correction <= 0.01:  # 너무 측면인 경우 방지
        yaw_correction = 0.01

    corrected_distance = base_distance / yaw_correction
    return round(corrected_distance, 2)


# 추적할 특정 ID (초기값: None)
target_track_id = None
# 마우스 클릭으로 특정 사람 선택
clicked_point = None  # 사용자가 클릭한 좌표 저장


### "한 번 더 클릭하면 추적 해제" 기능을 추가
def select_target(event, x, y, flags, param):
    global clicked_point, target_track_id
    if event == cv2.EVENT_LBUTTONDOWN:
        if target_track_id is not None:
            # 👉 이미 추적 중이면 해제
            print(f"❌ 추적 해제 (ID {target_track_id})")
            target_track_id = None
            clicked_point = None
        else:
            # 👉 추적 대상이 없으면 선택 시작
            clicked_point = (x, y)


stop_event = threading.Event()


def yolo_worker_ultralytics():
    global clicked_point
    print("[YOLO] Ultralytics YOLO 객체 탐지 스레드 시작")
    yolo_client = airsim.MultirotorClient()
    yolo_client.confirmConnection()

    # ✅ 비디오 저장 설정
    output_path = "output_video/airsim드론_주둔지경계.mp4"
    fps = 1
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, fps, (1280, 720))

    while not stop_event.is_set():
        try:
            ## 기존 코드 ###
            response = yolo_client.simGetImage(
                "front_center", airsim.ImageType.Scene, vehicle_name="Drone1"
            )

            if response:
                # Decode RGB
                img_array = np.frombuffer(response, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                height, width, _ = frame.shape

                ### 거리 함수 파라미터
                state = yolo_client.getMultirotorState(vehicle_name="Drone1")
                DRONE_ALTITUDE = -state.kinematics_estimated.position.z_val
                # 거리 계산에 필요한 카메라 정보 자동 추출
                cam_info = yolo_client.simGetCameraInfo("front_center")
                pitch_rad, _, yaw_rad = airsim.to_eularian_angles(
                    cam_info.pose.orientation
                )

                # 드론 기준, 카메라가 아래를 보면 음수, 위를 보면 양수 → 삼각법 수식과 방향이 반대
                # math.degrees값 반전 필요
                CAMERA_YAW_DEG = math.degrees(yaw_rad)
                CAMERA_PITCH_DEG = -math.degrees(pitch_rad)  # pitch angle in degrees
                VERTICAL_FOV_DEG = cam_info.fov  # vertical FOV in degrees
                IMAGE_WIDTH_PX = width
                IMAGE_HEIGHT_PX = height
                HORIZONTAL_FOV_DEG = 2 * math.degrees(
                    math.atan(
                        math.tan(math.radians(VERTICAL_FOV_DEG / 2))
                        * (IMAGE_WIDTH_PX / IMAGE_HEIGHT_PX)
                    )
                )

                FOCAL_LENGTH_PX = 800  # 예시값, 실제 캘리브레이션 필요

                # DeepSORT 트래커 초기화
                tracker = DeepSort(max_age=30)

                # Kalman Filter 초기화
                kf = KalmanFilter(dim_x=4, dim_z=2)  # 4D 상태(위치+속도), 2D 관측(위치)
                kf.F = np.array(
                    [
                        [1, 0, 1, 0],  # x = x + vx    / 상태 전이 행렬 사용
                        [0, 1, 0, 1],  # y = y + vy
                        [0, 0, 1, 0],  # vx 유지
                        [0, 0, 0, 1],
                    ]
                )  # vy 유지
                kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # x만 관측  # y만 관측
                kf.P *= 1000  # 초기 불확실성
                kf.R *= 10  # 측정 노이즈

                cv2.namedWindow("YOLO + DeepSORT Tracking")
                cv2.setMouseCallback("YOLO + DeepSORT Tracking", select_target)

                if frame is not None:
                    # ⏩ Ultralytics 모델 추론
                    results = model(frame)[0]  # 첫 번째 결과
                    print("[YOLO] 박스 개수:", len(results.boxes))
                    detections = []

                    # ✅ 추적할 클래스 지정
                    track_classes = [0, 2, 7]  # person, car, truck

                    # ✅ 객체 정보 저장 리스트 초기화
                    detected_objects_info = []
                    class_counter = defaultdict(int)  # ✅ 객체별 수를 저장하는 딕셔너리

                    ### 테스트용 ###
                    for result in results.boxes.data:
                        x1, y1, x2, y2, score, class_id = result.tolist()
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        class_id = int(class_id)
                        label = (
                            model.names[class_id]
                            if class_id in model.names
                            else "Unknown"
                        )
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
                            continue  # 거리 추정 불가한 객체는 스킵하거나 표시 제외

                        # 👉 탐지된 객체 저장
                        detected_objects_info.append(
                            {
                                "label": label,
                                "distance": distance_to_object,
                                "score": round(score, 2),
                            }
                        )
                        # ✅ 개수 세기
                        class_counter[label] += 1

                        if class_id in track_classes and score > 0.5:
                            detections.append(
                                ([x1, y1, x2 - x1, y2 - y1], score, class_id)
                            )
                    # ✅ 탐지된 객체 수 및 정보 출력
                    print(f"[YOLO] ✅ 탐지된 객체 수: {len(detected_objects_info)}")
                    for label, count in class_counter.items():
                        print(f" - {label}: {count}개")

                    tracked_objects = tracker.update_tracks(detections, frame=frame)
                    print(f"tracked_objects : {tracked_objects}")

                    global target_track_id
                    if clicked_point is not None and target_track_id is None:
                        print(f"target_track_id : {target_track_id}")
                        print(f"clicked_point : {clicked_point}")
                        min_dist = float("inf")
                        for track in tracked_objects:
                            ltrb = track.to_ltrb()
                            x1, y1, x2, y2 = map(int, ltrb)
                            person_center = ((x1 + x2) // 2, (y1 + y2) // 2)

                            # 유클리드 거리 계산
                            distance = np.linalg.norm(
                                np.array(person_center) - np.array(clicked_point)
                            )
                            if distance < min_dist:
                                min_dist = distance
                                target_track_id = track.track_id

                        print(f"🔍 선택된 추적 대상 ID: {target_track_id}")
                        clicked_point = None  # 한 번 선택하면 더 이상 클릭 안 받음

                    target_found = (
                        False  # 추적 대상이 현재 프레임에서 감지되었는지 체크
                    )

                    for track in tracked_objects:
                        track_id = track.track_id
                        print(f"track_id : {track_id}")
                        ltrb = track.to_ltrb()  # 좌표 변환
                        x1, y1, x2, y2 = map(int, ltrb)
                        person_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])

                        if target_track_id is None:
                            # 아직 객체 선택 전 → 모든 사람에게 박스 표시
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
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
                            # 선택된 대상만 추적
                            target_found = True
                            kf.update(person_center)

                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(
                                frame,
                                f"Target ID {track_id} / Distance:{distance_to_object}m",
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 0),
                                2,
                            )

                    # # 만약 추적 대상이 사라졌다면 → Kalman Filter를 이용해 예상 위치 그리기
                    # if not target_found and target_track_id is not None:
                    #     kf.predict()  # 객체의 이동 예측
                    #     predicted_x, predicted_y = kf.x[:2]  # 예측된 위치
                    #     pred_point = int(predicted_x), int(predicted_y)

                    #     min_dist = float("inf")
                    #     for track in tracked_objects:
                    #         ltrb = track.to_ltrb()
                    #         x1, y1, x2, y2 = map(int, ltrb)
                    #         person_center = ((x1 + x2) // 2, (y1 + y2) // 2)

                    #         # 유클리드 거리 계산
                    #         distance = np.linalg.norm(
                    #             np.array(person_center) - np.array(pred_point)
                    #         )
                    #         if distance < min_dist:
                    #             min_dist = distance
                    #             target_track_id = (
                    #                 track.track_id
                    #             )  # 가장 가까운 사람의 ID를 저장

                    #     cv2.circle(
                    #         frame,
                    #         (int(predicted_x), int(predicted_y)),
                    #         10,
                    #         (0, 0, 255),
                    #         -1,
                    #     )  # 빨간 점으로 예측 위치 표시
                    #     cv2.putText(
                    #         frame,
                    #         "Predicted Position",
                    #         (int(predicted_x), int(predicted_y) - 10),
                    #         cv2.FONT_HERSHEY_SIMPLEX,
                    #         0.7,
                    #         (0, 0, 255),
                    #         2,
                    #     )

                    # 화면 출력
                    cv2.imshow("YOLO + DeepSORT Tracking", frame)

                    out.write(frame)

                    # ESC 키 종료
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

        except Exception as e:
            print(f"[YOLO] 오류: {e}")
        time.sleep(0.2)

    out.release()  # ❗ 영상 파일 닫기
    cv2.destroyAllWindows()


def find_nearest_waypoint_index(current_xyz, waypoints):
    """현위치에서 가장 가까운 경유지 인덱스 찾기"""
    dists = np.linalg.norm(waypoints[:, :2] - current_xyz[:2], axis=1)
    return np.argmin(dists)


def run_path(airsim_path, velocity):
    """경로 전체를 moveOnPathAsync로 이동 (스레드용)"""
    local_client = airsim.MultirotorClient()
    local_client.confirmConnection()
    print(f"[moveOnPath] {len(airsim_path)}개 경유지로 비행 시작")
    local_client.moveOnPathAsync(
        airsim_path,
        velocity=velocity,
        drivetrain=airsim.DrivetrainType.ForwardOnly,
        yaw_mode=airsim.YawMode(False, 0),
        vehicle_name="Drone1",
    ).join()
    print("[moveOnPath] moveOnPathAsync 종료")


# --- AirSim 드론 연결 ---
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name="Drone1")
client.armDisarm(True, vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone1").join()
print("[시작] 드론 연결 및 이륙 완료")

# --- fence_path.csv에서 x, y, z 읽기 및 변환 ---
df = pd.read_csv("fence_path.csv", header=0, names=["name", "x", "y", "z"])
waypoints = df[["x", "y", "z"]].values
waypoints[:, 2] = -waypoints[:, 2]  # z값 부호 반전 (AirSim은 z가 음수일수록 위)
waypoints = waypoints / 100  # (cm → m 변환)
print(f"[로드] 경유지 샘플:\n{waypoints[:10]}")
print(f"[로드] NaN 포함 여부: {np.isnan(waypoints).any()}")

obstacle_dist = 0.8  # 장애물 판정 거리(m)
fov_deg = 90  # 장애물 감지 각도(전방 120도)
safety_margin = 20  # 펜스에서 추가로 몇 m 위를 날지
velocity = 2

# --- 출발점 텔레포트 및 이륙 ---
start = waypoints[0]
start_z = start[2] - safety_margin
print(f"🔸 출발점으로 텔레포트: x={start[0]:.2f}, y={start[1]:.2f}, z={start_z:.2f}")
client.simSetVehiclePose(
    Pose(
        Vector3r(float(start[0]), float(start[1]), float(start_z)),
        to_quaternion(0, 0, 0),
    ),
    ignore_collision=True,
    vehicle_name="Drone1",
)
time.sleep(1)
client.hoverAsync(vehicle_name="Drone1").join()
print("[시작] 드론 위치 이동 및 호버 완료")

# --- YOLO 스레드 시작 ---
yolo_thread = threading.Thread(target=yolo_worker_ultralytics, daemon=True)
yolo_thread.start()
print("[YOLO] 탐지 스레드 시작")


def patrol_loop():
    """전체 경로를 moveOnPathAsync로 순찰, 장애물 탐지 및 회피"""
    global waypoints, target_track_id
    while True:
        # 👉 객체를 클릭하여 target_track_id가 존재하면 일시 정지
        if target_track_id is not None:
            print(f"🛑 객체(ID: {target_track_id}) 선택됨 — 드론 Hover 및 이동 중단")
            client.cancelLastTask(vehicle_name="Drone1")  # 이동 명령 취소
            client.hoverAsync(vehicle_name="Drone1").join()
            time.sleep(0.5)
            continue  # 계속 대기 (클릭 해제 전까지)

        # 현재 위치 기준으로 가장 가까운 waypoint부터 경로 시작
        state = client.getMultirotorState(vehicle_name="Drone1")
        drone_xyz = np.array(
            [
                state.kinematics_estimated.position.x_val,
                state.kinematics_estimated.position.y_val,
                state.kinematics_estimated.position.z_val,
            ]
        )
        start_idx = find_nearest_waypoint_index(drone_xyz, waypoints)

        # 경로를 시작 인덱스부터 재구성
        reordered_waypoints = np.concatenate(
            (waypoints[start_idx:], waypoints[:start_idx]), axis=0
        )
        # 👉 클릭 해제된 경우 경로 순찰 시작
        airsim_path = [
            Vector3r(float(x), float(y), float(z) - safety_margin)
            for x, y, z in reordered_waypoints
        ]

        print(
            f"\n[moveOnPathAsync] {len(airsim_path)}개 지점으로 순찰 시작! (from idx {start_idx})"
        )
        path_thread = threading.Thread(target=run_path, args=(airsim_path, velocity))
        path_thread.start()

        while path_thread.is_alive():
            # 클릭 시 실시간 중단
            if target_track_id is not None:
                print("🔁 중간 정지 요청됨 — moveOnPathAsync 중단 및 Hover")
                client.cancelLastTask(vehicle_name="Drone1")
                client.hoverAsync(vehicle_name="Drone1").join()
                break

            time.sleep(0.1)

        print("\n✅ 전체 경로 순찰 완료 또는 중단됨 — 루프 재시작")

        #     # 아래 코드는 드론의 현재 위치를 실시간으로 받아와서,
        #     # 다음 waypoint 방향을 계산하고 heading_rad (진행 방향 각도)를 구하는 부분
        #     # 하지만 moveOnPathAsync()가 경로를 자동으로 따라가기 때문에,
        #     # 별도로 heading 값을 직접 구해서 조작할 필요는 없음
        #     # heading_rad 값은 현재 코드에서 아무 데서도 사용되지 않음
        #     state = client.getMultirotorState(vehicle_name="Drone1")
        #     drone_xyz = np.array(
        #         [
        #             state.kinematics_estimated.position.x_val,
        #             state.kinematics_estimated.position.y_val,
        #             state.kinematics_estimated.position.z_val,
        #         ]
        #     )
        #     # 만약 향후 "다음 방향 각도"를 HUD에 표시하거나 드론 방향 제어에 사용할 생각이 있다면,
        #     # 위 부분만 남겨도 됨!!!

        #     prev_idx = find_nearest_waypoint_index(drone_xyz, waypoints)
        #     if prev_idx + 1 < len(waypoints):
        #         next_xyz = waypoints[prev_idx + 1]
        #     else:
        #         next_xyz = waypoints[0]
        #     diff = next_xyz[:2] - drone_xyz[:2]
        #     heading_rad = np.arctan2(diff[1], diff[0])

        #     time.sleep(0.1)
        # print("\n✅ 전체 경로 순찰 완료, 다시 시작")


def cleanup():
    client = airsim.MultirotorClient()
    # client.confirmConnection()
    client.armDisarm(False, vehicle_name="Drone1")
    client.enableApiControl(False, vehicle_name="Drone1")
    print(f"[Drone1] 종료 완료")


atexit.register(cleanup)

# --- 패트롤 시작 ---
try:
    print("[순찰] 드론 1대 순찰 시스템 시작")
    patrol_loop()
except KeyboardInterrupt:
    print("🛑 사용자 종료 요청")
    cleanup()
