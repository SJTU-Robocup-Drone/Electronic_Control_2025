#!/home/amov/miniconda3/envs/yolo/bin/python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import torch
import numpy as np
# import pyrealsense2 as rs
import time
import threading
import cv2
import math
# import tensorrt
from nav_msgs.msg import Odometry
import threading
from utils.augmentations import letterbox
from utils.general import non_max_suppression
from tensorrt_model_loader import TensorRTModel
from collections import deque, defaultdict, Counter  # 轨迹与类别历史

# ==================== 可调参数 ====================
vision_rate = 120            # 视觉处理fps（提示性，不强制sleep）
enable_viz = False           # 是否显示窗口，关闭可提高运行速度
log_every_n = 30             # 每N帧打印一次检测日志，减轻IO瓶颈
lock_min_observations = 3    # 初始阶段过滤所需的最少观测次数（越小越快锁定）
conf_thres_detect = 0.8      # 发布与追踪使用的置信度阈值
# =================================================

# Vision state control
scanning_active = False
detection_thread = None
stop_event = threading.Event()

# Initialize sliding-average buffer
items = [
    ["bridge", 0, []], ["bunker", 0, []], ["car", 0, []],
    ["Helicopter", 0, []], ["tank", 0, []], ["tent", 0, []], ["red", 0, []]
]

# Class names (TensorRT engine不包含)
names = {0: 'bridge', 1: 'bunker', 2: 'car', 3: 'Helicopter', 4: 'tank', 5: 'tent', 6: 'red'}

camera_matrix = np.array([
    [432.9524, 0, 417.1712],  # fx, 0, cx
    [0, 426.8039, 315.5234],  # 0, fy, cy
    [0, 0, 1.0]
], dtype=np.float32)

desired_width = 640
desired_height = 480
desired_fps = 120
cap = None

# Publishers
detection_pub = None
random_pub = None
odom_pos = None

# TensorRT
engine_path = '/home/amov/board_ws/src/board_ros/scripts/best_0909.engine'
rospy.loginfo("[INFO] Loading TensorRT model...")
model = TensorRTModel(engine_path)
rospy.loginfo("[INFO] TensorRT model loaded successfully.")

latest_frame = None
frame_lock = threading.Lock()

# --------------------------
# 2D 卡尔曼追踪（原有）
# --------------------------
tank_tracking_active = False
tank_kf = None  # cv2.KalmanFilter(4,2): [x,y,vx,vy] / [x,y]
tank_traj_world = deque(maxlen=200)  # 世界坐标轨迹（米）
tank_traj_pix = deque(maxlen=200)    # 像素轨迹（绘制用）
tank_last_time = None
last_tank_detection_time = None
tank_lost_timeout = 2.0  # s

# --------------------------
# 类别历史过滤（仅初始阶段）
# --------------------------
track_class_history = defaultdict(list)  # track_id -> [class_name, ...]
track_confirmed_class = {}               # track_id -> confirmed_class（由过滤逻辑确认）
track_locked_class = {}                  # track_id -> locked_class（锁定后不再过滤）
track_seen_count = defaultdict(int)      # track_id -> 已观测次数（仅非red/未锁定）

def filter_class_by_history(track_id, current_class):
    """基于历史信息过滤类别，减少误识别。red 不生效。"""
    global track_class_history, track_confirmed_class

    if current_class == "red":
        return current_class

    track_class_history[track_id].append(current_class)

    if len(track_class_history[track_id]) >= 3:
        recent_classes = list(track_class_history[track_id])[-5:]  # 最近5次
        class_counter = Counter(recent_classes)

        most_common_class, most_common_count = class_counter.most_common(1)[0]
        if most_common_count / len(recent_classes) >= 0.6:
            track_confirmed_class[track_id] = most_common_class
            return most_common_class
        else:
            if track_id in track_confirmed_class:
                return track_confirmed_class[track_id]

    return current_class

# --------------------------
# 2D KF 工具（含安全predict）
# --------------------------
def init_tank_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)
    kf.measurementMatrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ], dtype=np.float32)
    
    # 优化的噪声参数 - 与usb_filter.py保持一致
    Q_diag = np.array([5e-3, 5e-3, 1e-2, 1e-2], dtype=np.float32)  # [x, y, vx, vy]
    kf.processNoiseCov = np.diag(Q_diag)
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 2e-2
    
    # 优化的初始协方差矩阵
    P_diag = np.array([0.1, 0.1, 1.0, 1.0], dtype=np.float32)  # [x, y, vx, vy]
    kf.errorCovPost = np.diag(P_diag)
    return kf

def update_kf_dt(kf, dt):
    dt = max(1e-3, float(dt))
    A = kf.transitionMatrix.copy()
    A[0, 2] = dt
    A[1, 3] = dt
    kf.transitionMatrix = A
    return dt

def safe_kf_predict(kf):
    try:
        return kf.predict()
    except Exception:
        A = np.array(kf.transitionMatrix, dtype=np.float32)
        x_post = np.array(kf.statePost, dtype=np.float32)
        P_post = np.array(kf.errorCovPost, dtype=np.float32)
        Q = np.array(kf.processNoiseCov, dtype=np.float32)
        x_prior = A @ x_post
        P_prior = A @ P_post @ A.T + Q
        kf.statePre = x_prior
        kf.errorCovPre = P_prior
        return x_prior

def start_tank_tracker(x_world, y_world, t_now):
    global tank_kf, tank_tracking_active, tank_traj_world, tank_traj_pix, tank_last_time, last_tank_detection_time
    tank_kf = init_tank_kalman()
    tank_kf.statePost = np.array([[x_world], [y_world], [0.0], [0.0]], dtype=np.float32)
    tank_tracking_active = True
    tank_traj_world.clear()
    tank_traj_pix.clear()
    tank_traj_world.append((float(x_world), float(y_world)))
    tank_last_time = float(t_now)
    last_tank_detection_time = float(t_now)
    rospy.loginfo("[TRACK] Tank tracker started at (%.3f, %.3f)", x_world, y_world)

def update_tank_tracker(x_world, y_world, t_now):
    global tank_kf, tank_last_time, last_tank_detection_time
    if tank_kf is None:
        start_tank_tracker(x_world, y_world, t_now)
        return (x_world, y_world, 0.0, 0.0)
    dt = update_kf_dt(tank_kf, float(t_now) - float(tank_last_time))
    safe_kf_predict(tank_kf)
    meas = np.array([[x_world], [y_world]], dtype=np.float32)
    est = tank_kf.correct(meas)
    x_est, y_est = float(est[0, 0]), float(est[1, 0])
    vx_est, vy_est = float(est[2, 0]), float(est[3, 0])
    tank_last_time = float(t_now)
    last_tank_detection_time = float(t_now)
    tank_traj_world.append((x_est, y_est))
    return (x_est, y_est, vx_est, vy_est)

def predict_tank_tracker(t_now):
    global tank_kf, tank_last_time
    if tank_kf is None:
        return None
    dt = update_kf_dt(tank_kf, float(t_now) - float(tank_last_time))
    pred = safe_kf_predict(tank_kf)
    x_pred, y_pred = float(pred[0, 0]), float(pred[1, 0])
    vx_pred, vy_pred = float(pred[2, 0]), float(pred[3, 0])
    tank_last_time = float(t_now)
    tank_traj_world.append((x_pred, y_pred))
    return (x_pred, y_pred, vx_pred, vy_pred)

def stop_tank_tracker():
    global tank_tracking_active, tank_kf, tank_last_time, last_tank_detection_time
    if tank_tracking_active:
        rospy.loginfo("[TRACK] Tank tracker stopped.")
    tank_tracking_active = False
    tank_kf = None
    tank_last_time = None
    last_tank_detection_time = None

def world_to_pixel(x, y, h, fx, fy, cx, cy):
    if h is None or h == 0:
        return None
    u = int(x * fx / h + cx)
    v = int(y * fy / h + cy)
    return (u, v)

def draw_tank_overlay(frame, speed_ms, measured_uv=None, estimated_uv=None, traj_uv=None, lost=False):
    if traj_uv and len(traj_uv) > 1:
        for i in range(1, len(traj_uv)):
            if traj_uv[i - 1] is None or traj_uv[i] is None:
                continue
            cv2.line(frame, traj_uv[i - 1], traj_uv[i], (255, 0, 0), 2)
    if measured_uv is not None:
        cv2.circle(frame, measured_uv, 4, (0, 0, 255), -1)
    if estimated_uv is not None:
        color = (0, 255, 0) if not lost else (0, 255, 255)
        cv2.circle(frame, estimated_uv, 6, color, 2)
    cv2.putText(frame, f"Tank speed: {speed_ms:.2f} m/s", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

def grab_thread():
    global latest_frame, grab_running, cap
    grab_running = True
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while grab_running:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame.copy()
        else:
            grab_running = False
            cap.release()
            rospy.loginfo("Camera read failed")

def detect_red_cross(frame):
    """Detect red-cross shape in the image and return its center pixel coordinate."""
    h, w = frame.shape[:2]
    # rospy.loginfo("TODELETE")
    h_min, h_max = 7, 10
    s_min, s_max = 110, 255
    v_min, v_max = 60, 255
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([h_min, s_min, v_min])
    upper_red1 = np.array([h_max, s_max, v_max])
    lower_red2 = np.array([max(0, 170 - (10 - h_min)), s_min, v_min]) if h_min < 10 else np.array([170, s_min, v_min])
    upper_red2 = np.array([min(179, 180 + (h_max - 170)), s_max, v_max]) if h_max > 170 else np.array([180, s_max, v_max])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    closed_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel_open)
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cross_center = None
    max_area = 0
    for cnt in contours:
        min_area = (w * h) / 2000
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        hull = cv2.convexHull(approx)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        solidity = float(area) / hull_area
        if 8 <= len(hull) <= 12 and 0.6 <= solidity <= 0.9:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if area > max_area:
                    max_area = area
                    cross_center = (cX, cY)
    # rospy.loginfo("TODELETE1")
    return cross_center

def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = ((img1_shape[1] - img0_shape[1] * gain) / 2,
               (img1_shape[0] - img0_shape[0] * gain) / 2)
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]
    coords[:, [0, 2]] -= pad[0]
    coords[:, [1, 3]] -= pad[1]
    coords[:, :4] /= gain
    coords[:, :4] = coords[:, :4].clamp(min=0)
    return coords

def detect_targets():
    global detection_pub, odom_pos, random_pub, grab_running
    global tank_tracking_active, last_tank_detection_time
    global track_locked_class, track_seen_count, track_class_history, track_confirmed_class

    frame_count = 0

    while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():
        with frame_lock:
            if latest_frame is None:
                rospy.sleep(0.001)
                continue
            color_image = latest_frame.copy()

        # 图像预处理
        img_letterboxed = letterbox(
            color_image, new_shape=(640, 640), color=(114, 114, 114),
            auto=False, scaleFill=False, scaleup=True
        )[0]
        img = img_letterboxed[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img).astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        # TensorRT 推理
        output = model.infer(img)
        pred = torch.from_numpy(output).reshape(1, -1, 12)
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)[0]

        # 帧内常量
        cam_h = (odom_pos.z + 0.26) if odom_pos else 0.0
        fx = camera_matrix[0, 0]; fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]; cy = camera_matrix[1, 2]

        # 本帧tank最佳量测
        best_tank = None

        # 后处理
        if pred is not None and len(pred):
            pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()

            for *xyxy, conf, cls in pred:
                conf_val = float(conf)
                if conf_val < conf_thres_detect:
                    continue

                xmin, ymin, xmax, ymax = map(int, xyxy)
                x_found = (xmin + xmax) // 2
                y_found = (ymin + ymax) // 2

                x = (x_found - cx) / fx * cam_h
                y = (y_found - cy) / fy * cam_h
                z = cam_h

                class_id = int(cls)
                raw_class_name = names[class_id]
                track_id = f"grid_{int(x_found / 50)}_{int(y_found / 50)}"

                # red: 永不过滤；tank: 若进入追踪模式，也不再过滤
                if raw_class_name == "red":
                    publish_class = "red"
                    filtered_class_name = "red"
                elif raw_class_name == "tank" and tank_tracking_active:
                    publish_class = "tank"
                    filtered_class_name = "tank"
                else:
                    # 其余类别（含未进入追踪的tank）仅“初始阶段”过滤，达到阈值后锁定
                    if track_id in track_locked_class:
                        publish_class = track_locked_class[track_id]
                        filtered_class_name = publish_class
                    else:
                        track_seen_count[track_id] += 1
                        filtered_once = filter_class_by_history(track_id, raw_class_name)
                        publish_class = filtered_once
                        filtered_class_name = filtered_once
                        if track_seen_count[track_id] >= lock_min_observations:
                            track_locked_class[track_id] = filtered_once
                            # 清理历史，释放后续开销
                            if track_id in track_class_history:
                                del track_class_history[track_id]
                            if track_id in track_confirmed_class:
                                del track_confirmed_class[track_id]
                            if track_id in track_seen_count:
                                del track_seen_count[track_id]

                # 追踪选择：使用“发布用类别”
                if publish_class == "tank":
                    if (best_tank is None) or (conf_val > best_tank['conf']):
                        best_tank = {
                            'x': float(x), 'y': float(y),
                            'u': int(x_found), 'v': int(y_found),
                            'conf': conf_val
                        }

                # 发布
                detection_msg = PointStamped()
                detection_msg.header.stamp = rospy.Time.now()
                detection_msg.header.frame_id = publish_class

                if publish_class == "red":
                    cross_center = detect_red_cross(color_image)
                    if cross_center is not None:
                        x_new, y_new = cross_center
                        x_new = (x_new - cx) / fx * cam_h
                        y_new = (y_new - cy) / fy * cam_h
                        if math.hypot(x - x_new, y - y_new) <= 0.3:
                            detection_msg.point.x = -x_new
                            detection_msg.point.y = -y_new
                            detection_msg.point.z = z
                            random_pub.publish(detection_msg)
                else:
                    detection_msg.point.x = -x
                    detection_msg.point.y = -y
                    detection_msg.point.z = z
                    detection_pub.publish(detection_msg)

                # 降低日志频率
                frame_count += 1
                if frame_count % log_every_n == 0:
                    rospy.loginfo(
                        f"YOLO detected: filtered={filtered_class_name}, publish={publish_class}, "
                        f"coords: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, conf: {conf_val:.2f}"
                    )

        # --------------------------
        # Tank 追踪与绘制
        # --------------------------
        t_now = time.time()
        est_uv = None
        meas_uv = None
        speed_ms = 0.0
        lost = False

        if best_tank is not None:
            meas_uv = (best_tank['u'], best_tank['v'])
            if not tank_tracking_active:
                start_tank_tracker(best_tank['x'], best_tank['y'], t_now)
                x_e, y_e, vx_e, vy_e = best_tank['x'], best_tank['y'], 0.0, 0.0
            else:
                x_e, y_e, vx_e, vy_e = update_tank_tracker(best_tank['x'], best_tank['y'], t_now)

            speed_ms = math.hypot(vx_e, vy_e)
            uv = world_to_pixel(x_e, y_e, cam_h, fx, fy, cx, cy)
            if uv is not None:
                est_uv = uv
                tank_traj_pix.append(uv)
            else:
                est_uv = meas_uv
                tank_traj_pix.append(meas_uv)
        else:
            if tank_tracking_active:
                pred = predict_tank_tracker(t_now)
                if pred is not None:
                    x_p, y_p, vx_p, vy_p = pred
                    speed_ms = math.hypot(vx_p, vy_p)
                    uv = world_to_pixel(x_p, y_p, cam_h, fx, fy, cx, cy)
                    est_uv = uv
                    if uv is not None:
                        tank_traj_pix.append(uv)
                    lost = True
                if last_tank_detection_time is not None and (t_now - last_tank_detection_time) > tank_lost_timeout:
                    stop_tank_tracker()

        # 可视化（可关闭以提高速度）
        if enable_viz and tank_tracking_active:
            draw_tank_overlay(
                color_image,
                speed_ms=speed_ms,
                measured_uv=meas_uv,
                estimated_uv=est_uv,
                traj_uv=list(tank_traj_pix),
                lost=lost
            )
            try:
                cv2.imshow("Tank Tracking", color_image)
                cv2.waitKey(1)
            except Exception:
                pass

    cv2.destroyAllWindows()
    rospy.on_shutdown(shutdown_hook)
    rospy.loginfo("[INFO] Detection thread exited safely.")

def vision_state_callback(msg):
    global scanning_active, detection_thread, stop_event, global_running
    if msg.data and not scanning_active:
        rospy.loginfo("[INFO] Received vision start command, beginning scan...")
        scanning_active = True
        stop_event.clear()
        detection_thread = threading.Thread(target=detect_targets)
        detection_thread.daemon = True
        detection_thread.start()

def odom_callback(msg):
    global odom_pos
    odom_pos = msg.pose.pose.position

def shutdown_hook():
    rospy.loginfo("Shutting down TensorRT model...")
    model.cleanup()

if __name__ == "__main__":
    rospy.init_node("object_detector")
    threading.Thread(target=grab_thread, daemon=True).start()

    global_running = True

    detection_pub = rospy.Publisher("/detection_results", PointStamped, queue_size=20)
    random_pub = rospy.Publisher("/random_target", PointStamped, queue_size=5)
    rospy.Subscriber("/vision_state", Bool, vision_state_callback)
    rospy.Subscriber("/odom_high_freq", Odometry, odom_callback)
    rospy.loginfo("[INFO] Vision node started, waiting for /vision_state command...]")
    rospy.loginfo("[INFO] Vision node started, auto-starting scan...]")

    scanning_active = True
    stop_event = threading.Event()
    detection_thread = threading.Thread(target=detect_targets)
    detection_thread.daemon = True
    detection_thread.start()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        if scanning_active:
            stop_event.set()
            detection_thread.join(timeout=1.0)
        rospy.loginfo("[INFO] Node shut down safely.")