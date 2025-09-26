#!/home/amov/miniconda3/envs/yolo/bin/python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import torch
import numpy as np
import pyrealsense2 as rs
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
vision_rate =60            # 视觉处理fps（提示性，不强制sleep）
#enable_viz = False           # 是否显示窗口，关闭可提高运行速度
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

# Class names must be defined manually (TensorRT engine does not contain them)
names = {0: 'bridge', 1: 'bunker', 2: 'car', 3: 'Helicopter', 4: 'tank', 5: 'tent', 6: 'red'}

camera_matrix = np.array([
    [432.9524, 0, 417.1712],  # fx, 0, cx    fx,fy focal length
    [0, 426.8039, 315.5234],  # 0, fy, cy    cx,cy principal point  needs calibration
    [0, 0, 1.0]  # 0, 0, 1
], dtype=np.float32)  # Explicitly set to 32-bit float

desired_width = 640
desired_height = 480
desired_fps = 120
cap = None
# Coordinate publishers
detection_pub = None
random_pub = None
odom_pos = None

# Replace PyTorch model with TensorRT
engine_path = '/home/amov/board_ws/src/board_ros/scripts/best_0909.engine'
rospy.loginfo("[INFO] Loading TensorRT model...")

model = TensorRTModel(engine_path)
rospy.loginfo("[INFO] TensorRT model loaded successfully.")

latest_frame = None
frame_lock = threading.Lock()


grab_running = False

# 2D 卡尔曼追踪
# --------------------------
tank_tracking_active = False
tank_kf = None  # cv2.KalmanFilter(4,2): [x,y,vx,vy] / [x,y]
tank_last_time = None
last_tank_detection_time = None
tank_lost_timeout = 2.0
tank_traj_world = deque(maxlen=60)  # 世界坐标轨迹（米）
tank_traj_pix = deque(maxlen=60)    # 像素轨迹（绘制用）

# 滤波器性能监控变量
tank_innovation_history = deque(maxlen=10)  # 创新序列历史
tank_velocity_history = deque(maxlen=5)     # 速度历史
tank_convergence_count = 0                  # 收敛计数器

# --------------------------

#卡尔曼滤波所需的初始化的值
def init_tank_kalman():
    #4，2分别表示状态向量和观测向量的维度
    kf = cv2.KalmanFilter(4, 2)
    #状态转移矩阵A  即Xk=Xk-1+Vk-1*t
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)
    #观测矩阵H 相当于从状态向量中提取观测值，即Zx,Zy,传感器得到的x,y坐标值
    kf.measurementMatrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ], dtype=np.float32)

    # 优化的过程噪声协方差矩阵 (Q) - 增加过程噪声以提升响应速度
    # 位置方差稍小，速度方差适中，提升对运动变化的适应性
    Q_diag = np.array([5e-3, 5e-3, 1e-2, 1e-2], dtype=np.float32)  # [x, y, vx, vy]
    kf.processNoiseCov = np.diag(Q_diag)
    
    # 优化的观测噪声协方差矩阵 (R) - 降低观测噪声以信任观测值
    # 提高对观测的信任度，加快收敛
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 2e-2
    
    # 优化的初始后验估计协方差矩阵 (P) - 降低初始不确定性
    # 位置不确定性适中，速度不确定性较大
    P_diag = np.array([0.1, 0.1, 1.0, 1.0], dtype=np.float32)  # [x, y, vx, vy]
    kf.errorCovPost = np.diag(P_diag)

    return kf

#输入前后观测坐标的时间差，用于具体修正状态转移矩阵
def update_kf_dt(kf, dt):
    dt = max(1e-3, float(dt))
    A = kf.transitionMatrix.copy()
    A[0, 2] = dt
    A[1, 3] = dt
    kf.transitionMatrix = A
    return dt

# 自适应噪声调整函数
def adaptive_noise_adjustment(kf, velocity_magnitude, dt):
    """
    根据目标运动特性动态调整噪声参数
    velocity_magnitude: 目标速度大小
    dt: 时间间隔
    """
    # 速度阈值定义
    SLOW_THRESHOLD = 0.1  # m/s
    FAST_THRESHOLD = 2.0  # m/s
    
    # 基础噪声参数
    base_process_noise = np.array([5e-3, 5e-3, 1e-2, 1e-2], dtype=np.float32)
    base_measurement_noise = 2e-2
    
    # 根据速度调整过程噪声
    if velocity_magnitude < SLOW_THRESHOLD:
        # 慢速/静止目标 - 降低过程噪声，提高稳定性
        noise_scale = 0.5
    elif velocity_magnitude > FAST_THRESHOLD:
        # 快速目标 - 增加过程噪声，提高跟踪响应
        noise_scale = 2.0
    else:
        # 中等速度 - 线性插值
        ratio = (velocity_magnitude - SLOW_THRESHOLD) / (FAST_THRESHOLD - SLOW_THRESHOLD)
        noise_scale = 0.5 + 1.5 * ratio
    
    # 根据时间间隔调整 - 较大的dt需要更多过程噪声
    dt_scale = min(2.0, max(0.5, dt / 0.033))  # 基于30fps标准化
    
    # 应用调整
    adjusted_process_noise = base_process_noise * noise_scale * dt_scale
    kf.processNoiseCov = np.diag(adjusted_process_noise)
    
    # 观测噪声根据速度轻微调整
    measurement_scale = 1.0 + 0.2 * min(1.0, velocity_magnitude / FAST_THRESHOLD)
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * (base_measurement_noise * measurement_scale)
    
    return noise_scale, dt_scale

#计算X,P的先验值
def safe_kf_predict(kf):
    try:
        #首先尝试opencv内置的卡尔曼滤波运算，若失败再手动实现
        return kf.predict()
    except Exception:
        A = np.array(kf.transitionMatrix, dtype=np.float32)
        x_post = np.array(kf.statePost, dtype=np.float32)
        P_post = np.array(kf.errorCovPost, dtype=np.float32)
        Q = np.array(kf.processNoiseCov, dtype=np.float32)

        #计算先验值
        x_prior = A @ x_post
        P_prior = A @ P_post @ A.T + Q
        kf.statePre = x_prior
        kf.errorCovPre = P_prior
        return x_prior

#开始跟踪目标时，初始化滤波器
def start_tank_tracker(x_world, y_world, t_now):
    global tank_kf, tank_tracking_active, tank_traj_world, tank_traj_pix, tank_last_time, last_tank_detection_time
    global tank_innovation_history, tank_velocity_history, tank_convergence_count
    
    tank_kf = init_tank_kalman()
    #初始状态向量,x,y,Vx,Vy
    tank_kf.statePost = np.array([[x_world], [y_world], [0.0], [0.0]], dtype=np.float32)
    tank_tracking_active = True
    
    #清除世界系，像素系存储的轨迹点
    tank_traj_world.clear()
    tank_traj_pix.clear()
    
    # 清除性能监控历史
    tank_innovation_history.clear()
    tank_velocity_history.clear()
    tank_convergence_count = 0
    
    #加入轨迹点
    tank_traj_world.append((float(x_world), float(y_world)))
    #上一次卡尔曼滤波的更新时间,上一次侦测到tank的时间
    tank_last_time = float(t_now)
    last_tank_detection_time = float(t_now)
    rospy.loginfo("[TRACK] Tank tracker started at (%.3f, %.3f)", x_world, y_world)

#检测到目标，进行最优估计
def update_tank_tracker(x_world, y_world, t_now):
    global tank_kf, tank_last_time, last_tank_detection_time
    global tank_innovation_history, tank_velocity_history, tank_convergence_count
    
    if tank_kf is None:
        start_tank_tracker(x_world, y_world, t_now)
        return (x_world, y_world, 0.0, 0.0)

    #计算dt,即当前时间和上一次检测到tank的时间差，并更新状态转移矩阵A
    dt = update_kf_dt(tank_kf, float(t_now) - float(tank_last_time))
    
    # 预测步骤
    safe_kf_predict(tank_kf)
    
    # 获取当前速度估计用于自适应调整
    current_state = tank_kf.statePre if hasattr(tank_kf, 'statePre') else tank_kf.statePost
    vx_current = float(current_state[2, 0]) if current_state.shape[0] > 2 else 0.0
    vy_current = float(current_state[3, 0]) if current_state.shape[0] > 3 else 0.0
    velocity_magnitude = np.sqrt(vx_current**2 + vy_current**2)
    
    # 自适应噪声调整
    noise_scale, dt_scale = adaptive_noise_adjustment(tank_kf, velocity_magnitude, dt)
    
    #观测向量，包含观测到的x,y坐标
    meas = np.array([[x_world], [y_world]], dtype=np.float32)
    
    # 计算创新（观测与预测的差异）用于性能监控
    if hasattr(tank_kf, 'statePre'):
        pred_x, pred_y = float(tank_kf.statePre[0, 0]), float(tank_kf.statePre[1, 0])
        innovation = np.sqrt((x_world - pred_x)**2 + (y_world - pred_y)**2)
        tank_innovation_history.append(innovation)
    
    #估测值,根据先验预测值和观测值得到最优估计
    est = tank_kf.correct(meas)
    x_est, y_est = float(est[0, 0]), float(est[1, 0])
    vx_est, vy_est = float(est[2, 0]), float(est[3, 0])
    
    # 更新速度历史用于趋势分析
    tank_velocity_history.append(velocity_magnitude)
    
    # 收敛性监控
    if len(tank_innovation_history) >= 3:
        recent_innovations = list(tank_innovation_history)[-3:]
        avg_innovation = sum(recent_innovations) / len(recent_innovations)
        if avg_innovation < 0.05:  # 收敛阈值：5cm
            tank_convergence_count += 1
        else:
            tank_convergence_count = 0
            
        # 每20次更新输出一次性能信息
        if tank_convergence_count == 1 or tank_convergence_count % 20 == 0:
            rospy.loginfo("[FILTER] Convergence: %d, Avg Innovation: %.3f, Velocity: %.3f, Noise Scale: %.2f", 
                         tank_convergence_count, avg_innovation, velocity_magnitude, noise_scale)
    
    #更新滤波的更新时间和tank的观测时间
    tank_last_time = float(t_now)
    last_tank_detection_time = float(t_now)
    #加入轨迹点队列
    tank_traj_world.append((x_est, y_est))
    return (x_est, y_est, vx_est, vy_est)

#未检测到目标，仅给出预测值
# def predict_tank_tracker(t_now):
#     global tank_kf, tank_last_time
#     if tank_kf is None:
#         return None
#     dt = update_kf_dt(tank_kf, float(t_now) - float(tank_last_time))
#
#     pred = safe_kf_predict(tank_kf)
#     x_pred, y_pred = float(pred[0, 0]), float(pred[1, 0])
#     vx_pred, vy_pred = float(pred[2, 0]), float(pred[3, 0])
#     tank_last_time = float(t_now)
#     tank_traj_world.append((x_pred, y_pred))
#     return (x_pred, y_pred, vx_pred, vy_pred)

#停止跟踪，重置所有变量
def stop_tank_tracker():
    global tank_tracking_active, tank_kf, tank_last_time, last_tank_detection_time
    global tank_innovation_history, tank_velocity_history, tank_convergence_count
    
    if tank_tracking_active:
        rospy.loginfo("[TRACK] Tank tracker stopped.")
    tank_tracking_active = False
    tank_kf = None
    tank_last_time = None
    last_tank_detection_time = None
    
    # 清除性能监控变量
    tank_innovation_history.clear()
    tank_velocity_history.clear()
    tank_convergence_count = 0

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
    current_res = (w, h)

    #rospy.loginfo("TODELETE")
    # Preset HSV thresholds
    h_min, h_max = 7, 10
    s_min, s_max = 110, 255
    v_min, v_max = 60, 255

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red masks
    lower_red1 = np.array([h_min, s_min, v_min])
    upper_red1 = np.array([h_max, s_max, v_max])

    # For red, two ranges are usually required
    lower_red2 = np.array([max(0, 170 - (10 - h_min)), s_min, v_min]) if h_min < 10 else np.array(
        [170, s_min, v_min])
    upper_red2 = np.array([min(179, 180 + (h_max - 170)), s_max, v_max]) if h_max > 170 else np.array(
        [180, s_max, v_max])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2

    # Morphological operations to enhance cross shape
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    closed_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)

    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel_open)

    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cross_center = None
    max_area = 0

    for cnt in contours:
        # Ignore small contours (adaptive threshold based on resolution)
        min_area = (w * h) / 2000  # 0.05% of image area
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # Polygon approximation
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        # Convex hull
        hull = cv2.convexHull(approx)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        solidity = float(area) / hull_area

        # Check for cross shape: hull vertices 8-12, solidity 0.6-0.9
        if 8 <= len(hull) <= 12 and 0.6 <= solidity <= 0.9:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if area > max_area:
                    max_area = area
                    cross_center = (cX, cY)
    #rospy.loginfo("TODELETE1")
    return cross_center

def check_queue(items, class_id, confidence, x, y, z):
    item = items[class_id]
    if (item[0] == 'tank'):
        if confidence > 0.7:
            if item[1] < 1:
                item[1] += 1
                item[2].append([x, y, z])
            else:
                item[2].pop(0)
                item[2].append([x, y, z])

        if item[2]:
            x_avg = sum(p[0] for p in item[2]) / len(item[2])
            y_avg = sum(p[1] for p in item[2]) / len(item[2])
            z_avg = sum(p[2] for p in item[2]) / len(item[2])
        else:
            x_avg, y_avg, z_avg = 0, 0, 0
        return x_avg, y_avg, z_avg
    if (item[0] == 'red'):
        if confidence > 0.7:
            if item[1] < 2:
                item[1] += 1
                item[2].append([x, y, z])
            else:
                item[2].pop(0)
                item[2].append([x, y, z])

        if item[2]:
            x_avg = sum(p[0] for p in item[2]) / len(item[2])
            y_avg = sum(p[1] for p in item[2]) / len(item[2])
            z_avg = sum(p[2] for p in item[2]) / len(item[2])
        else:
            x_avg, y_avg, z_avg = 0, 0, 0
        return x_avg, y_avg, z_avg
    else:
        if confidence > 0.7:
            if item[1] < 5:
                item[1] += 1
                item[2].append([x, y, z])
            else:
                item[2].pop(0)
                item[2].append([x, y, z])

        if item[2]:
            x_avg = sum(p[0] for p in item[2]) / len(item[2])
            y_avg = sum(p[1] for p in item[2]) / len(item[2])
            z_avg = sum(p[2] for p in item[2]) / len(item[2])
        else:
            x_avg, y_avg, z_avg = 0, 0, 0
        return x_avg, y_avg, z_avg

# Coordinate scaling function (YOLO output -> original image size)

# img1_shape: network input size   img0_shape: original image size
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
    while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():

        with frame_lock:
            if latest_frame is None:
                rospy.sleep(0.001)
                continue
            color_image = latest_frame

            # color_image = np.asanyarray(frames)
        # Image preprocessing
        img_letterboxed = \
        letterbox(color_image, new_shape=(640, 640), color=(114, 114, 114), auto=False, scaleFill=False, scaleup=True)[
            0]
        img = img_letterboxed[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        # TensorRT inference

        output = model.infer(img)
        pred = torch.from_numpy(output).reshape(1, -1, 12)  # 7 classes + 5 = 12
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)[0]

        # Post-processing
        if pred is not None and len(pred):
            pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()
            for *xyxy, conf, cls in pred:
                if (conf < 0.8):
                    continue

                xmin, ymin, xmax, ymax = map(int, xyxy)

                # Bounding-box center in pixel coordinates
                x_found = (xmin + xmax) // 2
                y_found = (ymin + ymax) // 2

                camera_height = odom_pos.z if odom_pos else 0

                fx = camera_matrix[0, 0]
                fy = camera_matrix[1, 1]
                cx = camera_matrix[0, 2]
                cy = camera_matrix[1, 2]

                x = (x_found - cx) / fx * camera_height
                y = (y_found - cy) / fy * camera_height
                z = camera_height

                class_id = int(cls)

                x, y, z = check_queue(items, class_id, float(conf), x, y, z)
                # Publish detection results
                if conf > conf_thres_detect:
                    detection_msg = PointStamped()
                    detection_msg.header.stamp = rospy.Time.now()
                    detection_msg.header.frame_id = names[class_id]
                    if names[class_id] == "red":
                        cross_center = detect_red_cross(color_image)
                        if cross_center is not None:
                            x_new, y_new = cross_center
                            x_new = (x_new - cx) / fx * camera_height
                            y_new = (y_new - cy) / fy * camera_height
                            rospy.loginfo(
                                f"OpenCV detected: {names[class_id]}, coords: X={x_new:.2f}, Y={y_new:.2f}, Z={z:.2f}, conf: {conf:.2f}")

                            if math.sqrt((x - x_new) ** 2 + (y - y_new) ** 2) <= 0.3:
                                detection_msg.point.x = -x_new
                                detection_msg.point.y = -y_new
                                detection_msg.point.z = z
                                random_pub.publish(detection_msg)
                            else:
                                rospy.loginfo("OpenCV not detected, skip publishing")
                    #tank->kalman
                    if names[class_id] =="tank":
                        t_now=time.time()

                        tank_state = update_tank_tracker(x, y, t_now)
                        x_est, y_est, vx_est, vy_est = tank_state

                        # 发布卡尔曼滤波后的估计位置
                        detection_msg.point.x = -x_est
                        detection_msg.point.y = -y_est
                        detection_msg.point.z = z
                        detection_pub.publish(detection_msg)

                        rospy.loginfo(
                            f"tank tracked: X={x_est:.2f}, Y={y_est:.2f}, VX={vx_est:.2f}, VY={vy_est:.2f}, conf: {conf:.2f}")

                    if names[class_id] != "red":
                        detection_msg.point.x = -x
                        detection_msg.point.y = -y
                        detection_msg.point.z = z
                        detection_pub.publish(detection_msg)

                    rospy.loginfo(
                        f"YOLO detected: {names[class_id]}, coords: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, conf: {conf:.2f}")

    # cv2.destroyAllWindows()
    # rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("[INFO] Detection thread exited safely.")


def vision_state_callback(msg):
    global scanning_active, detection_thread, stop_event

    if msg.data and not scanning_active:
        rospy.loginfo("[INFO] Received vision start command, beginning scan...")
        scanning_active = True
        stop_event.clear()

        detection_thread = threading.Thread(target=detect_targets)
        detection_thread.daemon = True
        detection_thread.start()

    # elif not msg.data and scanning_active:
    #     rospy.loginfo("[INFO] Received vision stop command, stopping scan...")
    #     stop_event.set()
    #     scanning_active = False


def odom_callback(msg):
    global odom_pos
    odom_pos = msg.pose.pose.position


def shutdown_hook():
    rospy.loginfo("Shutting down TensorRT model...")
    model.cleanup()


if __name__ == "__main__":
    rospy.init_node("object_detector")
    threading.Thread(target=grab_thread, daemon=True).start()

    # Publishers
    detection_pub = rospy.Publisher("/detection_results", PointStamped, queue_size=10)
    random_pub = rospy.Publisher("/random_target", PointStamped, queue_size=2)
    rospy.Subscriber("/vision_state", Bool, vision_state_callback)
    rospy.Subscriber("/odom_high_freq", Odometry, odom_callback)
    rospy.loginfo("[INFO] Vision node started, waiting for /vision_state command...")

    # Auto-start scan (no external trigger required)
    rospy.loginfo("[INFO] Vision node started, auto-starting scan...")
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

