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
from nav_msgs.msg import Odometry
from utils.augmentations import letterbox
from utils.general import non_max_suppression

# 视觉处理频率 帧/秒
vision_rate=60  

# 视觉状态控制
scanning_active = False
detection_thread = None
stop_event = threading.Event()

# 初始化滑动平均缓存
items = [
    ["bridge", 0, []], ["bunker", 0, []], ["car", 0, []],
    ["Helicopter", 0, []], ["tank", 0, []], ["tent", 0, []], ["red", 0, []]
]

# 相机内参矩阵 - 需要根据实际相机进行标定
camera_matrix = np.array([
    [688.1914, 0, 643.0892],      # fx, 0, cx    fx,fy为焦距
    [0, 677.2838, 359.0183],      # 0, fy, cy    cx,cy为光心  需校准
    [0, 0, 1.0]             # 0, 0, 1
], dtype=np.float32)    # 这里指定数据类型为32位浮点数


desired_width = 1280
desired_height = 720
desired_fps = 60
# 坐标发布器
detection_pub = None
random_pub = None
odom_pos = None

def detect_red_cross(frame):
    """检测图像中的红色十字并返回中心点坐标"""
    # 获取当前帧分辨率
    h, w = frame.shape[:2]
    current_res = (w, h)


    rospy.loginfo("TODELETE")
    # 预设HSV阈值
    h_min, h_max = 7, 10
    s_min, s_max = 110, 255
    v_min, v_max = 60, 255

    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 红色掩码
    lower_red1 = np.array([h_min, s_min, v_min])
    upper_red1 = np.array([h_max, s_max, v_max])

    # 对于红色，通常需要两个范围
    lower_red2 = np.array([max(0, 170 - (10 - h_min)), s_min, v_min]) if h_min < 10 else np.array(
        [170, s_min, v_min])
    upper_red2 = np.array([min(179, 180 + (h_max - 170)), s_max, v_max]) if h_max > 170 else np.array(
        [180, s_max, v_max])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2

    # 对红色掩码进行形态学操作，增强十字形状
    # 先闭运算填充小孔洞
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    closed_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)

    # 再开运算去除小噪点
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel_open)

    # 查找轮廓
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 存储候选十字中心
    cross_center = None
    max_area = 0

    for cnt in contours:
        # 忽略小轮廓（根据分辨率自适应阈值）
        min_area = (w * h) / 2000  # 图像面积的0.05%
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # 多边形近似
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        # 计算轮廓的凸包
        hull = cv2.convexHull(approx)

        # 计算凸包的面积和轮廓面积的比例
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        solidity = float(area) / hull_area

        # 检查是否为十字形状（凸包点数在8-12之间，实心度在0.6-0.9之间）
        if 8 <= len(hull) <= 12 and 0.6 <= solidity <= 0.9:
            # 计算轮廓的矩和中心点
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # 选择面积最大的十字作为目标
                if area > max_area:
                    max_area = area
                    cross_center = (cX, cY)
    rospy.loginfo("TODELETE1")
    return cross_center



def check_queue(items, class_id, confidence, x, y, z):
    item = items[class_id]
    if confidence > 0.7:
        if item[1] < 3:
            item[1] += 1
            item[2].append([x, y, z])
        else:
            r = np.random.randint(0, 3)
            item[2][r] = [x, y, z]

    if item[2]:
        x_avg = sum(p[0] for p in item[2]) / len(item[2])
        y_avg = sum(p[1] for p in item[2]) / len(item[2])
        z_avg = sum(p[2] for p in item[2]) / len(item[2])
    else:
        x_avg, y_avg, z_avg = 0, 0, 0
    return x_avg, y_avg, z_avg

# 坐标缩放函数（YOLO输出 → 原图尺寸）
#对于输入给Yolov5处理的图像，Yolov5会预处理成640×640（默认，可更改）的图像，进行目标检测后输出
#该函数将输出的检测框坐标映射回符合原图像尺寸的坐标
#img1_shape:输出图尺寸  img0_shape：原图尺寸
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
    global detection_pub, odom_pos, random_pub

    # 初始化 YOLO 模型
    model_path = '/home/amov/board_ws/src/board_ros/scripts/best_0909.pt'
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    rospy.loginfo("[INFO] 加载模型中...")
    model = torch.load(model_path, map_location=device)['model'].float()
    model.to(device).eval()
    model.half()
    names = model.names if hasattr(model, 'names') else {i: f'class_{i}' for i in range(100)}
    rospy.loginfo("[INFO] 模型加载完成。")

    # # 初始化 RealSense 相机
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # profile = pipeline.start(config)
    # depth_sensor = profile.get_device().first_depth_sensor()
    # depth_scale = depth_sensor.get_depth_scale()
    # color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    # align = rs.align(rs.stream.color)


    cap = cv2.VideoCapture(0)  # 打开默认摄像头
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    
    if not cap.isOpened():
        rospy.logerr("无法打开摄像头")
        exit()
    print("[INFO] 摄像头已打开。")

    frame_count = 0
    try:
        while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():
            start_time = time.time()
            frame_count += 1
            

            ret, frames = cap.read()
            if not ret:
                print("无法读取帧")
                break

            
            # color_image = np.asanyarray(cv2.cvtColor(frames, cv2.COLOR_BGR2RGB))#转换为numpy数组
            color_image = np.asanyarray(frames)#转换为numpy数组

            # 图像预处理
            img_letterboxed = letterbox(color_image, new_shape=640)[0]
            img = img_letterboxed[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device).half() / 255.0
            img = img.unsqueeze(0)

            with torch.no_grad():
                pred = model(img, augment=False)[0]
                pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)[0]

            if pred is not None and len(pred):
                pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()
                for *xyxy, conf, cls in pred:
                    xmin, ymin, xmax, ymax = map(int, xyxy)
                    
                    #检测框中心像素坐标
                    x_found= (xmin + xmax) // 2
                    y_found= (ymin + ymax) // 2

                    camera_height = odom_pos.z if odom_pos else 0

                    fx = camera_matrix[0, 0]  # x方向焦距
                    fy = camera_matrix[1, 1]  # y方向焦距
                    cx = camera_matrix[0, 2]  # 主点x坐标
                    cy = camera_matrix[1, 2]  # 主点y坐标

                    x = (x_found - cx) / fx * camera_height
                    y = (y_found - cy) / fy * camera_height
                    z = camera_height  

                    class_id = int(cls)
                    x, y, z = check_queue(items, class_id, float(conf), x, y, z)

                    # 发布检测结果
                    if conf > 0.8 and frame_count % (60/vision_rate) == 0:
                        detection_msg = PointStamped()
                        detection_msg.header.stamp = rospy.Time.now()
                        detection_msg.header.frame_id = names[class_id]  # 使用类别名作为frame_id
                        if names[class_id] == "red":
                            cross_center = detect_red_cross(frames)
                            if cross_center != None:
                              x_new, y_new = cross_center
                              x_new = (x_new - cx) / fx * camera_height
                              y_new = (y_new - cy) / fy * camera_height
                              rospy.loginfo(f"OpenCV检测到: {names[class_id]}, 坐标: X={x_new:.2f}, Y={y_new:.2f}, Z={z:.2f}, 置信度: {conf:.2f}")
                              if math.sqrt((x-x_new)**2 +(y-y_new)**2) <= 0.1:
                                  detection_msg.point.x = -x_new
                                  detection_msg.point.y = -y_new
                                  detection_msg.point.z = z
                                  random_pub.publish(detection_msg)
                            else:
                              rospy.loginfo(f"OpenCV未检测到，不上传话题")
                        if names[class_id] != "red":
                            detection_msg.point.x = -x
                            detection_msg.point.y = -y
                            detection_msg.point.z = z
                            detection_pub.publish(detection_msg)
                        
                        rospy.loginfo(f"YOLO检测到: {names[class_id]}, 坐标: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, 置信度: {conf:.2f}")

            

    except Exception as e:
        rospy.logerr(f"检测过程中出现错误: {e}")
    finally:
        cv2.destroyAllWindows()
        cap.release()
        rospy.loginfo("[INFO] 已安全退出检测线程。")

def vision_state_callback(msg):
    global scanning_active, detection_thread, stop_event

    if msg.data and not scanning_active:
        rospy.loginfo("[INFO] 收到视觉启动指令，开始扫描...")
        scanning_active = True
        stop_event.clear()

        detection_thread = threading.Thread(target=detect_targets)
        detection_thread.daemon = True
        detection_thread.start()

    elif not msg.data and scanning_active:
        rospy.loginfo("[INFO] 收到视觉停止指令，停止扫描...")
        stop_event.set()
        scanning_active = False

def odom_callback(msg):
    global odom_pos
    odom_pos = msg.pose.pose.position

if __name__ == "__main__":
    rospy.init_node("object_detector")
    
    # 创建发布器
    detection_pub = rospy.Publisher("/detection_results", PointStamped, queue_size=10)
    random_pub = rospy.Publisher("/random_target", PointStamped, queue_size=2)
    rospy.Subscriber("/vision_state", Bool, vision_state_callback)
    rospy.Subscriber("/odom_high_freq", Odometry, odom_callback)
    rospy.loginfo("[INFO] 视觉节点已启动，等待/vision_state指令...")

    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        if scanning_active:
            stop_event.set()
            detection_thread.join(timeout=1.0)
        rospy.loginfo("[INFO] 节点已安全关闭。")
