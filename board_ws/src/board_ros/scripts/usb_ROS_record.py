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
from nav_msgs.msg import Odometry
from utils.augmentations import letterbox
from utils.general import non_max_suppression
import os
from datetime import datetime

# 视觉处理频率 帧/秒
vision_rate=30  

# 视觉状态控制
scanning_active = False
detection_thread = None
stop_event = threading.Event()

# 视频录制相关变量
video_writer = None
recording_active = False
video_filename = None

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
odom_pos = None

def check_queue(items, class_id, confidence, x, y, z):
    item = items[class_id]
    if confidence > 0.7:    #模型默认剔除置信度低于0.7的目标框，此处可再升高
        if item[1] < 20:
            item[1] += 1
            item[2].append([x, y, z])
        else:
            r = np.random.randint(0, 20)
            item[2][r] = [x, y, z]

    if item[2]:
        x_avg = sum(p[0] for p in item[2]) / len(item[2])
        y_avg = sum(p[1] for p in item[2]) / len(item[2])
        z_avg = sum(p[2] for p in item[2]) / len(item[2])
    else:
        x_avg, y_avg, z_avg = 0, 0, 0
    return x_avg, y_avg, z_avg

def init_video_writer(width, height, fps):
    """初始化视频录制器"""
    global video_writer, video_filename, recording_active
    
    # 创建录制目录
    recording_dir = "/home/amov/board_ws/recordings"
    if not os.path.exists(recording_dir):
        os.makedirs(recording_dir)
    
    # 生成视频文件名（带时间戳）
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_filename = os.path.join(recording_dir, f"detection_recording_{timestamp}.mp4")
    
    # 初始化视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
    
    if video_writer.isOpened():
        recording_active = True
        rospy.loginfo(f"[INFO] 开始录制视频: {video_filename}")
        return True
    else:
        rospy.logerr(f"[ERROR] 无法初始化视频录制器: {video_filename}")
        return False

def release_video_writer():
    """释放视频录制器"""
    global video_writer, recording_active, video_filename
    
    if video_writer:
        video_writer.release()
        recording_active = False
        rospy.loginfo(f"[INFO] 视频录制完成，保存至: {video_filename}")
        video_writer = None
        video_filename = None

def draw_detection_boxes(image, detections, names):
    """在图像上绘制检测框"""
    display_image = image.copy()
    
    if detections is not None and len(detections):
        for *xyxy, conf, cls in detections:
            xmin, ymin, xmax, ymax = map(int, xyxy)
            class_id = int(cls)
            confidence = float(conf)
            
            # 设置不同类别的颜色
            color = (0, 255, 0)   # 绿色
                
            # 绘制检测框
            cv2.rectangle(display_image, (xmin, ymin), (xmax, ymax), color, 2)
            
            # 绘制标签和置信度
            label = f"{names[class_id]}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            
            # 绘制标签文字
            cv2.putText(display_image, label, (xmin, ymin - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # 添加时间戳和录制状态
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(display_image, f"Time: {timestamp}", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    if recording_active:
        cv2.putText(display_image, "● REC", (display_image.shape[1] - 100, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return display_image

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
    global detection_pub, odom_pos, video_writer, recording_active

    # 初始化 YOLO 模型
    model_path = '/home/amov/board_ws/src/board_ros/scripts/best_0801.pt'
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    rospy.loginfo("[INFO] 加载模型中...")
    model = torch.load(model_path, map_location=device)['model'].float()
    model.to(device).eval()
    model.half()
    names = model.names if hasattr(model, 'names') else {i: f'class_{i}' for i in range(100)}
    rospy.loginfo("[INFO] 模型加载完成。")

    cap = cv2.VideoCapture(0)  # 打开默认摄像头
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    
    if not cap.isOpened():
        rospy.logerr("无法打开摄像头")
        return
    
    # 获取实际的摄像头参数
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    rospy.loginfo(f"[INFO] 摄像头已打开。分辨率: {actual_width}x{actual_height}, FPS: {actual_fps}")
    
    # 初始化视频录制器
    if not init_video_writer(actual_width, actual_height, vision_rate):
        rospy.logerr("[ERROR] 视频录制器初始化失败")
        cap.release()
        return

    frame_count = 0
    try:
        while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():
            start_time = time.time()
            frame_count += 1

            ret, frames = cap.read()
            if not ret:
                rospy.logwarn("无法读取帧")
                continue

            color_image = np.asanyarray(frames)

            # 图像预处理
            img_letterboxed = letterbox(color_image, new_shape=640)[0]
            img = img_letterboxed[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device).half() / 255.0
            img = img.unsqueeze(0)

            pred = None
            with torch.no_grad():
                pred = model(img, augment=False)[0]
                pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)[0]

            if pred is not None and len(pred):
                pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()
                
                for *xyxy, conf, cls in pred:
                    xmin, ymin, xmax, ymax = map(int, xyxy)
                    
                    #检测框中心像素坐标
                    x_found = (xmin + xmax) // 2
                    y_found = (ymin + ymax) // 2

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
                    if conf > 0.8 and detection_pub and frame_count % (vision_rate//30) == 0:
                        detection_msg = PointStamped()
                        detection_msg.header.stamp = rospy.Time.now()
                        detection_msg.header.frame_id = names[class_id]  # 使用类别名作为frame_id
                        detection_msg.point.x = x
                        detection_msg.point.y = y
                        detection_msg.point.z = z
                        detection_pub.publish(detection_msg)
                        
                        rospy.loginfo(f"检测到: {names[class_id]}, 坐标: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, 置信度: {conf:.2f}")

            # 绘制检测框并保存帧
            display_frame = draw_detection_boxes(color_image, pred, names)
            
            # 将帧写入视频文件
            if recording_active and video_writer:
                video_writer.write(display_frame)
            
            # 控制帧率
            elapsed_time = time.time() - start_time
            sleep_time = max(0, 1.0/vision_rate - elapsed_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        rospy.logerr(f"检测过程中出现错误: {e}")
    finally:
        # 清理资源
        release_video_writer()
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
    
    rospy.Subscriber("/vision_state", Bool, vision_state_callback)
    rospy.Subscriber("/odom_high_freq", Odometry, odom_callback)
    rospy.loginfo("[INFO] 视觉节点已启动，等待/vision_state指令...")
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        if scanning_active:
            stop_event.set()
            if detection_thread and detection_thread.is_alive():
                detection_thread.join(timeout=1.0)
        rospy.loginfo("[INFO] 节点已安全关闭。")
