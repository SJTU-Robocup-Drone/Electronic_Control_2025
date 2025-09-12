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

# 视频录制控制
recording_active = False
video_writer = None
video_save_path = "/home/amov/board_ws/src/board_ros/videos/"

# 初始化滑动平均缓存
items = [
    ["bridge", 0, []], ["bunker", 0, []], ["car", 0, []],
    ["Helicopter", 0, []], ["tank", 0, []], ["tent", 0, []],["red", 0, []]
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

def init_video_writer():
    """初始化视频写入器"""
    global video_writer, video_save_path
    
    # 确保保存目录存在
    if not os.path.exists(video_save_path):
        os.makedirs(video_save_path)
        rospy.loginfo(f"[INFO] 创建视频保存目录: {video_save_path}")
    
    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_filename = f"detection_video_{timestamp}.avi"
    video_path = os.path.join(video_save_path, video_filename)
    
    # 初始化视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 可以改为 'MP4V' 或其他编码器
    video_writer = cv2.VideoWriter(video_path, fourcc, desired_fps, (desired_width, desired_height))
    
    if video_writer.isOpened():
        rospy.loginfo(f"[INFO] 视频录制开始，保存路径: {video_path}")
        return True
    else:
        rospy.logerr(f"[ERROR] 无法初始化视频写入器: {video_path}")
        return False

def release_video_writer():
    """释放视频写入器"""
    global video_writer
    if video_writer is not None:
        video_writer.release()
        video_writer = None
        rospy.loginfo("[INFO] 视频录制结束，文件已保存")

def detect_targets():
    global detection_pub, odom_pos, video_writer, recording_active

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

    # 初始化视频录制
    recording_active = True
    if not init_video_writer():
        recording_active = False

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

            processed_frame = frames.copy()
            
            if pred is not None and len(pred):
                pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()
                for *xyxy, conf, cls in pred:
                    xmin, ymin, xmax, ymax = map(int, xyxy)

                    class_id = int(cls)
                    label = f"{names[class_id]}{conf:.2f}"

                    # 选择颜色 (BGR格式)
                    colors = [(0, 255, 0), (0, 165, 255), (255, 0, 0), 
                             (255, 255, 0), (0, 255, 255), (255, 0, 255), (0, 0, 255)]
                    color = colors[class_id % len(colors)]
                    
                    # 绘制矩形框
                    cv2.rectangle(processed_frame, (xmin, ymin), (xmax, ymax), color, 2)
                    
                    # 绘制标签背景
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    cv2.rectangle(processed_frame, (xmin, ymin - label_size[1] - 10), 
                                 (xmin + label_size[0], ymin), color, -1)
                    
                    # 绘制标签文字
                    cv2.putText(processed_frame, label, (xmin, ymin - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
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

                   
                    x, y, z = check_queue(items, class_id, float(conf), x, y, z)

                    # 发布检测结果
                    if conf > 0.8 and detection_pub and frame_count % (vision_rate/30) == 0:
                        detection_msg = PointStamped()
                        detection_msg.header.stamp = rospy.Time.now()
                        detection_msg.header.frame_id = names[class_id]  # 使用类别名作为frame_id
                        detection_msg.point.x = x
                        detection_msg.point.y = y
                        detection_msg.point.z = z
                        detection_pub.publish(detection_msg)
                        
                        rospy.loginfo(f"检测到: {names[class_id]}, 坐标: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, 置信度: {conf:.2f}")
            # 保存处理后的帧到视频文件
            if recording_active and video_writer is not None:
                video_writer.write(processed_frame)
                
    except Exception as e:
        rospy.logerr(f"检测过程中出现错误: {e}")
    finally:
        cv2.destroyAllWindows()
        cap.release()
        
        # 停止视频录制
        if recording_active:
            release_video_writer()
            recording_active = False
        
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
            detection_thread.join(timeout=1.0)
        
        # 确保视频录制正常结束
        if recording_active:
            release_video_writer()
        
        rospy.loginfo("[INFO] 节点已安全关闭。")
