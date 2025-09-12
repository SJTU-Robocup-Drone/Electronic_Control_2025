#!/home/amov/miniconda3/envs/yolo/bin/python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import torch
import numpy as np
import pyrealsense2 as rs
import time
import threading
from nav_msgs.msg import Odometry
from utils.augmentations import letterbox
from utils.general import non_max_suppression

# 视觉处理频率 帧/秒
vision_rate=30  

# 视觉状态控制
scanning_active = False
detection_thread = None
stop_event = threading.Event()

# 初始化滑动平均缓存
items = [
    ["bridge", 0, []], ["bunker", 0, []], ["car", 0, []],
    ["Helicopter", 0, []], ["tank", 0, []], ["tent", 0, []], ["car", 0, []]
]

# 坐标发布器
detection_pub = None
odom_pos = None

def check_queue(items, class_id, confidence, x, y, z):
    item = items[class_id]
    if confidence > 0.5:
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
    global detection_pub, odom_pos

    # 初始化 YOLO 模型
    model_path = '/home/amov/board_ws/src/board_ros/scripts/best_0801.pt'
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    rospy.loginfo("[INFO] 加载模型中...")
    model = torch.load(model_path, map_location=device)['model'].float()
    model.to(device).eval()
    model.half()
    names = model.names if hasattr(model, 'names') else {i: f'class_{i}' for i in range(100)}
    rospy.loginfo("[INFO] 模型加载完成。")

    # 初始化 RealSense 相机
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    align = rs.align(rs.stream.color)

    frame_count = 0
    try:
        while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():
            start_time = time.time()
            frame_count += 1
            
            rospy.loginfo(f"[INFO] 处理第 {frame_count} 帧")

            frames = pipeline.wait_for_frames()
            
            rospy.loginfo("[INFO] 获取到新帧。")
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            img_letterboxed = letterbox(color_image, new_shape=640)[0]
            img = img_letterboxed[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device).half() / 255.0
            img = img.unsqueeze(0)

            rospy.loginfo("[INFO] 运行推理...")

            with torch.no_grad():
                pred = model(img, augment=False)[0]
                pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)[0]

            rospy.loginfo("[INFO] 推理完成。")
            if pred is not None and len(pred):
                pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()
                for *xyxy, conf, cls in pred:
                    xmin, ymin, xmax, ymax = map(int, xyxy)
                    cx = (xmin + xmax) // 2
                    cy = (ymin + ymax) // 2

                    depth_value = odom_pos.z + 0.16 if odom_pos else 0
                    point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [cx, cy], depth_value)
                    x, y, z = point_3d
                    class_id = int(cls)
                    x, y, z = check_queue(items, class_id, float(conf), x, y, z)

                    # 发布检测结果
                    if conf > 0.5 and detection_pub and frame_count % (vision_rate/30) == 0:
                        detection_msg = PointStamped()
                        detection_msg.header.stamp = rospy.Time.now()
                        detection_msg.header.frame_id = names[class_id]  # 使用类别名作为frame_id
                        detection_msg.point.x = x
                        detection_msg.point.y = y
                        detection_msg.point.z = z
                        detection_pub.publish(detection_msg)
                        
                        rospy.loginfo(f"检测到: {names[class_id]}, 坐标: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, 置信度: {conf:.2f}")

            total_time = time.time() - start_time
            if total_time < 1 / 30:
                time.sleep(1 / 30 - total_time)

    except Exception as e:
        rospy.logerr(f"检测过程中出现错误: {e}")
    finally:
        pipeline.stop()
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
        rospy.loginfo("[INFO] 节点已安全关闭。")
