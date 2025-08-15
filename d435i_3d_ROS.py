#!/home/amov/miniconda3/envs/yolo/bin/python

import rospy
from std_msgs.msg import Bool
#import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import time
import os
from utils.augmentations import letterbox
from utils.general import non_max_suppression
import threading  # 添加线程支持

# 视觉状态控制
scanning_active = False # 标记扫描是否开启
detection_thread = None
stop_event = threading.Event()  # 用于停止检测线程

# 初始化滑动平均缓存
items = [
    ["bridge", 0, []], ["bunker", 0, []], ["car", 0, []],
    ["Helicopter", 0, []], ["tank", 0, []], ["tent", 0, []]
]

# 坐标缓存滑动平均

def check_queue(items, class_id, confidence, x, y, z):
    item = items[class_id]
    if confidence > 0.5:      #模型默认剔除置信度低于0.25的目标框，此处再升高
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
# 对于输入给Yolov5处理的图像，Yolov5会预处理成640×640（默认，可更改）的图像，进行目标检测后输出
# 该函数将输出的检测框坐标映射回符合原图像尺寸的坐标
# img1_shape:输出图尺寸  img0_shape：原图尺寸

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

# 获取检测框内有效深度均值（单位：米）,取四分之一

def get_valid_depth(depth_image, xmin, ymin, xmax, ymax, depth_scale, cx, cy):
    h, w = depth_image.shape
    if cx < 2 or cx >= w - 2 or cy < 2 or cy >= h - 2:
        return 0
    length = abs(xmax - xmin)
    width = abs(ymax - ymin)
    patch = depth_image[cy - int(width / 4) + 1:cy + int(width / 4) - 1,
            cx - int(length / 4) + 1:cx + int(length / 4) - 1]
    patch = patch[patch > 0]
    if patch.size == 0:
        return 0
    return np.mean(patch) * depth_scale

# 主检测逻辑函数
def detect_targets():
    # 初始化 YOLO 模型
    model_path = 'runs/train/exp_gpu8/weights/best_0702.pt'
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

    output_file = "detection_log.txt"
    frame_count = 0

    try:
        with open(output_file, 'a') as log:
            while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():
                frame_count += 1
                start_time = time.time()

                frames = pipeline.wait_for_frames()
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

                with torch.no_grad():
                    pred = model(img, augment=False)[0]
                    pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)[0]

                if pred is not None and len(pred):
                    pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], color_image.shape).round()
                    for *xyxy, conf, cls in pred:
                        xmin, ymin, xmax, ymax = map(int, xyxy)
                        cx = (xmin + xmax) // 2
                        cy = (ymin + ymax) // 2

                        depth_value = get_valid_depth(depth_image, xmin, ymin, xmax, ymax, depth_scale, cx, cy)
                        point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [cx, cy], depth_value) if depth_value > 0 else (0, 0, 0)
                        x, y, z = point_3d
                        class_id = int(cls)
                        x, y, z = check_queue(items, class_id, float(conf), x, y, z)

                        label = f"{names[int(cls)]} {conf:.2f}"
                        coord_text = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}"
                        full_label = f"{label} {coord_text}"

                        #cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                        #cv2.putText(color_image, full_label, (xmin, ymin - 10),
                                    #cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

                        if frame_count % 30 == 0:
                            timestamp = time.time()
                            log.write(f"{timestamp:.3f},{names[int(cls)]},{conf:.2f},{x:.3f},{y:.3f},{z:.3f}\n")
                            log.flush()
                            os.fsync(log.fileno())

                total_time = time.time() - start_time
                if total_time < 1 / 30:
                    time.sleep(1 / 30 - total_time)

                #if cv2.waitKey(1) & 0xFF == 27:
                    #break

    except KeyboardInterrupt:
        rospy.loginfo("[INFO] 用户中断程序。")
    finally:
        pipeline.stop()
        #cv2.destroyAllWindows()
        rospy.loginfo("[INFO] 已安全退出。")



def vision_state_callback(msg):
    global scanning_active, detection_thread, stop_event
    
    # 当收到True且当前扫描状态为关闭时，启动扫描并标记状态为开启
    if msg.data and not scanning_active:
        rospy.loginfo("[INFO] 收到视觉启动指令，开始扫描...")
        scanning_active = True
        stop_event.clear()
            
        # 创建并启动检测线程
        detection_thread = threading.Thread(target=detect_targets)
        detection_thread.daemon = True  # 设置为守护线程
        detection_thread.start()
    
    # 当收到False且当前扫描状态为开启时，停止扫描并标记状态为关闭
    elif not msg.data and scanning_active:
        rospy.loginfo("[INFO] 收到视觉停止指令，停止扫描...")
        stop_event.set()  # 设置停止事件
        scanning_active = False


if __name__ == "__main__":
    rospy.init_node("object_detector")
    
    # 订阅/vision_state话题（std_msgs/Bool）
    rospy.Subscriber("/vision_state", Bool, vision_state_callback)
    
    rospy.loginfo("[INFO] 视觉节点已启动，等待/vision_state指令...")
    
    try:
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        # 确保节点退出时停止扫描
        if scanning_active:
            stop_event.set()
            detection_thread.join(timeout=1.0)
        rospy.loginfo("[INFO] 节点已安全关闭。")