#!/home/amov/miniconda3/envs/yolo/bin/python

import rospy
import math # 用于解算
import tf2_ros # 用于坐标系变换
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Point
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
# 注意：在新版代码中，items储存的坐标都为camera_init坐标系下的全局坐标！
# 第一个字段表示item名称，第二个表示记录的坐标数目，第三个为坐标数组，第四个为投弹情况
# items顺序已修改为分数顺序
items = [
    ["tent", 0, [],False], ["bunker", 0, [],False], ["bridge", 0, [],False],
    ["car", 0, [],False], ["tank", 0, [],False], ["Helicopter", 0, [],False]
]

# 以下变量用于坐标解算
yaw = 0.0
coordX = 0
coordY = 0


# 用于表示一个锁定的目标
current_index = -1

# 存入新的坐标
def append_queue(items, class_id, confidence, x, y, z,coordX,coordY,yaw):
    item = items[class_id]
    # global_x和global_y表示全局坐标系下的坐标
    global_x = (coordX + x * math.sin(yaw) - y * math.cos(yaw))
    global_y = (coordY - x * math.cos(yaw) - y * math.sin(yaw))
    if confidence > 0.3:      #剔除置信度低于0.3的目标框
        if item[1] < 20:
            item[1] += 1
            item[2].append([global_x,global_y])
        else:
            r = np.random.randint(0, 20)
            item[2][r] = [global_x,global_y]

# 坐标缩放函数（YOLO输出 → 原图尺寸）
# 对于输入给Yolov5处理的图像，Yolov5会预处理成640×640（默认，可更改）的图像，进行目标检测后输出
# 该函数将输出的检测框坐标映射回符合原图像尺寸的坐标
# img1_shape:输出图尺寸  img0_shape：原图尺寸

def get_average_coords(items,class_id):
    item = items[class_id]
    if item[2]:
        x_avg = sum(p[0] for p in item[2]) / len(item[2])
        y_avg = sum(p[1] for p in item[2]) / len(item[2])
    else:
        x_avg, y_avg = 0, 0
    return x_avg, y_avg

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

    try:
        while scanning_active and not stop_event.is_set() and not rospy.is_shutdown():
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
                    class_id = cls_to_classid(int(cls)) # 注意，这里的class_id是重映射的！

                    if(class_id != -1) : append_queue(items, class_id, float(conf), x, y, z,coordX, coordY, yaw)
                    
                    label = f"{names[int(cls)]} {conf:.2f}"
                    coord_text = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}"
                    full_label = f"{label} {coord_text}"

                    #cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    #cv2.putText(color_image, full_label, (xmin, ymin - 10),
                                #cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

            # 控制30帧率
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

# 由于历史问题，cls的排序与靶标分值排序并不相符
# 因此我们使用了switch函数，将cls转换为新版本的class_id
def cls_to_classid(value):
    switcher = {
        0:2,
        1:1,
        2:3,
        3:5,
        4:4,
        5:0
    }
    return switcher.get(value,-1)

def get_yaw_from_pose(pose):
    """
    从geometry_msgs/Pose中提取yaw角(偏航角)
    参数:
        pose: geometry_msgs.msg.Pose对象
    返回:
        yaw: 弧度制的偏航角(绕Z轴旋转的角度)
    """
    orientation = pose.orientation
    # 提取四元数
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    
    # 将四元数转换为欧拉角 (roll, pitch, yaw)
    try:
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        return yaw
    except Exception as e:
        rospy.logerr(f"四元数转换失败: {str(e)}")
        return 0.0

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

def local_position_callback(msg):
    # 读取coordX,coordY,yaw以便后续解算
    global coordX, coordY, yaw
    coordX = msg.pose.position.x
    coordY = msg.pose.position.y
    yaw = get_yaw_from_pose(msg.pose)

def manba_callback(msg):
    global current_index,items
    if current_index != -1 :
        items[current_index][3] = False # 表示已经投弹
        current_index = -1 # 重置current_index

if __name__ == "__main__":
    rospy.init_node("object_detector")
    
    # 订阅/vision_state话题（std_msgs/Bool）
    rospy.Subscriber("/vision_state", Bool, vision_state_callback)
    # 订阅/local_position/pose话题，获取当前坐标（geometry_msgs::PoseStamped）
    rospy.Subscriber("/uav1/mavros/local_position/pose",PoseStamped,local_position_callback)
    # 订阅投弹情况
    rospy.Subscriber("/manba_input",Int32,manba_callback)
    # 用于发布目标
    target_pub = rospy.Publisher("/target",PoseStamped,10)
    # 创建target_pose消息
    target_pose = PoseStamped()
    target_pose.header = Header()
    # 调试信息
    rospy.loginfo("[INFO] 视觉节点已启动，等待/vision_state指令...")
    
    try:
        rospy.spin()  # 保持节点运行

        # target发布功能
        target_pose.header.stamp = rospy.Time.now()  # ROS时间戳
        target_pose.header.frame_id = "map"         # 参考坐标系
        if current_index != -1 :
            # 有锁定的目标，就直接发布对应坐标
            (target_x,target_y) = get_average_coords(items,current_index)    
            target_pose.pose.position = Point(x=target_x,y=target_y,z=1.0)
        else :
            # 没有锁定的目标，就先进行遍历
            is_found = False
            for i in range(4,-1,-1): # 对0~4号靶标进行降序遍历
                if((not items[i][3]) and items[i][2]) :
                    (target_x,target_y) = get_average_coords(items,i)    
                    target_pose.pose.position = Point(x=target_x,y=target_y,z=1.0)
                    current_index = i
                    is_found = True
                break # 直接结束遍历
            if not is_found :
                target_pose.pose.position = Point(x=0,y=0,z=-1.0) # 表示无效值
        target_pub.publish(target_pose) # 发布
    except rospy.ROSInterruptException:
        # 确保节点退出时停止扫描
        if scanning_active:
            stop_event.set()
            detection_thread.join(timeout=1.0)
        rospy.loginfo("[INFO] 节点已安全关闭。")