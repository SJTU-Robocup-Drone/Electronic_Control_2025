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

# Vision processing frequency (frames per second)
vision_rate = 60

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
    [432.9524, 0, 417.1712],      # fx, 0, cx    fx,fy focal length
    [0, 426.8039, 315.5234],      # 0, fy, cy    cx,cy principal point  needs calibration
    [0, 0, 1.0]                   # 0, 0, 1
], dtype=np.float32)              # Explicitly set to 32-bit float

desired_width = 640
desired_height = 480
desired_fps = 120
cap  =  None
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
frame_lock   = threading.Lock()                   


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

    rospy.loginfo("TODELETE")
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
    rospy.loginfo("TODELETE1")
    return cross_center


def check_queue(items, class_id, confidence, x, y, z):
    item = items[class_id]
    if(item == 'tank'):
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
    if(item == 'red'):
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
        img_letterboxed = letterbox(color_image,new_shape=(640, 640),color=(114, 114, 114),auto=False,   scaleFill=False, scaleup=True)[0]
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
                if(conf<0.8):
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
                if conf > 0.8:
                    detection_msg = PointStamped()
                    detection_msg.header.stamp = rospy.Time.now()
                    detection_msg.header.frame_id = names[class_id]
                    if names[class_id] == "red":
                        cross_center = detect_red_cross(color_image)
                        if cross_center is not None:
                            x_new, y_new = cross_center
                            x_new = (x_new - cx) / fx * camera_height
                            y_new = (y_new - cy) / fy * camera_height
                            rospy.loginfo(f"OpenCV detected: {names[class_id]}, coords: X={x_new:.2f}, Y={y_new:.2f}, Z={z:.2f}, conf: {conf:.2f}")

                            if math.sqrt((x - x_new) ** 2 + (y - y_new) ** 2) <= 0.3:
                                detection_msg.point.x = -x_new
                                detection_msg.point.y = -y_new
                                detection_msg.point.z = z
                                random_pub.publish(detection_msg)
                            else:
                                rospy.loginfo("OpenCV not detected, skip publishing")

                    if names[class_id] != "red":
                        detection_msg.point.x = -x
                        detection_msg.point.y = -y
                        detection_msg.point.z = z
                        detection_pub.publish(detection_msg)

                    rospy.loginfo(f"YOLO detected: {names[class_id]}, coords: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, conf: {conf:.2f}")


    
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
    
    global_running = True
    
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


