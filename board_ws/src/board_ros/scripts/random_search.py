import cv2
import numpy as np

def detect_red_cross(frame):
    """检测图像中的红色十字并返回中心点坐标"""
    # 获取当前帧分辨率
    h, w = frame.shape[:2]
    current_res = (w, h)



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

    return cross_center


#
# usage for now
# cap = cv2.VideoCapture(1)

# if not cap.isOpened():
#     print("无法打开摄像头")

# while True:
#     # 读取一帧
#     ret, frame = cap.read()
#     if not ret:
#         print("无法获取帧")
#         break

#     cross_center = detect_red_cross(frame)

#     print(cross_center)

