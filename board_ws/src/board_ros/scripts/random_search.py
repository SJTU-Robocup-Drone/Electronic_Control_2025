import cv2
import numpy as np

def detect_red_cross(image):
    # 预设HSV阈值
    h_min, h_max = 7, 10
    s_min, s_max = 110, 255
    v_min, v_max = 60, 255
    
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 红色掩码
    lower_red1 = np.array([h_min, s_min, v_min])
    upper_red1 = np.array([h_max, s_max, v_max])
    
    # 对于红色，通常需要两个范围
    lower_red2 = np.array([max(0, 170 - (10 - h_min)), s_min, v_min]) if h_min < 10 else np.array([170, s_min, v_min])
    upper_red2 = np.array([min(179, 180 + (h_max - 170)), s_max, v_max]) if h_max > 170 else np.array([180, s_max, v_max])
    
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

    result = image.copy()
    cross_positions = []
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 50:  # 过滤小区域
            # 获取轮廓的边界矩形
            x, y, w, h = cv2.boundingRect(contour)
            
            # 计算轮廓的纵横比，用于筛选十字形状
            aspect_ratio = float(w) / h if h > 0 else 0
            
            # 十字通常有相对平衡的纵横比（接近1:1）
            if 0.3 <= aspect_ratio <= 3.0:
                cross_positions.append((x, y, w, h))
                
                # 在原图上绘制矩形框
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(result, 'Cross', (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # 创建显示图像
    red_mask_display = cv2.cvtColor(cleaned_mask, cv2.COLOR_GRAY2BGR)
    
    # 在掩码图像上也绘制检测结果
    for x, y, w, h in cross_positions:
        cv2.rectangle(red_mask_display, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(red_mask_display, 'Cross', (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
 
    return result, cross_positions, cleaned_mask

def main():
    image_path = "3.jpg"  # 请替换为您的图像路径
    image = cv2.imread(image_path)
    if image is None:
        print("无法读取图片，请检查路径")
        return
    
    result, crosses, red_mask = detect_red_cross(image) 
    
    print(f"检测到 {len(crosses)} 个红色十字")
    for i, (x, y, w, h) in enumerate(crosses):
        print(f"十字 {i+1}: 位置({x}, {y}), 尺寸({w}x{h})")
    
    cv2.namedWindow('Red Cross Detection', cv2.WINDOW_NORMAL)
    cv2.imshow('Red Cross Detection', result)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
