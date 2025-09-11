import cv2
import numpy as np
import time


class RedCrossDetector:
    def __init__(self, calib_matrix, calib_dist, calib_resolution,
                 camera_height, pitch_angle):
        """
        初始化红色十字检测器
        参数:
        calib_matrix: 标定时的内参矩阵 (3x3)
        calib_dist: 标定时的畸变系数 (1x5)
        calib_resolution: 标定时的分辨率 (width, height)
        camera_height: 相机离地面的高度 (米)
        pitch_angle: 相机俯仰角 (弧度, 向下为正)
        """
        self.calib_matrix = calib_matrix
        self.calib_dist = calib_dist
        self.calib_resolution = calib_resolution
        self.camera_height = camera_height
        self.pitch_angle = pitch_angle

        # 验证参数有效性
        if camera_height <= 0:
            raise ValueError("相机高度必须为正实数！")

        # 检查俯仰角是否有效（避免0°和180°）
        if abs(pitch_angle) < 1e-5 or abs(pitch_angle - np.pi) < 1e-5:
            raise ValueError("俯仰角不能为0°或180°！")

        # 当前分辨率下的内参和畸变系数（初始化为标定值）
        self.current_matrix = calib_matrix.copy()
        self.current_dist = calib_dist.copy()

        # 计算相机相对于地面的旋转矩阵
        self.R = self._calculate_rotation_matrix(pitch_angle)

        # 初始化单应性矩阵
        self.H = None
        self.H_inv = None

        # 初始化状态变量
        self.last_detection_time = 0
        self.smoothed_world_pos = None
        self.smoothing_factor = 0.2  # 位置平滑因子

        # 计算单应性矩阵
        self._calculate_homography()

    def _calculate_rotation_matrix(self, pitch):
        """计算相机相对于地面的旋转矩阵"""
        # 绕X轴旋转 (俯仰角)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])

        # 由于相机坐标系Y轴向下，需要额外的180度绕X轴旋转
        R_flip = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])

        return R_flip @ R_x

    def update_resolution(self, current_resolution):
        """
        更新当前分辨率并重新计算内参矩阵
        参数:
        current_resolution: 当前分辨率 (width, height)
        """
        # 计算分辨率缩放比例
        scale_x = current_resolution[0] / self.calib_resolution[0]
        scale_y = current_resolution[1] / self.calib_resolution[1]

        # 更新内参矩阵
        self.current_matrix = self.calib_matrix.copy()
        self.current_matrix[0, 0] *= scale_x  # fx
        self.current_matrix[1, 1] *= scale_y  # fy
        self.current_matrix[0, 2] *= scale_x  # cx
        self.current_matrix[1, 2] *= scale_y  # cy

        # 重新计算单应性矩阵
        self._calculate_homography()

    def _calculate_homography(self):
        """计算从图像平面到地面的单应性矩阵（带奇异性检查）"""
        # 构造投影矩阵的前三列 [R | t]
        T = np.array([0, self.camera_height, 0])
        M = np.column_stack((self.R[:, 0], self.R[:, 1], T))

        # 计算单应性矩阵 H = K * M
        self.H = self.current_matrix @ M

        # 检查矩阵是否奇异
        det = np.linalg.det(self.H)
        if abs(det) < 1e-10:
            # 使用伪逆代替，并打印警告
            self.H_inv = np.linalg.pinv(self.H)
            print(f"警告：单应性矩阵接近奇异 (det={det:.6f})，使用伪逆代替")
        else:
            self.H_inv = np.linalg.inv(self.H)

    def detect_red_cross(self, frame):
        """检测图像中的红色十字并返回中心点坐标"""
        # 获取当前帧分辨率
        h, w = frame.shape[:2]
        current_res = (w, h)

        # 检查分辨率是否变化
        if not hasattr(self, 'last_resolution') or self.last_resolution != current_res:
            self.update_resolution(current_res)
            self.last_resolution = current_res

        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义红色的HSV范围（更宽的范围）
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 创建红色掩码
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        closed = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    def pixel_to_world(self, pixel_point):
        """
        将像素坐标转换为世界坐标
        参数:
        pixel_point: 像素坐标 (x, y)
        返回:
        world_point: 世界坐标 (X, Y) 在地面平面上
        """
        # 创建齐次坐标
        pixel_homogeneous = np.array([pixel_point[0], pixel_point[1], 1.0])

        # 应用单应性逆矩阵
        world_homogeneous = self.H_inv @ pixel_homogeneous

        # 检查z分量是否接近零
        if abs(world_homogeneous[2]) < 1e-10:
            # 处理无穷远点或无效点
            return (float('inf'), float('inf'))

        # 归一化
        world_homogeneous /= world_homogeneous[2]

        # 提取地面坐标 (X, Y)
        world_point = (world_homogeneous[0], world_homogeneous[1])

        return world_point

    def process_frame(self, frame):
        """处理单个视频帧并返回结果"""
        # 检测红色十字中心
        cross_center = self.detect_red_cross(frame)
        world_pos = None
        detection_status = False

        # 创建用于显示的副本
        display_frame = frame.copy()

        if cross_center is not None:
            # 更新最后检测时间
            self.last_detection_time = time.time()
            detection_status = True

            # 将像素坐标转换为世界坐标
            world_pos = self.pixel_to_world(cross_center)

            # 应用平滑滤波
            if self.smoothed_world_pos is None:
                self.smoothed_world_pos = world_pos
            else:
                self.smoothed_world_pos = (
                    self.smoothing_factor * world_pos[0] + (1 - self.smoothing_factor) * self.smoothed_world_pos[0],
                    self.smoothing_factor * world_pos[1] + (1 - self.smoothing_factor) * self.smoothed_world_pos[1]
                )

            # 在图像上绘制结果
            # 绘制十字中心点
            cv2.circle(display_frame, cross_center, 10, (0, 0, 255), -1)
            cv2.circle(display_frame, cross_center, 15, (0, 255, 255), 2)

            # 绘制坐标信息
            cv2.putText(display_frame, f"Pixel: ({cross_center[0]}, {cross_center[1]})",
                        (cross_center[0] - 100, cross_center[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(display_frame, f"World: ({self.smoothed_world_pos[0]:.2f}, {self.smoothed_world_pos[1]:.2f})m",
                        (cross_center[0] - 100, cross_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 在左上角显示状态
            cv2.putText(display_frame, "RED CROSS DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 如果最近2秒内检测到过目标，继续显示位置
        elif time.time() - self.last_detection_time < 2.0 and self.smoothed_world_pos is not None:
            detection_status = True
            world_pos = self.smoothed_world_pos

            # 在左上角显示预测位置
            cv2.putText(display_frame,
                        f"World: ({self.smoothed_world_pos[0]:.2f}, {self.smoothed_world_pos[1]:.2f})m (predicted)",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(display_frame, "PREDICTED POSITION", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            # 显示未检测到目标
            cv2.putText(display_frame, "NO RED CROSS DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 显示当前分辨率信息
        h, w = frame.shape[:2]
        cv2.putText(display_frame, f"Res: {w}x{h}", (w - 150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return display_frame, world_pos, detection_status


def main():
    # 标定参数（标定时的分辨率为1280x720）
    calib_resolution = (1280, 720)

    # 标定时的内参矩阵（更接近真实值）
    calib_matrix = np.array([
        [800.0, 0, 640.0],  # fx, 0, cx
        [0, 800.0, 360.0],  # 0, fy, cy
        [0, 0, 1]  # 0, 0, 1
    ])

    # 标定时的畸变系数
    calib_dist = np.array([-0.1, 0.01, 0, 0, 0])

    # 相机安装参数
    camera_height = 0.75  # 相机离地面高度 (米)
    pitch_angle = np.deg2rad(15)  # 相机俯仰角 (15度向下)

    # 创建检测器
    detector = RedCrossDetector(calib_matrix, calib_dist, calib_resolution,
                                camera_height, pitch_angle)

    # 打开摄像头
    cap = cv2.VideoCapture(1)  # 使用默认摄像头
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("按 'q' 键退出程序")

    # 创建世界坐标系可视化窗口
    world_vis = np.zeros((600, 600, 3), dtype=np.uint8)

    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("无法获取帧")
            break

        # 处理帧
        processed_frame, world_pos, detected = detector.process_frame(frame)

        # 更新世界坐标系可视化
        world_vis[:] = 0  # 清空画布

        # 绘制坐标轴和网格
        center_x, center_y = 300, 500  # 世界坐标系中心位置

        # 绘制网格
        for i in range(-3, 4):
            # X轴网格线
            cv2.line(world_vis, (center_x + i * 100, 0), (center_x + i * 100, 600), (50, 50, 50), 1)
            # Y轴网格线
            cv2.line(world_vis, (0, center_y - i * 100), (600, center_y - i * 100), (50, 50, 50), 1)

        # 绘制坐标轴
        cv2.arrowedLine(world_vis, (50, center_y), (550, center_y), (0, 255, 0), 2)  # X轴
        cv2.arrowedLine(world_vis, (center_x, 550), (center_x, 50), (255, 0, 0), 2)  # Y轴

        # 添加坐标轴标签
        cv2.putText(world_vis, "X (m)", (520, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(world_vis, "Y (m)", (center_x + 10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # 添加刻度标记
        for i in range(-3, 4):
            if i == 0:
                continue
            # X轴刻度
            cv2.putText(world_vis, f"{i}", (center_x + i * 100 - 5, center_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            # Y轴刻度
            cv2.putText(world_vis, f"{-i}", (center_x - 25, center_y - i * 100 + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # 添加原点标记
        cv2.putText(world_vis, "0", (center_x - 10, center_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 添加标题
        cv2.putText(world_vis, "World Coordinate System (Top View)", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 添加相机位置标记
        cv2.circle(world_vis, (center_x, center_y), 8, (0, 255, 255), -1)
        cv2.putText(world_vis, "Camera", (center_x - 40, center_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if detected and world_pos is not None:
            # 在世界坐标系中绘制位置 (缩放以适应可视化)
            # 1像素 = 0.1米，原点在(300,500)
            x = int(center_x + world_pos[0] * 100)  # X方向：1米 = 100像素
            y = int(center_y - world_pos[1] * 100)  # Y方向：向上为正

            # 确保点在可视范围内
            x = max(20, min(x, 580))
            y = max(20, min(y, 580))

            # 绘制位置点
            cv2.circle(world_vis, (x, y), 10, (0, 0, 255), -1)
            cv2.circle(world_vis, (x, y), 15, (0, 255, 255), 2)

            # 绘制位置线
            cv2.line(world_vis, (center_x, center_y), (x, y), (100, 100, 255), 1)

            # 添加坐标标签
            cv2.putText(world_vis, f"({world_pos[0]:.2f}, {world_pos[1]:.2f})m",
                        (x - 80, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # 添加距离信息
            distance = np.sqrt(world_pos[0] ** 2 + world_pos[1] ** 2)
            cv2.putText(world_vis, f"Distance: {distance:.2f}m",
                        (x - 80, y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        # 显示处理结果
        cv2.imshow('Red Cross Detection', processed_frame)
        cv2.imshow('World Coordinate View', world_vis)

        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()