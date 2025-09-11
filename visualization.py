#!/usr/bin/env python3
# viz_endpoints_posearray_dir.py — Visualize endpoints (PoseArray) + direction (Vector3Stamped) + target points
#
# 订阅:
#   /track/endpoints_posearray : geometry_msgs/PoseArray   (poses[0]=A, poses[1]=B)
#   /track/endpoints_dir       : geometry_msgs/Vector3Stamped (u: 单位方向 A->B)
#   /target_pose               : geometry_msgs/PoseStamped (目标点坐标)
#
# 绘制:
#   - 滑窗内目标点散点
#   - A—B 线段
#   - A 点出发的方向箭头 u
#
# 用法:
#   python3 viz_endpoints_posearray_dir.py \
#       --endpoints /track/endpoints_posearray \
#       --udir /track/endpoints_dir \
#       --target /target_pose \
#       --window 8 \
#       --rate 15
#
import argparse
import threading
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3Stamped

import matplotlib
matplotlib.use("TkAgg")  # 也可以换成 Qt5Agg
import matplotlib.pyplot as plt


class VizNode:
    def __init__(self, endpoints_topic, u_topic, target_topic, window_sec, rate_hz):
        self.endpoints_topic = endpoints_topic
        self.u_topic = u_topic
        self.target_topic = target_topic
        self.window_sec = window_sec
        self.rate_hz = rate_hz

        self.lock = threading.Lock()
        self.buf = deque()  # (t, x, y)
        self.A = None
        self.B = None
        self.u = None
        self.have_u = False

        rospy.Subscriber(self.endpoints_topic, PoseArray, self.cb_endpoints)
        rospy.Subscriber(self.u_topic, Vector3Stamped, self.cb_u)
        rospy.Subscriber(self.target_topic, PoseStamped, self.cb_target)

        # Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.sc_points = self.ax.scatter([], [], s=14, label="Target points")
        (self.line_fit,) = self.ax.plot([], [], lw=2, label="Segment A—B")
        self.ptA = self.ax.scatter([], [], s=60, marker='x', label="A")
        self.ptB = self.ax.scatter([], [], s=60, marker='x', label="B")
        self.arrow = None
        self.text_info = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes,
                                      ha="left", va="top", fontsize=9)

        self.ax.set_title("Endpoints + Direction + Target")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)
        self.ax.legend(loc="best")

    def cb_target(self, msg: PoseStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        x = msg.pose.position.x
        y = msg.pose.position.y
        with self.lock:
            self.buf.append((t, x, y))
            # drop old
            t_now = t
            while self.buf and (t_now - self.buf[0][0]) > self.window_sec:
                self.buf.popleft()

    def cb_endpoints(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return
        with self.lock:
            A = msg.poses[0].position
            B = msg.poses[1].position
            self.A = np.array([A.x, A.y], dtype=float)
            self.B = np.array([B.x, B.y], dtype=float)
            # 如果还没收到 u，就临时用 B-A
            if not self.have_u:
                AB = self.B - self.A
                n = np.linalg.norm(AB)
                self.u = (AB / n) if n > 1e-9 else np.array([1.0, 0.0])

    def cb_u(self, msg: Vector3Stamped):
        with self.lock:
            self.u = np.array([msg.vector.x, msg.vector.y], dtype=float)
            n = np.linalg.norm(self.u)
            if n > 1e-9:
                self.u = self.u / n
                self.have_u = True

    def step_plot(self):
        with self.lock:
            pts = np.array([(x, y) for (_, x, y) in self.buf], dtype=float)
            A = None if self.A is None else self.A.copy()
            B = None if self.B is None else self.B.copy()
            u = None if self.u is None else self.u.copy()

        if len(pts) > 0:
            self.sc_points.set_offsets(pts)
            mn = pts.min(axis=0) - 0.2
            mx = pts.max(axis=0) + 0.2
            if A is not None and B is not None:
                mn = np.minimum(mn, np.minimum(A, B) - 0.2)
                mx = np.maximum(mx, np.maximum(A, B) + 0.2)
            self.ax.set_xlim(mn[0], mx[0])
            self.ax.set_ylim(mn[1], mx[1])

        if A is not None and B is not None:
            self.line_fit.set_data([A[0], B[0]], [A[1], B[1]])
            self.ptA.set_offsets([A])
            self.ptB.set_offsets([B])

            if self.arrow is not None:
                self.arrow.remove()
                self.arrow = None
            if u is None and A is not None and B is not None:
                AB = B - A
                n = np.linalg.norm(AB)
                u = (AB / n) if n > 1e-9 else None
            if u is not None:
                length = max(0.15 * np.linalg.norm(B - A), 0.2)
                self.arrow = self.ax.arrow(A[0], A[1], u[0]*length, u[1]*length,
                                           head_width=0.05, head_length=0.08, length_includes_head=True,
                                           alpha=0.85)
            L = np.linalg.norm(B - A)
            self.text_info.set_text(f"A=({A[0]:.2f},{A[1]:.2f})  B=({B[0]:.2f},{B[1]:.2f})  L={L:.2f}")
        else:
            self.line_fit.set_data([], [])
            self.ptA.set_offsets([])
            self.ptB.set_offsets([])
            if self.arrow is not None:
                self.arrow.remove()
                self.arrow = None
            self.text_info.set_text("Waiting for endpoints...")

        self.fig.canvas.draw_idle()

    def run(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step_plot()
            plt.pause(0.001)
            r.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoints", type=str, default="/track/endpoints_posearray")
    parser.add_argument("--udir", type=str, default="/track/endpoints_dir")
    parser.add_argument("--target", type=str, default="/target_pose")
    parser.add_argument("--window", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=15.0)
    args = parser.parse_args()

    rospy.init_node("viz_endpoints_posearray_dir", anonymous=True)
    node = VizNode(args.endpoints, args.udir, args.target, args.window, args.rate)
    node.run()


if __name__ == "__main__":
    main()
