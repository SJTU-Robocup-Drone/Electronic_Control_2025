#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# viz_endpoints_target.py — 可视化：端点(PoseArray) + 方向(Vector3Stamped) + 目标点(PoseStamped)

import argparse
import threading
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3Stamped

import matplotlib
matplotlib.use("TkAgg")  # 或 Qt5Agg，按环境选择
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

        # Matplotlib
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.sc_points = self.ax.scatter(np.empty((0, 2)), np.empty((0,)))
        (self.line_fit,) = self.ax.plot([], [], lw=2, label="Segment A—B")
        self.ptA = self.ax.scatter(np.empty((0, 2)), np.empty((0,)), s=60, marker='x', label="A")
        self.ptB = self.ax.scatter(np.empty((0, 2)), np.empty((0,)), s=60, marker='x', label="B")
        self.arrow = None
        self.text_info = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes,
                                      ha="left", va="top", fontsize=9)

        self.ax.set_title("Endpoints + Direction + Target")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)
        self.ax.legend(loc="best")

    # --- 回调 ---
    def cb_target(self, msg: PoseStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        x, y = msg.pose.position.x, msg.pose.position.y
        with self.lock:
            self.buf.append((t, x, y))
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

    # --- 绘制一步 ---
    def step_plot(self):
        with self.lock:
            pts = np.array([(x, y) for (_, x, y) in self.buf], dtype=float) if len(self.buf) else np.empty((0, 2))
            A = None if self.A is None else self.A.copy()
            B = None if self.B is None else self.B.copy()
            u = None if self.u is None else self.u.copy()

        # 目标点散点
        self.sc_points.set_offsets(pts if len(pts) else np.empty((0, 2)))

        # 自动缩放（有数据再设）
        if len(pts) or (A is not None and B is not None):
            mins = []
            maxs = []
            if len(pts):
                mins.append(pts.min(axis=0)); maxs.append(pts.max(axis=0))
            if A is not None:
                mins.append(A); maxs.append(A)
            if B is not None:
                mins.append(B); maxs.append(B)
            mn = np.min(np.stack(mins, axis=0), axis=0) - 0.2
            mx = np.max(np.stack(maxs, axis=0), axis=0) + 0.2
            self.ax.set_xlim(mn[0], mx[0])
            self.ax.set_ylim(mn[1], mx[1])

        # 端点与线段
        if A is not None and B is not None:
            self.line_fit.set_data([A[0], B[0]], [A[1], B[1]])
            self.ptA.set_offsets(np.array([[A[0], A[1]]]))
            self.ptB.set_offsets(np.array([[B[0], B[1]]]))

            if self.arrow is not None:
                self.arrow.remove()
                self.arrow = None
            if u is None:
                AB = B - A
                n = np.linalg.norm(AB)
                u = (AB / n) if n > 1e-9 else None
            if u is not None:
                # 缩小箭头（长度=线段长度的5%，最小0.1m；箭头宽/长也更小）
                seg_len = float(np.linalg.norm(B - A))
                length = max(0.05 * seg_len, 0.1)
                self.arrow = self.ax.arrow(A[0], A[1], u[0]*length, u[1]*length,
                                           head_width=0.03, head_length=0.05,
                                           length_includes_head=True, alpha=0.85)
            L = float(np.linalg.norm(B - A))
            self.text_info.set_text(f"A=({A[0]:.2f},{A[1]:.2f})  B=({B[0]:.2f},{B[1]:.2f})  L={L:.2f}")
        else:
            self.line_fit.set_data([], [])
            self.ptA.set_offsets(np.empty((0, 2)))
            self.ptB.set_offsets(np.empty((0, 2)))
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
    parser = argparse.ArgumentParser(description="可视化 Endpoints(PoseArray) + Direction(Vector3Stamped) + Target(PoseStamped)")
    parser.add_argument("--endpoints", type=str, default="/track/endpoints_posearray", help="PoseArray 端点话题")
    parser.add_argument("--udir", type=str, default="/track/endpoints_dir", help="方向向量话题(Vector3Stamped)")
    parser.add_argument("--target", type=str, default="/target", help="目标点话题(PoseStamped)")
    parser.add_argument("--window", type=float, default=30.0, help="滑窗时间长度 (s)")
    parser.add_argument("--rate", type=float, default=30.0, help="刷新频率 (Hz)")
    args = parser.parse_args()

    rospy.init_node("viz_endpoints_target", anonymous=True)
    node = VizNode(args.endpoints, args.udir, args.target, args.window, args.rate)
    node.run()


if __name__ == "__main__":
    main()
