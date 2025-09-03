#pragma once
#include "offboard.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
// 工具模块实现：distance / init_params（参数提取）
// 定义点集
// 悬停 / 设置目标点

// 参数提取（写入 searching_points / obstacle_zone_points）
void init_params(ros::NodeHandle &nh);
// 悬停
void hovering(float z, float time, bool if_exit, ros::Rate &rate);
// 平移到目标点
void set_pose(float x, float y, float z);
// 计算两点距离
double distance(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::Point &point);

struct FollowParams
{
    double height = 8.0;       // 跟随高度 h (m)
    double tau = 0.25;         // 超前补偿 τ (s)
    double kp_xy = 1.2;        // XY 比例
    double kd_xy = 0.0;        // XY 微分（若可读到 UAV 速度再用）
    double kp_z = 1.0;         // Z 比例
    double vmax_xy = 5.0;      // 水平限速
    double vmax_z = 1.0;       // 垂直限速
    double lost_timeout = 0.6; // 目标丢失阈值 (s)
};

//计算“正上方跟随”的速度控制指令：速度XY + 位置Z（高度保持）
void computeOverheadVelocityCmd(const geometry_msgs::Point &tgt_vel, // 目标速度（请从你的视觉/KF得到）
                                double now_sec,
                                double target_stamp_sec,              // 目标状态时间戳（Odometry.header.stamp）
                                geometry_msgs::Twist &output_vel,     // 输出：速度命令（XY速度 + Z速度）
                                geometry_msgs::Point &output_ref_xy); // 输出：参考点，仅用于调试/可视化