#pragma once
#include "offboard.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
// 工具模块实现：distance / init_params（参数提取）

// 声明点集
extern std::vector<geometry_msgs::Point> searching_points;
extern std::vector<geometry_msgs::Point> obstacle_zone_points;

extern std::queue<RetryPoint> retry_searching_points;            // 针对searching点的重试队列
extern std::queue<geometry_msgs::Point> retry_navigating_points; // 针对避障点的重试队列

extern Target targetArray[7];
// 参数提取（写入 searching_points / obstacle_zone_points）
void init_params(ros::NodeHandle &nh);
// 悬停
void hovering(float z, float time, bool if_exit, ros::Rate &rate);
// 平移到目标点
void set_and_pub_pose(float x, float y, float z);
// 发布导航状态和导航点
void set_and_pub_nav(float x, float y, float z);
// 计算两点距离
double distance(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::Point &point);

void addPose(const geometry_msgs::PoseStamped &pose, std::deque<geometry_msgs::PoseStamped> g_pose_buffer, ros::Duration g_window_len);

bool getPoseAt(const ros::Time &t, geometry_msgs::PoseStamped &out, std::deque<geometry_msgs::PoseStamped> g_pose_buffer, ros::Duration g_window_len);

struct FollowParams
{
    double height = 1.5; // 固定高度（你要的 1.5 m）
    double tau = 0.25;   // 常规超前补偿
    double kp_xy = 1.2;
    double kd_xy = 0.0;
    double vmax_xy = 5.0;
    double vmax_z = 0.0; // 固定高度时不用
    double lost_timeout = 0.6;

    // —— 直线往返增强参数 ——
    double k_perp = 1.0;     // 垂直偏差的纠正增益（把 UAV 拉回到直线上）
    double tau_turn = 0.10;  // 掉头段的更小超前量
    double d_turn = 1.0;     // 认为“接近端点”的 along-track 距离阈值（m）
    double ema_alpha = 0.2;  // 速度/方向的指数滑动平均系数（抖动越大取值越大）
    double v_slow_cap = 2.0; // 掉头段限速（m/s），避免过冲
    double v_thresh = 0.3;   // 判定“速度很慢≈掉头中”的阈值
};

// 计算“正上方跟随”的速度控制指令：速度XY + 位置Z（高度保持）
void computeOverheadVelocityCmd(const geometry_msgs::Vector3 &tgt_vel, // 目标速度（请从你的视觉/KF得到）
                                ros::Time now_sec,
                                ros::Time target_stamp_sec,           // 目标状态时间戳（Odometry.header.stamp）
                                geometry_msgs::Twist &output_vel,     // 输出：速度命令（XY速度 + Z速度）
                                geometry_msgs::Point &output_ref_xy); // 输出：参考点，仅用于调试/可视化