#pragma once
#include "offboard.h"

// 初始化视觉话题/服务函数
void init_vis_interfaces(ros::NodeHandle &nh);
// 定时视觉信息存储和刷新函数
void process_target_cb();
// 视觉信息最终纠偏与检查函数
void receive_target();
// 视觉储存入栈函数
void visionCallback(const geometry_msgs::PoseStamped &msg);
// 视觉目标回调函数
void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
// 计算平均速度函数
geometry_msgs::Vector3 computeAverageVelocity();
// 预测下一位置函数
geometry_msgs::Point predictNextPosition(double predict_dt);