#pragma once
#include "offboard.h"
// 视觉模块只实现：target_cb / visionCallback / computeAverageVelocity / predictNextPosition

// 视觉回调函数
void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
// 视觉储存入栈函数
void visionCallback(const geometry_msgs::PoseStamped &msg);
// 计算平均速度函数
geometry_msgs::Vector3 computeAverageVelocity();
// 预测下一位置函数
geometry_msgs::Point predictNextPosition(double predict_dt);