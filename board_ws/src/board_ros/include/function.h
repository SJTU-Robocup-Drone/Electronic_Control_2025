#pragma once
#include "offboard.h"
// 工具模块实现：distance / init_params（参数提取）
// 定义点集
// 悬停 / 设置目标点

// 参数提取（写入 searching_points / obstacle_zone_points）
void init_params(ros::NodeHandle &nh);
// 悬停
void hovering(float z, float time, bool if_exit);
// 平移到目标点
void set_pose(float x, float y, float z);
// 计算两点距离
double distance(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::Point &point);