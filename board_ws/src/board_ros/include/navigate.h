#pragma once
#include "offboard.h"
// 导航模块只实现：pose_cb / state_cb / nav_check_cb / init_nav_interfaces

// 导航回调函数
void pose_cb(const nav_msgs::Odometry::ConstPtr &msg);
// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr &msg);
// 导航检查回调函数
void nav_check_cb(const mavros_msgs::PositionTarget::ConstPtr &msg);
// 导航话题/服务初始化（变量名不变）
void init_nav_interfaces(ros::NodeHandle &nh);