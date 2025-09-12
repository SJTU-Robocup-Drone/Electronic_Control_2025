#pragma once
#include "offboard.h"
// 状态机模块实现

// 状态机函数
void takeoff(ros::Rate &rate);
void overlooking(ros::Rate &rate);
void searching(ros::Rate &rate);
void bomb_navigating(ros::Rate &rate);
void adjusting(ros::Rate &rate);
void bombing(ros::Rate &rate);
void obstacle_avoiding(ros::NodeHandle &nh, ros::Rate &rate);
void descending(ros::Rate &rate);
void landing(ros::Rate &rate);
void following(ros::Rate &rate);
void returning(ros::Rate &rate);
void detecting(ros::Rate &rate);