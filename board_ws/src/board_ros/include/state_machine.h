#pragma once
#include "offboard.h"
// 状态机模块实现

// 状态机函数
void takeoff();
void overlooking();
void searching();
void bomb_navigating();
void adjusting();
void bombing();
void obstacle_avoiding();
void descending();
void landing();
void following();
void returning();