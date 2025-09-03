#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <deque>
#include <vector>

// —— 状态（原名保留）——
enum MissionState
{
    TAKEOFF,
    OVERLOOKING,
    SEARCHING,
    BOMB_NAVIGATING,
    ADJUSTING,
    BOMBING,
    OBSTACLE_AVOIDING,
    DESCENDING,
    LANDING,
    FOLLOW,
    RETURNING
};

// —— 全局对象（用 extern 声明）——
extern MissionState mission_state;

extern ros::Publisher local_pos_pub;
extern ros::Publisher local_vel_pub;
extern ros::Publisher nav_goal_pub;
extern ros::Publisher nav_state_pub;
extern ros::Publisher vision_state_pub;
extern ros::Publisher return_state_pub;
extern ros::Publisher is_done_pub;
extern ros::Subscriber target_sub;
extern ros::Publisher param_set_pub;
extern ros::Publisher manba_pub;
extern ros::Subscriber local_pos_sub;
extern ros::Subscriber state_sub;
extern ros::Subscriber nav_check_sub;
extern ros::ServiceClient arming_client;
extern ros::ServiceClient set_mode_client;

extern geometry_msgs::PoseStamped initial_pose;
extern geometry_msgs::PoseStamped pose;
extern geometry_msgs::Twist vel;
extern geometry_msgs::PoseStamped target_pose;
extern geometry_msgs::PoseStamped nav_pose;
extern geometry_msgs::PoseStamped current_pose;
extern geometry_msgs::PoseStamped current_nav_pose;
extern geometry_msgs::Point last_nav_point;
extern geometry_msgs::Point first_target_point;
extern geometry_msgs::Point last_target_point;

extern mavros_msgs::State current_state;
extern mavros_msgs::CommandBool arm_cmd;
extern mavros_msgs::SetMode offb_set_mode;

extern std_msgs::Bool nav_state_msg;
extern std_msgs::Bool vision_state_msg;
extern std_msgs::Bool return_state_msg;
extern std_msgs::Bool is_done_msg;
extern std_msgs::Bool param_set_msg;

extern bool is_stuck;
extern bool is_once_stuck;
extern bool is_return;
extern bool is_vision_right;
extern bool adjust_has_target;

extern int vision_bias_cnt;
extern int target_index;

extern std_msgs::Int32 target_index_msg;

extern ros::Time last_request;
extern ros::Time takeoff_request;
extern ros::Time nav_request;

extern bool is_takeoff;
extern bool is_moving_target;
extern bool is_param_set;

extern double offset[3][2];

extern const double threshold_distance;

// 搜索/避障点（原名保留）
extern std::vector<geometry_msgs::Point> searching_points;
extern int searching_index;

extern std::vector<geometry_msgs::Point> obstacle_zone_points;
extern int obstacle_zone_index;

// 视觉历史缓冲
struct TimedPose
{
    geometry_msgs::Point pos;
    ros::Time stamp;
};

extern std::deque<TimedPose> history_;
extern const size_t MAX_HISTORY;
