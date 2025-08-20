#include "offboard/state_machine.h"

// 定义剩余全局
MissionState mission_state = TAKEOFF;

geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped pose;
geometry_msgs::Twist vel;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped nav_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_nav_pose;
geometry_msgs::Point last_nav_point;
geometry_msgs::Point first_target_point;
geometry_msgs::Point last_target_point;
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
std_msgs::Bool nav_state_msg;
std_msgs::Bool vision_state_msg;
std_msgs::Bool return_state_msg;
std_msgs::Bool is_done_msg;
bool is_stuck = false;
bool is_once_stuck = false;
bool is_return = false;
bool is_vision_right = true;
bool adjust_has_target = false;
int vision_bias_cnt = 0;
int target_index = 0;
std_msgs::Int32 target_index_msg;
ros::Time last_request;
ros::Time takeoff_request;
ros::Time nav_request;
bool is_takeoff = false;
double offset[3][2] = {{0, 0.18}, {0.18, 0}, {0, -0.18}};
bool is_moving_target = false;

const double threshold_distance = 0.1;

void state_machine_spin_once(ros::Rate &rate)
{
    ros::spinOnce();

    switch (mission_state)
    {
    case TAKEOFF:
    {
        // todo
        break;
    }
    case OVERLOOKING:
    {
        // todo
        break;
    }
    case SEARCHING:
    {
        // todo
        break;
    }
    case BOMB_NAVIGATING:
    {
        // todo
        break;
    }
    case ADJUSTING:
    {
        // todo
        break;
    }
    case BOMBING:
    {
        // todo
        break;
    }
    case OBSTACLE_AVOIDING:
    {
        // todo
        break;
    }
    case DESCENDING:
    {
        // todo
        break;
    }
    case LANDING:
    {
        // todo
        break;
    }
    case FOLLOW:
    {
        // todo
        break;
    }
    case RETURNING:
    {
        // todo
        break;
    }
    }
}
