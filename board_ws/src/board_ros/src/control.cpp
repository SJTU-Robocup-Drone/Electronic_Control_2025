#include "offboard.h"
#include "navigate.h"
#include "utils.h"
#include "state_machine.h"
#include "vision.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    // 视觉初值
    target_pose.pose.position.z = -1;

    // 初始化话题/服务
    init_nav_interfaces(nh);
    // 读取参数到 searching_points/obstacle_zone_points
    init_params(nh);

    // 等 FCU
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.0;
    for (int i = 0; i < 100 && ros::ok(); ++i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    initial_pose = current_pose;
    last_nav_point = current_pose.pose.position;

    // 主循环：每步推进一次状态机
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        switch (mission_state)
        {
        case TAKEOFF:
        {
            takeoff();
            break;
        }
        case OVERLOOKING:
        {
            overlooking();
            break;
        }
        case SEARCHING:
        {
            searching();
            break;
        }
        case BOMB_NAVIGATING:
        {
            bomb_navigating();
            break;
        }
        case ADJUSTING:
        {
            adjusting();
            break;
        }
        case BOMBING:
        {
            bombing();
            break;
        }
        case OBSTACLE_AVOIDING:
        {
            obstacle_avoiding();
            break;
        }
        case DESCENDING:
        {
            descending();
            break;
        }
        case LANDING:
        {
            landing();
            break;
        }
        case FOLLOW:
        {
            following();
            break;
        }
        case RETURNING:
        {
            returning();
            break;
        }
        }
    }
    return 0;
}
