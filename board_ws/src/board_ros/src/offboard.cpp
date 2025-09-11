#include "offboard.h"
#include "communication.h"
#include "function.h"
#include "state_machine.h"
#include "vision.h"

int main(int argc, char **argv)
{
    // ROS 节点初始化
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    ros::Rate follow_rate(20);
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
            takeoff(rate);
            break;
        }
        case OVERLOOKING:
        {
            overlooking(rate);
            break;
        }
        case SEARCHING:
        {
            searching(rate);
            break;
        }
        case BOMB_NAVIGATING:
        {
            bomb_navigating(rate);
            break;
        }
        case ADJUSTING:
        {
            adjusting(rate);
            break;
        }
        case BOMBING:
        {
            bombing(rate);
            break;
        }
        case OBSTACLE_AVOIDING:
        {
            obstacle_avoiding(nh,rate);
            break;
        }
        case DESCENDING:
        {
            descending(rate);
            break;
        }
        case LANDING:
        {
            landing(rate);
            break;
        }
        case FOLLOWING:
        {
            following(follow_rate);
            break;
        }
        case RETURNING:
        {
            returning(rate);
            break;
        }
        }
    }
    return 0;
}
