#include "function.h"
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/Point.h>

geometry_msgs::Point createPoint(double x, double y, double z) {
   geometry_msgs::Point p;
   p.x = x; p.y = y; p.z = z;
   return p;
}
// 定义点集（会在全局范围内用到）
std::vector<geometry_msgs::Point> searching_points;
std::vector<geometry_msgs::Point> obstacle_zone_points;

void hovering(float z, float time, bool if_exit, ros::Rate &rate)
{
    last_request = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = current_pose.pose.position.x;
    pose.pose.position.y = current_pose.pose.position.y;
    pose.pose.position.z = z;
    last_request = ros::Time::now();
    ROS_INFO("Hovering at height %f for %f seconds.", z, time);
    while (ros::ok() && ros::Time::now() - last_request < ros::Duration(time))
    {
        ros::spinOnce();
        pose.header.stamp = ros::Time::now(); // 更新时间戳
        local_pos_pub.publish(pose);          // 保持悬停
        rate.sleep();
        if (if_exit == true)
        {
            vision_state_pub.publish(vision_state_msg);
            if (target_pose.pose.position.z != -1)
                break;
        }
    }
}

void set_and_pub_pose(float x, float y, float z)
{
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    local_pos_pub.publish(pose);
}

void set_and_pub_nav(float x, float y, float z)
{
    nav_state_msg.data = true;
    nav_state_pub.publish(nav_state_msg);
    nav_pose.header.frame_id = "map";
    nav_pose.header.stamp = ros::Time::now();
    nav_pose.pose.position.x = x;
    nav_pose.pose.position.y = y;
    nav_pose.pose.position.z = z;
    nav_goal_pub.publish(nav_pose);
    nav_request = ros::Time::now();
}

double distance(const geometry_msgs::PoseStamped &current_pose_, const geometry_msgs::Point &point)
{
    const auto &p = current_pose_.pose.position;
    const double dx = p.x - point.x, dy = p.y - point.y, dz = p.z - point.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 读取参数到既有变量名
void init_params(ros::NodeHandle &nh)
{
    std::string filename = "/home/michmax/drone/board_ws/src/board_ros/points/param.txt";
    std::string line;
    std::string valid_str;
    int pos1 = 0;
    int pos2 = 0;

    std::ifstream infile(filename);
    if(!infile.is_open()){
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    int num_of_searching_points = 0;
    std::getline(infile,line);
    pos1 = line.find("= ") + 1;
    pos2 = line.find(';');
    valid_str = line.substr(pos1 + 1,pos2 - pos1 - 1);
    num_of_searching_points = stoi(valid_str);
    for(int i = 0; i < num_of_searching_points; ++i){
        float point[3];
        for(int j = 0; j < 3; ++j){
            std::getline(infile,line);
            pos1 = line.find("= ");
            pos2 = line.find(';');
            valid_str = line.substr(pos1 + 1,pos2 - pos1 - 1);
            point[j] = stof(valid_str);
        }
        searching_points.push_back(createPoint(point[0],point[1],point[2]));
    }

    int num_of_obstacle_zone_points = 0;
    std::getline(infile,line);
    pos1 = line.find("= ") + 1;
    pos2 = line.find(';');
    valid_str = line.substr(pos1 + 1,pos2 - pos1 - 1);
    num_of_obstacle_zone_points = stoi(valid_str);
    for(int i = 0; i < num_of_obstacle_zone_points; ++i){
        float point[3];
        for(int j = 0; j < 3; ++j){
            std::getline(infile,line);
            pos1 = line.find("= ");
            pos2 = line.find(';');
            valid_str = line.substr(pos1 + 1,pos2 - pos1 - 1);
            point[j] = stof(valid_str);
        }
        obstacle_zone_points.push_back(createPoint(point[0],point[1],point[2]));
    }

    infile.close();

    // std::vector<double> searching_x_points, searching_y_points, searching_z_points;
    // if (nh.getParam("searching_points/x", searching_x_points) &&
    //     nh.getParam("searching_points/y", searching_y_points) &&
    //     nh.getParam("searching_points/z", searching_z_points))
    // {

    //     searching_points.clear();
    //     for (size_t i = 0; i < searching_x_points.size(); ++i)
    //     {
    //         geometry_msgs::Point point;
    //         point.x = searching_x_points[i];
    //         point.y = searching_y_points[i];
    //         point.z = searching_z_points[i];
    //         searching_points.push_back(point);
    //     }
    // }

    // std::vector<double> obstacle_x_points, obstacle_y_points, obstacle_z_points;
    // if (nh.getParam("obstacle_points/x", obstacle_x_points) &&
    //     nh.getParam("obstacle_points/y", obstacle_y_points) &&
    //     nh.getParam("obstacle_points/z", obstacle_z_points))
    // {

    //     obstacle_zone_points.clear();
    //     for (size_t i = 0; i < obstacle_x_points.size(); ++i)
    //     {
    //         geometry_msgs::Point point;
    //         point.x = obstacle_x_points[i];
    //         point.y = obstacle_y_points[i];
    //         point.z = obstacle_z_points[i];
    //         obstacle_zone_points.push_back(point);
    //     }
    // }
}

// 速度限幅
void velocity_limited(double &vx, double &vy, double vmax)
{
    const double n = std::sqrt(vx * vx + vy * vy);
    if (n > vmax && n > 1e-6)
    {
        const double s = vmax / n;
        vx *= s;
        vy *= s;
    }
}

// 数值死区函数
double clip(double v, double low, double high)
{
    return std::max(low, std::min(high, v));
}

void computeOverheadVelocityCmd(const geometry_msgs::Point &tgt_vel, // 目标速度（请从你的视觉/KF得到）
                                double now_sec,
                                double target_stamp_sec,             // 目标状态时间戳（Odometry.header.stamp）
                                geometry_msgs::Twist &output_vel,    // 输出：速度命令（XY速度 + Z速度）
                                geometry_msgs::Point &output_ref_xy) // 输出：参考点，仅用于调试/可视化
{
    FollowParams follow_param;
    // 目标是否新鲜
    const bool fresh = (now_sec - target_stamp_sec) < follow_param.lost_timeout;

    // 参考点（含超前补偿）
    output_ref_xy.x = target_pose.pose.position.x + tgt_vel.x * follow_param.tau;
    output_ref_xy.y = target_pose.pose.position.y + tgt_vel.y * follow_param.tau;
    output_ref_xy.z = 0.0;

    // xy误差
    const double ex = output_ref_xy.x - current_pose.pose.position.x;
    const double ey = output_ref_xy.y - current_pose.pose.position.y;

    // 速度计算 前馈 + P（如有 UAV 水平速度，可把 D 项补上：- kd*(uvx - v_ref)）
    double vx_cmd = fresh ? (tgt_vel.x + follow_param.kp_xy * ex) : 0.0;
    double vy_cmd = fresh ? (tgt_vel.y + follow_param.kp_xy * ey) : 0.0;

    // 速度限幅
    velocity_limited(vx_cmd, vy_cmd, follow_param.vmax_xy);

    // 高度保持（Z）
    const double ez = follow_param.height - current_pose.pose.position.z;
    double vz_cmd = clip(follow_param.kp_z * ez, -follow_param.vmax_z, follow_param.vmax_z);

    // 输出（只写值，不发布）
    output_vel = geometry_msgs::Twist();
    output_vel.linear.x = vx_cmd;
    output_vel.linear.y = vy_cmd;
    output_vel.linear.z = vz_cmd;
    // yaw 如需控制
}
