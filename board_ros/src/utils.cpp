#include "offboard/utils.h"
#include <cmath>

// 定义点集
std::vector<geometry_msgs::Point> searching_points;
int searching_index = 0;

std::vector<geometry_msgs::Point> obstacle_zone_points;
int obstacle_zone_index = 0;

double distance(const geometry_msgs::PoseStamped &current_pose_, const geometry_msgs::Point &point)
{
    const auto &p = current_pose_.pose.position;
    const double dx = p.x - point.x, dy = p.y - point.y, dz = p.z - point.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 读取参数到既有变量名
void init_params(ros::NodeHandle &nh)
{
    std::vector<double> searching_x_points, searching_y_points, searching_z_points;
    if (nh.getParam("searching_points/x", searching_x_points) &&
        nh.getParam("searching_points/y", searching_y_points) &&
        nh.getParam("searching_points/z", searching_z_points))
    {

        searching_points.clear();
        for (size_t i = 0; i < searching_x_points.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = searching_x_points[i];
            point.y = searching_y_points[i];
            point.z = searching_z_points[i];
            searching_points.push_back(point);
        }
    }
    std::vector<double> obstacle_x_points, obstacle_y_points, obstacle_z_points;
    if (nh.getParam("obstacle_points/x", obstacle_x_points) &&
        nh.getParam("obstacle_points/y", obstacle_y_points) &&
        nh.getParam("obstacle_points/z", obstacle_z_points))
    {

        obstacle_zone_points.clear();
        for (size_t i = 0; i < obstacle_x_points.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = obstacle_x_points[i];
            point.y = obstacle_y_points[i];
            point.z = obstacle_z_points[i];
            obstacle_zone_points.push_back(point);
        }
    }
}
