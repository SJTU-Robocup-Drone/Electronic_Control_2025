#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/Point.h>
std::string filename = "/home/michmax/drone/board_ws/src/board_ros/points/param.txt";
std::string line;
std::string valid_str;
int pos1 = 0;
int pos2 = 0;

geometry_msgs::Point createPoint(double x, double y, double z) {
   geometry_msgs::Point p;
   p.x = x; p.y = y; p.z = z;
   return p;
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"set_param");
    ros::NodeHandle nh;

    std::ifstream infile(filename);
    if(!infile.is_open()){
        std::cerr << "无法打开文件: " << filename << std::endl;
        return -1;
    }

    std::vector<geometry_msgs::Point> searching_points;
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
    // 将 searching_points转换为三个独立的数组
    std::vector<double> x_points, y_points, z_points;
    for (const auto& point : searching_points) {
        x_points.push_back(point.x);
        y_points.push_back(point.y);
        z_points.push_back(point.z);
    }
    nh.setParam("searching_points/x", x_points);
    nh.setParam("searching_points/y", y_points);
    nh.setParam("searching_points/z", z_points);

    std::vector<geometry_msgs::Point> obstacle_zone_points;
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

    std::vector<double> obs_x_points, obs_y_points, obs_z_points;
    for (const auto& point : obstacle_zone_points) {
        obs_x_points.push_back(point.x);
        obs_y_points.push_back(point.y);
        obs_z_points.push_back(point.z);
    }
    nh.setParam("obstacle_points/x", obs_x_points);
    nh.setParam("obstacle_points/y", obs_y_points);
    nh.setParam("obstacle_points/z", obs_z_points);

    // 读完所有文件
    infile.close();
    return 0;
}