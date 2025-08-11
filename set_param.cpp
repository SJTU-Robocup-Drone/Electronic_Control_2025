#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/Point.h>
std::string filename = "param.txt";
std::string line;
std::string valid_str;
int pos1 = 0;
int pos2 = 0;
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
    nh.setParam("searching_points",searching_points);// 直接把vector上传到参数服务器

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
    nh.setParam("obstacle_zone_points",obstacle_zone_points);// 直接把vector上传到参数服务器

    // 读完所有文件
    infile.close();
    return 0;
}