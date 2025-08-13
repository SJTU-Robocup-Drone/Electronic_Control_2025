#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <cmath>

bool is_bombing = false; // 检测是否投弹
bool is_returning = false;// 检测是否正在返航
bool is_done = false;
int current_index = 0;
int read_pos = 0; // 当前已经读取过的行数
geometry_msgs::PoseStamped target_pose;
ros::Time last_request;

std::string filename = "/home/amov/board_ws/src/board_ros/scripts/detection_log.txt";                           // 视觉组传来的文件路径
std::string line;                                                                                               // 字符行
std::string field;                                                                                              // 字段
std::vector<std::string> fields;                                                                                // 用于储存所有6个字段
double coordArray[6][2] = {{-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}}; // 表示5种靶标+降落区的坐标
// 第一个下标对应类型，第二个下标对应xy
// -100表示暂时没有坐标信息，-50表示已经投过了

geometry_msgs::PoseStamped current_pose; // 位姿信息
double yaw;                              // 航偏角
double coordX;                           // 当前的X坐标
double coordY;                           // 当前的Y坐标
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    coordX = current_pose.pose.position.x;
    coordY = current_pose.pose.position.y;
    yaw = tf2::getYaw(current_pose.pose.orientation); // tf2默认返回弧度制
} // 获取当前位姿信息
// 为了方便计算，我们暂时不考虑其他变量，只考虑x,y和yaw
void man_check_cb(const std_msgs::Int32::ConstPtr &msg)
{
    is_bombing = true;
}
void return_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    is_returning = msg->data;
}
void is_done_cb(const std_msgs::Bool::ConstPtr &msg)
{
    is_done = msg->data;
}

void clear_file()
{
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }
    file.close();
}

void read_file()
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    for(int i = 0; i < read_pos; ++i){
        std::getline(file,line);
    }//跳过前面的读过的行

    while (std::getline(file, line))
    {
        // 对读取到的line进行处理 ；假设每行有6个字段和5个逗号；5个字段分别是时间戳，类别，置信度，相对坐标x，相对坐标y，相对坐标z
        int pos[6] = {-1}; // 通过检测逗号确定各字段
        for (int i = 1; i < 6; i++)
        {
            pos[i] = line.find(',', pos[i - 1] + 1);
            field = line.substr(pos[i - 1] + 1, pos[i] - pos[i - 1] - 1);
            fields.push_back(field);
        }
        field = line.substr(pos[5] + 1, std::string::npos);
        fields.push_back(field);
        int type = -1; // 类型编号，0~5有效
        if (fields[1] == "tent")
        {
            type = 0;
        }
        else if (fields[1] == "bunker")
        {
            type = 1;
        }
        else if (fields[1] == "bridge")
        {
            type = 2;
        }
        else if (fields[1] == "car")
        {
            type = 3;
        }
        else if (fields[1] == "tank")
        {
            type = 4;
        }
        else if (fields[1] == "Helicopter")
        { // 这里的首字母是大写，后续应注意是否真的如此
            type = 5;
        }
        if (type >= 0 && coordArray[type][0] != -50)
        {                                                                                             // 若靶标已投过就不再写入坐标
            coordArray[type][0] = (coordX + stod(fields[3]) * sin(yaw) - stod(fields[4]) * cos(yaw)); // 对应物体的X坐标
            coordArray[type][1] = (coordY - stod(fields[3]) * cos(yaw) - stod(fields[4]) * sin(yaw)); // 对应物体的y坐标
            std::cout << type << " " << coordArray[type][0] << " " << coordArray[type][1] << std::endl;
        }
        fields.clear();
    }
    // 读完所有文件
    file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "readfile");
    ros::NodeHandle nh;

    target_pose.pose.position.z = -1;

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target", 10);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber man_check_sub = nh.subscribe<std_msgs::Int32>("/manba_input", 10, man_check_cb);
    ros::Subscriber return_state_sub = nh.subscribe<std_msgs::Bool>("/return_state", 10, return_state_cb);
    ros::Subscriber is_done_sub = nh.subscribe<std_msgs::Bool>("/done_state", 10, is_done_cb);
    last_request = ros::Time::now();
    ros::Rate rate(20.0);
    clear_file(); // 清空文件内容，避免读取到旧数据
    while (ros::ok())
    {
        ros::spinOnce();
        read_file();

        bool is_found = false;
        if(is_returning){
            if (coordArray[5][0] != -100 && coordArray[5][0] != -50)
            {
                is_found = true;
                ROS_INFO("landing area found, location: (%.2f, %.2f)", coordArray[5][0], coordArray[5][1]);
                target_pose.header.frame_id = "map";
                target_pose.header.stamp = ros::Time::now();
                target_pose.pose.position.x = coordArray[5][0];
                target_pose.pose.position.y = coordArray[5][1];
                target_pose.pose.position.z = 1.0;
                target_pub.publish(target_pose);
                current_index = 5;
                break;
            }
        }
        else{
            for (int i = 4; i >= 0; i--)
            { // 优先投分数高的靶标
            if (coordArray[i][0] != -100 && coordArray[i][0] != -50)
            {
                is_found = true;
                ROS_INFO("target %d found, location: (%.2f, %.2f)", i + 1, coordArray[i][0], coordArray[i][1]);
                target_pose.header.frame_id = "map";
                target_pose.header.stamp = ros::Time::now();
                target_pose.pose.position.x = coordArray[i][0];
                target_pose.pose.position.y = coordArray[i][1];
                target_pose.pose.position.z = 1.0;
                if(i == 0 && !is_done) target_pose.pose.position.z = -1; // 没搜索完时不投最低分靶标
                target_pub.publish(target_pose);
                current_index = i;
                break;
            }
            }
        } // 对投弹阶段or返航阶段，执行两套提供目标点的逻辑
        
        if (!is_found)
        {
            target_pose.header.frame_id = "map";
            target_pose.header.stamp = ros::Time::now();
            target_pose.pose.position.z = -1;
            target_pub.publish(target_pose);
        }

        if (is_bombing)
        { // 投弹时对当前靶标标记已投过
            coordArray[current_index][0] = -50;
            coordArray[current_index][1] = -50;
            is_bombing = false;
        }

        rate.sleep();
    }
    return 0;
}
