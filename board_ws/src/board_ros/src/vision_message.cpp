#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <cmath>
#include <map>
#include <string>

bool is_bombing = false;
bool is_returning = false;
bool is_done = false;
geometry_msgs::PoseStamped current_pose;
double yaw = 0;
double coordX = 0;
double coordY = 0;

// 目标类型映射
std::map<std::string, int> target_types = {
    {"tent", 0},
    {"bunker", 1},
    {"bridge", 2},
    {"car", 3},
    {"tank", 4},
    {"Helicopter", 5}
};

// 目标坐标存储
double coordArray[6][2] = {{-100, -100}, {-100, -100}, {-100, -100}, 
                          {-100, -100}, {-100, -100}, {-100, -100}};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
    coordX = current_pose.pose.position.x;
    coordY = current_pose.pose.position.y;
    yaw = tf2::getYaw(current_pose.pose.orientation);
}

void man_check_cb(const std_msgs::Int32::ConstPtr &msg) {
    is_bombing = true;
}

void return_state_cb(const std_msgs::Bool::ConstPtr &msg) {
    is_returning = msg->data;
}

void is_done_cb(const std_msgs::Bool::ConstPtr &msg) {
    is_done = msg->data;
}

void detection_cb(const geometry_msgs::PointStamped::ConstPtr &msg) {
    std::string target_name = msg->header.frame_id;
    double rel_x = msg->point.x;
    double rel_y = msg->point.y;
    
    // 查找目标类型
    auto it = target_types.find(target_name);
    if (it != target_types.end()) {
        int type = it->second;
        
        // 相对坐标转全局坐标（考虑无人机偏航角）
        double global_x = coordX + rel_x * sin(yaw) - rel_y * cos(yaw);
        double global_y = coordY - rel_x * cos(yaw) - rel_y * sin(yaw);
        
        // 只更新未投掷的目标
        if (coordArray[type][0] != -50) {
            coordArray[type][0] = global_x;
            coordArray[type][1] = global_y;
            
            ROS_INFO("更新目标 %s 坐标: (%.2f, %.2f) 延迟：%.3f", target_name.c_str(), global_x, global_y, (ros::Time::now() - msg->header.stamp).toSec());
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vision_message");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.z = -1;

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target", 10);
    ros::Publisher vision_control_pub = nh.advertise<std_msgs::Bool>("/vision_state", 10);
    
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber man_check_sub = nh.subscribe<std_msgs::Int32>("/manba_input", 10, man_check_cb);
    ros::Subscriber return_state_sub = nh.subscribe<std_msgs::Bool>("/return_state", 10, return_state_cb);
    ros::Subscriber is_done_sub = nh.subscribe<std_msgs::Bool>("/done_state", 10, is_done_cb);
    ros::Subscriber detection_sub = nh.subscribe<geometry_msgs::PointStamped>("/detection_results", 10, detection_cb);

    ros::Rate rate(20.0);
    
    // 启动视觉检测
    std_msgs::Bool vision_start;
    vision_start.data = true;
    vision_control_pub.publish(vision_start);
    ROS_INFO("启动视觉检测...");

    int current_index = 0;

    while (ros::ok()) {
        ros::spinOnce();

        bool is_found = false;
        
        if (is_returning) {
            // 返航阶段：寻找降落区
            if (coordArray[5][0] != -100 && coordArray[5][0] != -50) {
                is_found = true;
                ROS_INFO("降落区找到, 位置: (%.2f, %.2f)", coordArray[5][0], coordArray[5][1]);
                
                target_pose.header.frame_id = "map";
                target_pose.header.stamp = ros::Time::now();
                target_pose.pose.position.x = coordArray[5][0];
                target_pose.pose.position.y = coordArray[5][1];
                target_pose.pose.position.z = 1.0;
                
                target_pub.publish(target_pose);
                current_index = 5;
            }
        } else {
            // 投弹阶段：按优先级选择目标
            for (int i = 4; i >= 0; i--) {
                if (coordArray[i][0] != -100 && coordArray[i][0] != -50) {
                    is_found = true;
                    ROS_INFO("目标 %d 找到, 位置: (%.2f, %.2f)", i + 1, coordArray[i][0], coordArray[i][1]);
                    
                    target_pose.header.frame_id = "map";
                    target_pose.header.stamp = ros::Time::now();
                    target_pose.pose.position.x = coordArray[i][0];
                    target_pose.pose.position.y = coordArray[i][1];
                    target_pose.pose.position.z = 1.0;
                    
                    target_pub.publish(target_pose);
                    current_index = i;
                    break;
                }
            }
        }

        if (!is_found) {
            target_pose.header.frame_id = "map";
            target_pose.header.stamp = ros::Time::now();
            target_pose.pose.position.z = -1;
            target_pub.publish(target_pose);
        }

        if (is_bombing) {
            // 标记当前目标为已投掷
            coordArray[current_index][0] = -50;
            coordArray[current_index][1] = -50;
            is_bombing = false;
            ROS_INFO("目标 %d 已投掷", current_index + 1);
        }

        rate.sleep();
    }

    // 停止视觉检测
    std_msgs::Bool vision_stop;
    vision_stop.data = false;
    vision_control_pub.publish(vision_stop);
    ROS_INFO("停止视觉检测...");

    return 0;
}