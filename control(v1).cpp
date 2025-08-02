#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ego_planner/Bspline.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <queue>
#include <vector>

enum MissionState {
    TAKEOFF,
    OVERLOOKING, //飞至3.5米悬停寻找靶标
    SEARCHING,  // 视觉没找到靶标时遍历地图
    BOMB_NAVIGATING, // 导航到靶标
    ADJUSTING, // 让飞机进一步对准靶标
    BOMBING,
    OBSTACLE_AVOIDING, // 导航到障碍区并避障
    DESCENDING,
    LANDING,
};
MissionState mission_state = TAKEOFF;

ros::Publisher local_pos_pub;
ros::Publisher nav_goal_pub;
ros::Publisher vision_state_pub; // 发布视觉开启或关闭扫描的指令
ros::Subscriber target_sub; //从视觉对接节点的话题接收当前靶标的坐标
ros::Publisher manba_pub;
ros::Subscriber local_pos_sub;
ros::Subscriber nav_info_sub;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

geometry_msgs::PoseStamped pose; // 直接发送给飞控的目标点
geometry_msgs::PoseStamped target_pose; // 接收靶标位置，若找到则z坐标为避障飞行高度，z坐标为-1表示未找到靶标
geometry_msgs::PoseStamped nav_pose; // 发布到egoplanner的目标点
geometry_msgs::PoseStamped current_pose;
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
std_msgs::Bool vision_state_msg; // 发布视觉状态的消息
int target_index = 0; // 当前投弹索引，也可视作已经投弹的数量
std_msgs::Int32 target_index_msg;
std::queue<geometry_msgs::Point> points; // 用于存储导航轨迹
ros::Time last_request;

const double threshold_distance = 0.1; // 定义到达目标点的距离阈值


double distance(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::Point& point) {
    return std::sqrt(std::pow(current_pose.pose.position.x - point.x, 2) +
                     std::pow(current_pose.pose.position.y - point.y, 2) +
                     std::pow(current_pose.pose.position.z - point.z, 2));
}
geometry_msgs::Point createPoint(double x, double y, double z) {
   geometry_msgs::Point p;
   p.x = x; p.y = y; p.z = z;
   return p;
}
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose.header = msg->header;
    current_pose.pose = msg->pose.pose;
}
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    target_pose = *msg;
    if(target_pose.pose.position.z != -1) ROS_INFO("Target found at (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
}
void navigation_info_cb(const ego_planner::Bspline::ConstPtr& msg) {
   while(!points.empty()){
       points.pop();
   }//重置当前队列
   int num_of_points = msg->pos_pts.size(); //对未知长度的数组计算其长度
   ROS_INFO("Total points:%d",num_of_points);
   for(int i = 0; i < num_of_points; ++i){
       points.push(msg->pos_pts[i]);
       ROS_INFO("POINT %d: %.2f,%.2f,%.2f",i,msg->pos_pts[i].x,msg->pos_pts[i].y,msg->pos_pts[i].z);
   }//置入队列
}

std::vector<geometry_msgs::Point> searching_points = {
    createPoint(1.8, 2.0, 1.0),
    createPoint(1.8, -0.5, 1.0),
    createPoint(4.3, -0.5, 1.0),
    createPoint(4.3, 2.0, 1.0),
    createPoint(6.8, 2.0, 1.0),
    createPoint(6.8, -0.5, 1.0)
};
int searching_index = 0; // 当前搜索点的索引

std::vector<geometry_msgs::Point> obstacle_zone_points = { //存储避障区的入口和出口，出口即终点；注意这里需要提前打点
    createPoint(8.3, -3.3, 1.0),
    createPoint(0.05, -3.3, 1.0)
};
int obstacle_zone_index = 0;

int main(int argc,char *argv[]){
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    
    target_pose.pose.position.z = -1;

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    nav_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    manba_pub = nh.advertise<std_msgs::Int32>("/manba_input", 10);

    vision_state_pub = nh.advertise<std_msgs::Bool>("/vision_state", 10);
    target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target", 10, target_cb);

    local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/odom_high_freq", 10, pose_cb);
    nav_info_sub = nh.subscribe<ego_planner::Bspline>("/planning/bspline", 10, navigation_info_cb); //这个话题名需要后续根据真机上egoplanner的参数进一步确定
    state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_cb);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");

    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.0;
    for (int i = 0; i < 100 && ros::ok(); ++i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        ros::spinOnce(); //读取回调函数
        switch(mission_state){
            case TAKEOFF:
            {
                offb_set_mode.request.custom_mode = "OFFBOARD";
                arm_cmd.request.value = true;
                last_request = ros::Time::now();

                while(ros::ok() && mission_state == TAKEOFF) {
                    ros::spinOnce();
                    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                            ROS_INFO("Offboard enabled");
                        }
                        last_request = ros::Time::now();
                    }
                    else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                         if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.x = 0;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = 1.1;//更容易达到起飞条件
                    local_pos_pub.publish(pose);
                    rate.sleep();
                    if(current_pose.pose.position.z >= 0.8) {
                        mission_state = OVERLOOKING;
                        ROS_INFO("Takeoff complete. Starting overlooking.");
                        break; // 跳出循环，进入导航状态
                    }
                }
                break;
            }

            case OVERLOOKING:
            {
                vision_state_msg.data = true; // 开启视觉扫描
                ROS_INFO("Start overlooking for target. Rising to overlooking position...");
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 3.5; // 悬停高度
                while(ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance) {
                    ros::spinOnce();
                    vision_state_pub.publish(vision_state_msg); // 重复发布启动扫描指令，防止视觉节点没有接收到
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                ROS_INFO("Reached overlooking position. Scanning for target...");
                last_request = ros::Time::now();
                while(ros::ok() && ros::Time::now() - last_request < ros::Duration(3.0)) { // 等待视觉识别靶标，时间为3秒
                    ros::spinOnce();
                    local_pos_pub.publish(pose); // 保持悬停
                    rate.sleep();
                }
                
                ROS_INFO("Overlooking complete, descending to normal flight height.");
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.z = 1.0; // 降低高度到1米(原飞行高度)
                local_pos_pub.publish(pose);
                vision_state_msg.data = false; // 关闭视觉扫描
                while(ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance) {
                    ros::spinOnce();
                    vision_state_pub.publish(vision_state_msg); // 重复发布关闭扫描指令，防止视觉节点没有接收到
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                if(target_pose.pose.position.z != -1) { // 找到了靶标，进入靶标导航状态
                    mission_state = BOMB_NAVIGATING;
                    ROS_INFO("Target found, starting navigation to target.");
                } else {
                    mission_state = SEARCHING; // 没有找到靶标，进入搜索状态
                    ROS_INFO("No target found, starting searching.");
                }
                break;
            }

            case SEARCHING:
            {
                if(searching_index >= searching_points.size()) { // 如果所有搜索点都已访问，为了防止下标越界直接降落
                    ROS_WARN("All searching points reached, and this should not happen. Landing now.");
                    mission_state = DESCENDING;
                    break;
                }

                ROS_INFO("Searching for target %d...", searching_index + 1);
                //发布航点
                nav_pose.header.frame_id = "map";
                nav_pose.header.stamp = ros::Time::now();
                nav_pose.pose.position = searching_points[searching_index];
                nav_goal_pub.publish(nav_pose);

                // 轨迹跟踪与检查
                while (!points.empty() && mission_state == SEARCHING) {
                    ros::spinOnce();
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position = points.front();
                    local_pos_pub.publish(pose);
                    last_request = ros::Time::now();
                    double dist = distance(current_pose, pose.pose.position);
                    // ROS_INFO("Tracking point, dist = %.2f", dist);
                    if (distance(current_pose, pose.pose.position) < threshold_distance) {
                        points.pop();
                    }

                    // 到目标点后进入悬停搜索状态
                    if(distance(current_pose, nav_pose.pose.position) < threshold_distance) {
                        ROS_INFO("Reached searching point %d.", searching_index + 1);
                        searching_index++;
                        mission_state = OVERLOOKING;
                        break;
                    }
                    rate.sleep();
                }
                break;
            }

            case BOMB_NAVIGATING:
            {
                ROS_INFO("Navigating to target at (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
                nav_pose.header.frame_id = "map";
                nav_pose.header.stamp = ros::Time::now();
                nav_pose.pose.position = target_pose.pose.position;
                nav_goal_pub.publish(nav_pose); // 发布导航目标点

                // 轨迹跟踪与检查
                while (!points.empty() && mission_state == BOMB_NAVIGATING) {
                    ros::spinOnce();
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position = points.front();
                    local_pos_pub.publish(pose);
                    last_request = ros::Time::now();
                    double dist = distance(current_pose, pose.pose.position);
                    // ROS_INFO("Tracking point, dist = %.2f", dist);
                    if (distance(current_pose, pose.pose.position) < threshold_distance) {
                        points.pop();
                    }

                    if(distance(current_pose, nav_pose.pose.position) < threshold_distance) {
                        ROS_INFO("Reached bomb target %d.", target_index + 1);
                        mission_state = ADJUSTING; // 到达目标点后进入调整对准状态
                        break;
                    }
                    rate.sleep();
                }
                break;
            }

            case ADJUSTING:
            {
                ROS_INFO("Adjusting position to target...");
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = target_pose.pose.position.x;
                pose.pose.position.y = target_pose.pose.position.y;
                pose.pose.position.z = target_pose.pose.position.z;
                while (distance(current_pose, pose.pose.position) > threshold_distance/2.0 && ros::ok()) { // 临时减小距离阈值
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }
                mission_state = BOMBING; // 调整完成后进入投弹状态
                break;
            }

            case BOMBING:
            {
                ROS_INFO("Start bombing...");
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 0.3; // 先下降到较低高度再投弹
                while(ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                //到点后悬停0.5秒并投弹
                last_request = ros::Time::now();
                target_index_msg.data = target_index;
                while(ros::ok() && ros::Time::now() - last_request < ros::Time::Duration(0.5)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    manba_pub.publish(target_index_msg);
                    rate.sleep();
                }
                target_pose.pose.position.z = -1; // 防止视觉节点没有来得及发布新目标点或发布未找到目标点的消息导致重复导航和投弹

                ROS_INFO("Bombing %d done, rising to normal flight height.", target_index + 1);
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 1.0; 
                while(ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                if(++target_index >= 3) mission_state = OBSTACLE_AVOIDING;
                else if(target_pose.pose.position.z == -1) mission_state = OVERLOOKING;
                else mission_state = BOMB_NAVIGATING;
                break;
            }

            case OBSTACLE_AVOIDING: // 这部分代码直接借鉴了测试代码
            {
                // 发布航点
                if (obstacle_zone_index < obstacle_zone_points.size()) {
                    nav_pose.header.frame_id = "map";
                    nav_pose.header.stamp = ros::Time::now();
                    nav_pose.pose.position = obstacle_zone_points[obstacle_zone_index];
                    nav_goal_pub.publish(nva_pose);
                }

                // 轨迹跟踪与检查
                while (!points.empty() && mission_state == OBSTACLE_AVOIDING) {
                    ros::spinOnce();
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position = points.front();
                    local_pos_pub.publish(pose);
                    last_request = ros::Time::now();
                    double dist = distance(current_pose, pose.pose.position);
                    // ROS_INFO("Tracking point, dist = %.2f", dist);
                    if (distance(current_pose, pose.pose.position) < threshold_distance) {
                        points.pop();
                    }

                    // 到点时索引自增，并清空队列
                    if(distance(current_pose, obstacle_zone_points[obstacle_zone_index]) < threshold_distance) {
                        ROS_INFO("reached mid point!");
                        obstacle_zone_index++;
                        while(!points.empty()){
                            points.pop();
                        }
                    }

                    // 若到达全部目标点，则进入降落模式
                    if (obstacle_zone_index >= obstacle_zone_points.size()) {
                        ROS_INFO("All obstacle_zone_points reached!");
                        mission_state = DESCENDING;
                        break;
                    }

                    // 发布航点
                    if (obstacle_zone_index < obstacle_zone_points.size()) {
                        target_pose.header.frame_id = "map";
                        target_pose.header.stamp = ros::Time::now();
                        target_pose.pose.position = obstacle_zone_points[obstacle_zone_index];
                        nav_goal_pub.publish(target_pose);
                    }
                    rate.sleep();
                }

                // 如果队列为空且仍在导航状态，则发布当前位姿以防止无人机进入failsafe模式紧急降落
                if(points.empty() && mission_state == OBSTACLE_AVOIDING) {
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose = current_pose;
                    local_pos_pub.publish(pose);
                    ROS_INFO("No more points to track, waiting for next waypoint.");
                    // // 如果长时间没有点可跟踪，则尝试导航到上一个航点（这是在仿真环境中用来碰运气的）
                    // if(ros::Time::now() - last_request > ros::Duration(5.0)) {
                    //     ROS_WARN("No points to track for a while, trying navigating to last objective.");
                    //     target_pose.header.frame_id = "map";
                    //     target_pose.header.stamp = ros::Time::now();
                    //     int last_index = current_waypoint_index > 0 ? current_waypoint_index - 1 : 0;
                    //     target_pose.pose.position = obstacle_zone_points[last_index];
                    //     nav_goal_pub.publish(target_pose);
                    //     ros::Duration(0.05).sleep();
                    // }
                }
                break;
            }
               
            case DESCENDING: // 先降到较低高度以缓冲
            {
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 0.3;
                while (distance(current_pose, pose.pose.position) > threshold_distance && ros::ok()) {
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    ROS_INFO("Descending to landing height...");
                    rate.sleep();
                }
                mission_state = LANDING;
                break;
            }

            case LANDING:
            {
                mavros_msgs::SetMode land_set_mode;
                land_set_mode.request.custom_mode = "AUTO.LAND";
                
                if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) { // 自动降落和锁桨
                    ROS_INFO("Landing initiated");
                    arm_cmd.request.value = false;
                    if( current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                            ROS_INFO("Vehicle disarmed");
                        }
                        last_request = ros::Time::now();
                    }
                } else {
                    ROS_ERROR("Failed to set LAND mode!");
                    // 添加重试逻辑
                }
                break;
            }
        }
        rate.sleep();
    }
    return 0;
}