#include "communication.h"
#include "function.h"
#include "vision.h"
#include "offboard.h"

// 定义话题服务
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher nav_goal_pub;
ros::Publisher nav_state_pub;
ros::Publisher vision_state_pub;
ros::Publisher return_state_pub;
ros::Publisher is_done_pub;
ros::Publisher param_set_pub;
ros::Publisher manba_pub;
ros::Publisher target_pub;

ros::Subscriber random_target_sub;
ros::Subscriber target_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber state_sub;
ros::Subscriber nav_check_sub;
ros::Subscriber pose_sub;
ros::Subscriber man_check_sub;
ros::Subscriber return_state_sub;
ros::Subscriber is_done_sub;
ros::Subscriber detection_sub;
ros::Subscriber adjusted_goal_sub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

ros::Timer process_target_timer;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void nav_check_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    if (!nav_state_msg.data)
        nav_request = ros::Time::now();
    else
    {
        current_nav_pose.header = msg->header;
        current_nav_pose.pose.position = msg->position;
        if (distance(current_nav_pose, last_nav_point) > 1e-2)
        {
            last_nav_point = current_nav_pose.pose.position;
            nav_request = ros::Time::now();
        }
        else if (ros::Time::now() - nav_request > ros::Duration(10.0))
        {
            is_stuck = true;
            nav_request = ros::Time::now();
        }
    }
}

void adjusted_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //TODO
    if(mission_state == SEARCHING){
        searching_points[searching_index] = msg->pose.position;
        ROS_INFO("Received adjusted goal for searching: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
    else if(mission_state == OBSTACLE_AVOIDING){
        obstacle_zone_points[obstacle_zone_index] = msg->pose.position;
        ROS_INFO("Received adjusted goal for obstacle avoiding: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
    else{
        ROS_WARN("Received adjusted goal in invalid state, ignoring.");
    }
}

// 统一通信初始化
void init_nav_interfaces(ros::NodeHandle &nh)
{
    target_pose.pose.position.z = -1;

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    nav_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/control/move_base_simple/goal", 10);
    nav_state_pub = nh.advertise<std_msgs::Bool>("/nav_state", 10);
    return_state_pub = nh.advertise<std_msgs::Bool>("/return_state", 10);
    is_done_pub = nh.advertise<std_msgs::Bool>("/done_state", 10);
    manba_pub = nh.advertise<std_msgs::Int32>("/manba_input", 10);
    vision_state_pub = nh.advertise<std_msgs::Bool>("/vision_state", 10);
    target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target", 10);

    local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/odom_high_freq", 10, pose_cb);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    nav_check_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50, nav_check_cb);
    random_target_sub = nh.subscribe<geometry_msgs::PointStamped>("/random_target", 10, random_target_cb);
    target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target", 10, target_cb);
    return_state_sub = nh.subscribe<std_msgs::Bool>("/return_state", 10, return_state_cb);
    is_done_sub = nh.subscribe<std_msgs::Bool>("/done_state", 10, is_done_cb);
    detection_sub = nh.subscribe<geometry_msgs::PointStamped>("/detection_results", 10, detection_cb);
    adjusted_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/adjusted_goal", 10, adjusted_goal_cb);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    process_target_timer = nh.createTimer(ros::Duration(0.02), [](const ros::TimerEvent &)
                                          { process_target_cb(); });
    portName = "/dev/ttyUSB1"; // 连接esp32的串口名
    baudrate = 115200; // 串口波特率
    try {
        ser.setPort(portName);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << portName);
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized: " << portName);
    } 
}
