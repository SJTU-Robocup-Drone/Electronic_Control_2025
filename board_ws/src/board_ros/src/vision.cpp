#include "vision.h"
#include "function.h"
#include <algorithm>
#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <cmath>
#include <map>
#include <string>

// 定义视觉缓存
std::deque<TimedPose> history_;
const size_t MAX_HISTORY = 10; // 缓冲 10 帧

bool is_bombing = false;
bool is_returning = false;
bool is_done = false;
bool is_found = false;
int current_index = 0;
double yaw = 0;
double coordX = 0;
double coordY = 0;
ros::Timer process_target_timer;

// 声明视觉信息接收相关函数
void detection_cb(const geometry_msgs::PointStamped::ConstPtr &msg);
void process_target_cb();
void receive_target();

ros::Subscriber pose_sub;
ros::Subscriber man_check_sub;
ros::Subscriber return_state_sub;
ros::Subscriber is_done_sub;
ros::Subscriber detection_sub;

// 目标类型映射
std::map<std::string, int> target_types = {
    {"tent", 0},
    {"bunker", 1},
    {"bridge", 2},
    {"car", 3},
    {"tank", 4},
    {"Helicopter", 5}};

// 目标坐标存储
double coordArray[6][2] = {{-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    coordX = current_pose.pose.position.x;
    coordY = current_pose.pose.position.y;
    yaw = tf2::getYaw(current_pose.pose.orientation);
}

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

// 初始化融合方案的视觉相关接口
void init_vis_interfaces(ros::NodeHandle &nh)
{
    ROS_INFO("Initializing vision interfaces...");
    target_pose.pose.position.z = -1;

    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    man_check_sub = nh.subscribe<std_msgs::Int32>("/manba_input", 10, man_check_cb);
    return_state_sub = nh.subscribe<std_msgs::Bool>("/return_state", 10, return_state_cb);
    is_done_sub = nh.subscribe<std_msgs::Bool>("/done_state", 10, is_done_cb);
    detection_sub = nh.subscribe<geometry_msgs::PointStamped>("/detection_results", 10, detection_cb);

    process_target_timer = nh.createTimer(ros::Duration(0.02), [](const ros::TimerEvent &)
                                          { process_target_cb(); });
}
// 接收视觉节点发的相对目标坐标并转化为全局坐标
void detection_cb(const geometry_msgs::PointStamped::ConstPtr &msg) {
    target_pose.header.stamp = msg -> header.stamp; // 记录消息时间戳
    std::string target_name = msg -> header.frame_id;
    double rel_x = msg->point.x;
    double rel_y = msg->point.y;

    // 查找目标类型
    auto it = target_types.find(target_name);
    if (it != target_types.end())
    {
        int type = it->second;

        //定义机体系x，y
        double drone_x = -rel_y;
        double drone_y = -rel_x;

        // 相对坐标转全局坐标（考虑无人机偏航角）
        double global_x = coordX + drone_x * cos(yaw) - drone_y * sin(yaw);
        double global_y = coordY + drone_x * sin(yaw) + drone_y * cos(yaw);

        // 只更新未投掷的目标
        if (coordArray[type][0] != -50)
        {
            coordArray[type][0] = global_x;
            coordArray[type][1] = global_y;
            
            ROS_INFO_THROTTLE(2.0, "Refreshing target %s, relative coord: (%.2f, %.2f)", target_name.c_str(), global_x, global_y);
        }
    }
}

// 定时遍历目标数组并根据其中的数据更新target_pose
void process_target_cb()
{
    if (is_returning)
    {
        // 返航阶段：寻找降落区
        if (coordArray[5][0] != -100 && coordArray[5][0] != -50)
        {
            is_found = true;
            ROS_INFO("Landing area found at: (%.2f, %.2f)", coordArray[5][0], coordArray[5][1]);
            
            target_pose.header.frame_id = "map";
            target_pose.pose.position.x = coordArray[5][0];
            target_pose.pose.position.y = coordArray[5][1];
            target_pose.pose.position.z = 1.0;

            current_index = 5;
        }
    }
    else
    {
        // 投弹阶段：按优先级选择目标
        for (int i = 4; i >= 0; i--)
        {
            if (coordArray[i][0] != -100 && coordArray[i][0] != -50)
            {
                is_found = true;

                target_pose.header.frame_id = "map";
                target_pose.pose.position.x = coordArray[i][0];
                target_pose.pose.position.y = coordArray[i][1];
                target_pose.pose.position.z = 1.0;

                current_index = i;
                break;
            }
        }
    }

    if (!is_found)
    {
        target_pose.header.frame_id = "map";
        target_pose.pose.position.z = -1;
        is_found = false;
    }

    if (is_bombing)
    {
        // 标记当前目标为已投掷
        coordArray[current_index][0] = -50;
        coordArray[current_index][1] = -50;
        is_bombing = false;
    }
    receive_target();
}

// 对接收到的target_pose进行最后处理，用于融合方案中代替target_cb
void receive_target()
{
    // 根据任务状态调整目标位置进行纠偏
    if (target_index < 3 && target_pose.pose.position.z != -1)
    {
        ros::Duration delay = ros::Time::now() - target_pose.header.stamp;
        ROS_INFO_THROTTLE(2.0, "Target found at (%.2f, %.2f, %.2f), propagation delay: %.2f s",
                          target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z, delay.toSec());
        if ((mission_state == ADJUSTING && !is_return) || mission_state == FOLLOWING)
        {
            target_pose.pose.position.x += offset[target_index][0];
            target_pose.pose.position.y += offset[target_index][1];
        }
    }

    // 判断是否为移动靶
    if (target_pose.pose.position.z == 2.0)
        is_moving_target = true;

    // 如果调整阶段靶标位置连续两次偏差超过0.5米，则认为视觉误识别（假定OVERLOOKING的识别结果没有问题，因为之前没有出过错）
    if (mission_state == ADJUSTING)
    {
        if (distance(target_pose, last_target_point) > 0.5)
        {
            if (++vision_bias_cnt >= 2)
            {
                vision_bias_cnt = 0;
                is_vision_right = false;
            }
        }
        else
            vision_bias_cnt = 0;
        last_target_point = target_pose.pose.position;
        adjust_has_target = true;
    }
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose = *msg;
    if (target_index < 3 && target_pose.pose.position.z != -1)
    {
        ROS_INFO_THROTTLE(2.0, "Target found at (%.2f, %.2f, %.2f)",
                          target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        if ((mission_state == ADJUSTING && !is_return) || mission_state == FOLLOWING)
        {
            target_pose.pose.position.x += offset[target_index][0];
            target_pose.pose.position.y += offset[target_index][1];
        }
    }
    if (target_pose.pose.position.z == 2.0)
        is_moving_target = true;

    if (mission_state == ADJUSTING) // 如果调整阶段靶标位置连续两次偏差超过0.5米，则认为视觉误识别（假定OVERLOOKING的识别结果没有问题，因为之前没有出过错）
    {
        if (distance(target_pose, last_target_point) > 0.5)
        {
            if (++vision_bias_cnt >= 2)
            {
                vision_bias_cnt = 0;
                is_vision_right = false;
            }
        }
        else
            vision_bias_cnt = 0;
        last_target_point = target_pose.pose.position;
        adjust_has_target = true;
    }
    ros::Duration delay = ros::Time::now() - msg->header.stamp;
    ROS_INFO_THROTTLE(2.0, "Target propagation delay: %.2f s", delay.toSec());
}

void visionCallback(const geometry_msgs::PoseStamped &msg) // 储存五个视觉目标点缓存
{
    TimedPose tp;
    tp.pos = msg.pose.position;
    tp.stamp = msg.header.stamp;
    history_.push_back(tp);
    if (history_.size() > MAX_HISTORY)
        history_.pop_front();
}

geometry_msgs::Vector3 computeAverageVelocity()
{
    geometry_msgs::Vector3 v{};
    if (history_.size() < 2)
        return v;

    geometry_msgs::Vector3 d{};
    d.x = history_.back().pos.x - history_.front().pos.x;
    d.y = history_.back().pos.y - history_.front().pos.y;
    d.z = history_.back().pos.z - history_.front().pos.z;

    double dt_total = std::max(1e-3, (history_.back().stamp - history_.front().stamp).toSec());
    v.x = d.x / dt_total;
    v.y = d.y / dt_total;
    v.z = d.z / dt_total;
    return v;
}

geometry_msgs::Point predictNextPosition(double predict_dt)
{
    auto v = computeAverageVelocity();
    geometry_msgs::Point p_last{};
    if (!history_.empty())
        p_last = history_.back().pos;

    geometry_msgs::Point p_pred{};
    p_pred.x = p_last.x + v.x * predict_dt;
    p_pred.y = p_last.y + v.y * predict_dt;
    p_pred.z = p_last.z + v.z * predict_dt;
    return p_pred;
}
