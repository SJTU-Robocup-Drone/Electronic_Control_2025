#include "communication.h"
#include "function.h"
#include "state_machine.h"
#include "vision.h"
#include "compute.h"

// 定义剩余全局
MissionState mission_state = TAKEOFF;

geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped pose;
geometry_msgs::Twist vel;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped nav_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_nav_pose;
geometry_msgs::Point last_nav_point;
geometry_msgs::Point first_target_point;
geometry_msgs::Point last_target_point;

mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;

std_msgs::Bool nav_state_msg;
std_msgs::Bool vision_state_msg;
std_msgs::Bool return_state_msg;
std_msgs::Bool is_done_msg;
std_msgs::Bool param_set_msg;

bool is_stuck = false;
bool is_once_stuck = false;
bool is_return = false;
bool is_vision_right = true;
bool adjust_has_target = false;

int vision_bias_cnt = 0;
int target_index = 0;
int searching_index = 0;
int obstacle_zone_index = 0;

std_msgs::Int32 target_index_msg;

ros::Time last_request;
ros::Time takeoff_request;
ros::Time nav_request;

bool is_takeoff = false;
bool is_moving_target = false;
bool is_param_set = false;

double offset[3][2] = {{0, 0.16}, {0.16, 0}, {0, -0.16}};

const double threshold_distance = 0.1;

void takeoff(ros::Rate &rate)
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    last_request = ros::Time::now();

    while (ros::ok() && mission_state == TAKEOFF)
    {
        ros::spinOnce();
        // 自动切换到offboard模式（仅限虚拟机）
        // if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        // {
        //     if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        //     {
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // }

        // 切进offboard模式后就解锁
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && current_state.mode == "OFFBOARD")
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        set_and_pub_pose(0, 0, 1.3); // 更容易达到起飞条件

        if (!is_takeoff)
            takeoff_request = ros::Time::now();
        if (current_state.mode == "AUTO.TAKEOFF" && !is_takeoff)
        {
            takeoff_request = ros::Time::now(); // 记录起飞请求的时间
            is_takeoff = true;
        }
        if (current_pose.pose.position.z <= 0.05 && (ros::Time::now() - takeoff_request > ros::Duration(5.0)))
        {
            mission_state = LANDING;
            ROS_INFO("Takeoff is wrong,position.z < 0.05");
            break;
        }

        if (current_pose.pose.position.z >= 0.8)
        {
            mission_state = SEARCHING;
            ROS_INFO("Takeoff complete. Starting overlooking.");

            // 起飞后悬停一秒，给建图和ego_planner启动留时间；同时也给pose一个初始的有效值，防止飞控在ego_planner未启动时因长时间接收不到目标点而进入failsafe模式
            hovering(1.0, 1, false, rate);
            break; // 跳出循环，进入导航状态
        }
        rate.sleep();
    }
}

void overlooking(ros::Rate &rate)
{
    vision_state_msg.data = true; // 开启视觉扫描
    ROS_INFO("Stabilizing first...");
    hovering(1.0, 2, false, rate);

    ROS_INFO("Start overlooking for target. Rising to overlooking position...");
    pose.pose.position.z = 3.0; // 悬停高度
    // 速度控制较低速上升，防止吹跑已经投放好的弹
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.7;
    while (ros::ok() && current_pose.pose.position.z <= 3.0)
    {
        ros::spinOnce();
        local_vel_pub.publish(vel);
        vision_state_pub.publish(vision_state_msg);
        rate.sleep();
    }

    ROS_INFO("Reached overlooking position. Scanning for target...");
    hovering(3, 10, true, rate);

    ROS_INFO("Overlooking complete, descending to normal flight height.");
    // 降低高度到1米(原飞行高度)
    set_and_pub_pose(current_pose.pose.position.x, current_pose.pose.position.y, 1.0);
    vision_state_msg.data = false; // 关闭视觉扫描
    while (ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance)
    {
        ros::spinOnce();
        vision_state_pub.publish(vision_state_msg);
        local_pos_pub.publish(pose);
        rate.sleep();
    }

    if (target_pose.pose.position.z != -1)
    { // 找到了靶标，进入靶标导航状态
        mission_state = BOMB_NAVIGATING;
        ROS_INFO("Target found, starting navigation to target.");
    }
    else
    {
        mission_state = SEARCHING; // 没有找到靶标，进入搜索状态
        ROS_INFO("No target found, starting searching.");
    }
}

void searching(ros::Rate &rate)
{
    // 如果有弹可投，直接投弹
    if (target_pose.pose.position.z != -1){
        mission_state = BOMB_NAVIGATING;
        return;
    }
    bool isRetrying = false;// 表示这一次searching是不是在重试之前卡住的点
    geometry_msgs::Point retry_point;
    if (!is_stuck)
    { // 上一次没有被卡住
        if(retry_points.size() > 0){ // 存在需要重试的点
            retry_point = retry_points.front();
            retry_points.pop();
            // TODO：把发布的导航点设为retry_point，而非searching_points[searching_index]
            isRetrying = true;
        }
        else{
            while (searching_points[searching_index].z == -1)
            searching_index++;
        }    
    }
    else
    { // 上一次被卡住了，这一次先不去重试上一次的点
        while (searching_points[searching_index].z == -1)
            searching_index--;
        is_stuck = false;
        is_once_stuck = true;
    }

    // 如果所有搜索点都已访问或ego-planner卡得索引降至零以下了，为了防止下标越界直接切状态
    if (searching_index >= searching_points.size() || searching_index < 0)
    {
        is_done_msg.data = true;
        is_done_pub.publish(is_done_msg);
        last_request = ros::Time::now();
        hovering(0.9, 2, false, rate);
        if (target_pose.pose.position.z != -1)
            mission_state = BOMB_NAVIGATING;
        else
        {
            ROS_WARN("All searching points reached. Navigating now.");
            mission_state = OBSTACLE_AVOIDING;
        }
        return;
    }

    // 发布航点并进入导航状态前先悬停，给建图留时间
    ROS_INFO("Hovering before navigating...");
    hovering(0.9, 5.0, false, rate);

    if(!isRetrying){// 导航前往新的点
        // 发布航点,更新导航时间
        set_and_pub_nav(searching_points[searching_index].x, searching_points[searching_index].y, searching_points[searching_index].z);

        // 轨迹跟踪与检查
        ROS_INFO("Searching for target %d...", searching_index + 1);
    }
    else{// 导航前往重试点
        // 发布航点,更新导航时间
        set_and_pub_nav(retry_point.x, retry_point.y, retry_point.z);

        // 轨迹跟踪与检查
        ROS_INFO("Searching for a retrying point...");
    }
   
    while (ros::ok())
    {
        ros::spinOnce();
        if (is_stuck)
        { // 如果ego-planner卡住了，放弃当前搜索点，同时暂时关闭导航
            ROS_WARN("Ego-planner is stuck. Trying navigating to last searching point and aborting current searching point...");
            searching_points[searching_index].z = -1;
            retry_points.push(searching_points[searching_index]);// 存入重试队列
            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);
            break;
        }

        if (distance(current_pose, nav_pose.pose.position) < threshold_distance)
        {
            ROS_INFO("Reached searching point %d.", searching_index + 1);
            searching_index++;
            if (target_pose.pose.position.z != -1)
                mission_state = BOMB_NAVIGATING;
            else if (is_once_stuck)
            {
                mission_state = SEARCHING;
                is_once_stuck = false;
            }
            else
                mission_state = OVERLOOKING;
            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);
            break;
        }

        rate.sleep();
    }
}

void bomb_navigating(ros::Rate &rate)
{
    // ROS_INFO("Hovering before navigating...");
    // hovering(0.9, 5, false, rate);

    // 发布航点并更新导航时间,初始化第一个和上一个靶标点
    set_and_pub_nav(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    first_target_point = nav_pose.pose.position;
    last_target_point = nav_pose.pose.position;
    ROS_INFO("Navigating to target at (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

    // 轨迹跟踪与检查
    while (ros::ok() && distance(current_pose, nav_pose.pose.position) > threshold_distance)
    {
        ros::spinOnce();
        // local_pos_pub.publish(pose);
        // ROS_INFO_THROTTLE(1.0, "Heading to(%.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        if (is_stuck)
        { // 如果ego-planner卡住了，进入搜索状态并放弃当前搜索点，同时暂时关闭导航
            ROS_WARN("Ego-planner is stuck. Trying navigating to last searching point and aborting current searching point...");
            mission_state = SEARCHING;
            searching_points[searching_index].z = -1;
            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);
            break;
        }

        // 到目标点后进入调整对准状态
        if (distance(current_pose, nav_pose.pose.position) < threshold_distance)
        {
            ROS_INFO("Reached bomb target %d.", target_index + 1);
            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);
            if (is_moving_target)
            {
                mission_state = FOLLOWING;
                ROS_INFO("The target is moving, following it.");
                is_moving_target = false; // 重置标志
            }
            else
                mission_state = ADJUSTING;
            break;
        }

        rate.sleep();
    }
}

void adjusting(ros::Rate &rate)
{
    vision_state_msg.data = true; // 开启视觉扫描
    adjust_has_target = false;
    ROS_INFO("2nd time of visual scanning...");
    hovering(1, 10, true, rate);
    vision_state_msg.data = false; // 关闭视觉扫描

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    // 判断视觉误识别
    if (!is_vision_right)
    {
        is_vision_right = true;
        ROS_WARN("High target bias. Vision scanning of ADJUSTING may be wrong. Bombing depending on OVERLOOKING...");
        set_and_pub_pose(first_target_point.x, first_target_point.y, first_target_point.z);
    }
    else if (!adjust_has_target)
    {
        ROS_WARN("Adjusting stage hasn't scanned a target. Vision scanning of OVERLOOKING may be wrong. Directly turning to SEARCHING mode...");
        mission_state = SEARCHING;
        return;
    }
    else
    {
        set_and_pub_pose(target_pose.pose.position.x, target_pose.pose.position.y, current_pose.pose.position.z);
    }
    ROS_INFO("Adjusting position to target...");
    // 临时减小距离阈值，并预先调整姿态
    pose.pose.orientation = initial_pose.pose.orientation;
    while (distance(current_pose, pose.pose.position) > threshold_distance/2.0 && ros::ok())
    {
        ros::spinOnce();
        pose.header.stamp = ros::Time::now();
        vision_state_pub.publish(vision_state_msg);
        local_pos_pub.publish(pose);
        rate.sleep();
    }
    if (is_return)
        mission_state = DESCENDING;
    else
        mission_state = BOMBING; // 调整完成后进入投弹状态
}

void bombing(ros::Rate &rate)
{
    ROS_INFO("Start bombing...");
    // 先下降到较低高度，并调整姿态后再投弹
    // pose.pose.position.x = current_pose.pose.position.x;
    // pose.pose.position.y = current_pose.pose.position.y;
    pose.pose.position.z = 0.2; // 实际上没用
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = -2.0;
    // 边下降边投弹
    bool isBombed = false;
    while (ros::ok() && current_pose.pose.position.z >= 0.25)
    {
        ros::spinOnce();
        // local_pos_pub.publish(pose);
        local_vel_pub.publish(vel);
        if(current_pose.pose.position.z <= 0.5 && !isBombed)
        {
            ROS_INFO("Releasing bomb %d...", target_index + 1);
            target_index_msg.data = target_index;
            manba_pub.publish(target_index_msg);
            isBombed = true;
        }
        rate.sleep();
    }

    // 到点后悬停0.5秒,然后索引自增
    hovering(0.2, 0.5, false, rate);
    target_pose.pose.position.z = -1; // 防止视觉节点没有来得及发布新目标点或发布未找到目标点的消息导致重复导航和投弹

    ROS_INFO("Bombing %d done, rising to normal flight height.", target_index + 1);
    pose.pose.position.z = 1.0; // 实际上没用
    // 速度控制较低速上升，防止吹跑已经投放好的弹
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.2;
    while (ros::ok() && current_pose.pose.position.z <= 1.0)
    {
        ros::spinOnce();
        local_vel_pub.publish(vel);
        rate.sleep();
    }

    if (++target_index >= 3)
        mission_state = OBSTACLE_AVOIDING;
    else if(target_pose.pose.position.z != -1) mission_state = BOMB_NAVIGATING;
    else mission_state = SEARCHING;
}

void obstacle_avoiding(ros::NodeHandle &nh, ros::Rate &rate)
{
    while (target_index < 3)
    {
        ROS_INFO("Still have bombs left, dropping now...");
        target_index_msg.data = target_index;
        manba_pub.publish(target_index_msg);
        hovering(1.0, 0.5, false, rate);
        target_index++;
    }
    if (!is_param_set && obstacle_zone_index >= 1)
    {
        is_param_set = true;
        nh.setParam("grid_map/obstacles_inflation", 0.2);
        nh.setParam("grid_map/depth_filter_mindist", 0.5);
        nh.setParam("optimization/lambda_smooth", 300);
        nh.setParam("optimization/lambda_collision", 750);
        nh.setParam("optimization/lambda_feasibility", 80);
        nh.setParam("optimization/lambda_fitness", 120);
        nh.setParam("optimization/dist0", 0.28);
        param_set_msg.data = true;
        // param_set_pub.publish(param_set_msg);
    }

    ROS_INFO("Hovering before navigating...");
    hovering(0.9, 5, false, rate);
    // 发布航点，更新导航时间
    if (obstacle_zone_index < obstacle_zone_points.size())
    {
        set_and_pub_nav(obstacle_zone_points[obstacle_zone_index].x, obstacle_zone_points[obstacle_zone_index].y, obstacle_zone_points[obstacle_zone_index].z);
    }

    // ROS_INFO("EGO thinking...");
    // hovering(0.7, 3, false, rate);

    // 轨迹跟踪与检查
    while (ros::ok())
    {
        ros::spinOnce();
        // 到点时索引自增，若到达全部目标点，则进入降落模式
        if (distance(current_pose, obstacle_zone_points[obstacle_zone_index]) < threshold_distance)
        {
            ROS_INFO("reached mid point!");
            if (++obstacle_zone_index >= obstacle_zone_points.size())
            {
                ROS_INFO("All obstacle_zone_points reached!");
                // 给视觉节点发返航消息，同时标记返航状态
                return_state_msg.data = true;
                return_state_pub.publish(return_state_msg);
                is_return = true;
                mission_state = ADJUSTING;
            }
            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);
            break; // 跳出循环，进入下一个避障点或进入降落状态
        }

        rate.sleep();
    }
}

void descending(ros::Rate &rate)
{
    set_and_pub_pose(current_pose.pose.position.x, current_pose.pose.position.y, 0.3);
    while (distance(current_pose, pose.pose.position) > threshold_distance && ros::ok())
    {
        ros::spinOnce();
        local_pos_pub.publish(pose);
        ROS_INFO("Descending to landing height...");
        rate.sleep();
    }
    mission_state = LANDING;
}

void landing(ros::Rate &rate)
{
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    // 自动降落和锁桨
    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
    {
        ROS_INFO_THROTTLE(2.0, "Landing initiated");
        arm_cmd.request.value = false;
        if (current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed");
            }
            last_request = ros::Time::now();
        }
    }
    else
    {
        ROS_ERROR("Failed to set LAND mode!");
        // 添加重试逻辑
    }
}

void following(ros::Rate &rate)
{
    int follow_timer = 0;
    vision_state_msg.data = true; // 开启视觉扫描
    ROS_INFO("Follow to target...");
    set_and_pub_pose(current_pose.pose.position.x, current_pose.pose.position.y, 3.0);

    while (ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance)
    {
        ros::spinOnce();
        vision_state_pub.publish(vision_state_msg); // 重复发布启动扫描指令，防止视觉节点没有接收到
        local_pos_pub.publish(pose);
        rate.sleep();
    }

    ROS_INFO("Reached overlooking position. Scanning for target...");
    hovering(3, 5, true, rate);

    while (follow_timer <= 20)
    {
        if (target_pose.pose.position.z != -1)
        { // 找到了靶标，进入靶标跟随状态
          // ROS_INFO_THROTTLE(2.0, "Target found, keeping following to target.");
        }
        else
        {
            hovering(2, 5, true, rate);
            ROS_INFO("No target found, starting hovering.");
            break;
        }

        if (target_pose.pose.position.z != -1) // 接受有效点后放入缓存
            visionCallback(target_pose);

        // geometry_msgs::PoseStamped ref_pose;
        // computeOverheadVelocityCmd(computeAverageVelocity(), ros::Time::now(), target_pose.header.stamp, vel, ref_pose.pose.position);

        // local_vel_pub.publish(vel);

        geometry_msgs::Point p = predictNextPosition(1 / 30);
        set_and_pub_pose(p.x, p.y, current_pose.pose.position.z);

        if (distance(current_pose, pose.pose.position) < threshold_distance / 2.0 && ros::ok()) // 记录成功跟上的次数
            follow_timer++;
        else
            follow_timer = 0;

        ros::spinOnce();
        rate.sleep();

        if (follow_timer >= 20)
        {
            ROS_INFO("FOLLOW success (%d cycles), switching to BOMBING", follow_timer);
            mission_state = BOMBING; // 确认跟随后进入投弹状态
        }
    }
}

void returning(ros::Rate &rate)
{
    return_state_msg.data = true;
    return_state_pub.publish(return_state_msg);
    is_return = true;
    is_stuck = false;

    // 发布航点并更新导航时间
    set_and_pub_nav(0.0, 0.0, 1.0);

    // 轨迹跟踪与检查
    while (ros::ok())
    {
        ros::spinOnce();
        // 如果返航的时候ego-planner卡住了，放弃导航并直接翻越所有障碍物返回起点
        if (is_stuck)
        {
            ROS_WARN("Ego-planner is stuck. Trying directly getting over obstacles to return.");

            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);

            set_and_pub_pose(current_pose.pose.position.x, current_pose.pose.position.y, 3.2);
            while (current_pose.pose.position.z <= 3.0)
            {
                ros::spinOnce();
                local_pos_pub.publish(pose);
                rate.sleep();
            }

            set_and_pub_pose(0.0, 0.0, 3.0);
            while (distance(current_pose, pose.pose.position) > threshold_distance)
            {
                ros::spinOnce();
                local_pos_pub.publish(pose);
                rate.sleep();
            }

            mission_state = ADJUSTING;
            break;
        }

        // 到目标点后进入调整状态，借助视觉进一步定位起点
        if (distance(current_pose, nav_pose.pose.position) < threshold_distance)
        {
            ROS_INFO("Reached returning target.");

            nav_state_msg.data = false;
            nav_state_pub.publish(nav_state_msg);

            mission_state = ADJUSTING;
            break;
        }
        rate.sleep();
    }
}

void detecting(ros::Rate &rate)
{
    static board_ros::track::TrackerDropper compute;
    board_ros::track::Endpoints establishedPoints;

    enum DetectingState
    {
        APPROACHING,
        HIGH_LEARNING,
        CYCLE_MODELING,
        PREPARING,
    } detecting_state = APPROACHING;

    vision_state_msg.data = true; // 开启视觉扫描
    hovering(3.0, 5, true, rate);
    ROS_INFO("Start detecting for target line...");
    // 主循环：每步推进一次状态机
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        switch (detecting_state)
        {
        case APPROACHING:
        {
            if (target_pose.pose.position.z != -1)
            {
                set_and_pub_pose(target_pose.pose.position.x, target_pose.pose.position.y, 3.0);
                while (distance(current_pose, pose.pose.position) > threshold_distance)
                {
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }
                detecting_state = HIGH_LEARNING;
                hovering(3.0, 1, true, rate);
            }
            else
            {
                hovering(3.0, 5, true, rate);
                ROS_INFO("No target found, hovering.");
            }
            break;
        }
        case HIGH_LEARNING:
        {
            if (target_pose.pose.position.z != -1) // 接受有效点后放入缓存
            {
                compute.feed(target_pose);
            }

            establishedPoints = compute.endpoints();
            ROS_INFO("establishedPoints: A(%.2f, %.2f), B(%.2f, %.2f), L=%.2f, valid=%d",
                     establishedPoints.A.x(), establishedPoints.A.y(),
                     establishedPoints.B.x(), establishedPoints.B.y(),
                     establishedPoints.L, establishedPoints.valid);

            board_ros::track::publish_endpoints_posearray(establishedPoints, target_pose);
            board_ros::track::publish_direction_u(establishedPoints, target_pose);
            target_pub.publish(target_pose);
            break;
        }
        }
    }
}