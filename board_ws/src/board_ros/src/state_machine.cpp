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

std::string portName;
std::string command;
int baudrate;
serial::Serial ser;

bool is_stuck = false;
bool is_once_stuck = false;
bool is_return = false;
bool is_vision_right = true;
bool adjust_has_target = false;
bool is_retrying_searching_point = false;
bool is_retrying_bombing_point = false;

int vision_bias_cnt = 0;
int target_index = 0;
int searching_index = 0;
int obstacle_zone_index = 0;
int retrying_target_index = -1;

std_msgs::Int32 target_index_msg;

ros::Time last_request;
ros::Time takeoff_request;
ros::Time nav_request;

bool is_takeoff = false;
bool is_moving_target = false;
bool is_param_set = false;

const double threshold_distance = 0.1;

bool need_exit = false;

void auto_offboard();
void auto_armed();
void record_takeoff();
void check_takeoff();
void check_and_complete_takeoff();

void vertically_set_vel_to_move(float v, float h, ros::Rate &rate);
void vertically_set_pose_to_move(float h, ros::Rate &rate);

void check_bombing();
void set_index_and_retry(RetryPoint &retry_point);
void check_index(ros::Rate &rate);
void pub_searching_or_retrying_point(RetryPoint &retry_point);
void se_check_stuck();
void se_check_arrival();

void pub_bomb_navigating_or_retrying_point();
void bn_check_stuck();
void bn_check_arrival();
void check_vision(float adj_height);
void adjust_by_pose(ros::Rate &rate, float adj_height);

void vertically_set_vel_to_bomb(float v, float bomb_height, float lowest_height, ros::Rate &rate);

void check_left_bombs(ros::Rate &rate);
void update_params(ros::NodeHandle &nh);
void oa_check_arrival();

void auto_land_and_disarmed(mavros_msgs::SetMode land_set_mode);

void return_init();
void return_check_stuck(ros::Rate &rate);
void return_check_arrival();
void horizontally_set_pose_to_move(double x, double y, ros::Rate &rate);
void follow_bombing(ros::Rate &rate, float kp);

void takeoff(ros::Rate &rate)
{
    need_exit = false;

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    last_request = ros::Time::now();

    while (ros::ok() && mission_state == TAKEOFF)
    {
        ros::spinOnce();

        // 自动切换到offboard模式（仅限虚拟机）
        // auto_offboard();

        // 切进offboard模式后就解锁
        auto_armed();

        // 更容易达到起飞条件
        set_and_pub_pose(0, 0, 1.3);

        // 记录起飞请求的时间
        record_takeoff();

        // 如果起飞时定位异常，直接降落
        check_takeoff();

        // 如果起飞到指定高度，退出起飞状态
        check_and_complete_takeoff();

        if(need_exit) return;

        rate.sleep();
    }
}

void overlooking(ros::Rate &rate)
{
    // 开启视觉扫描
    vision_state_msg.data = true;
    vision_state_pub.publish(vision_state_msg);

    ROS_INFO("Stabilizing first...");
    hovering(0.8, 2, false, rate);
    ROS_INFO("Start overlooking for target. Rising to overlooking position...");

    // 速度控制较低速上升，防止吹跑已经投放好的弹
    vertically_set_vel_to_move(0.7, 3.0, rate);

    ROS_INFO("Reached overlooking position. Scanning for target...");
    hovering(3, 2, false, rate);
    hovering(3, 4, true, rate);
    ROS_INFO("Overlooking complete, descending to normal flight height.");

    // 关闭视觉扫描
    vision_state_msg.data = false;
    vision_state_pub.publish(vision_state_msg);

    // 降低高度到1米(原飞行高度)
    vertically_set_pose_to_move(0.9, rate);

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
    need_exit = false;

    // 重试点结构体，包含了坐标&在searching_points中的索引
    RetryPoint retry_point;

    // 如果有弹可投，直接投弹
    check_bombing();
    if (need_exit) return;

    set_index_and_retry(retry_point);

    // 如果所有搜索点都已访问或ego-planner卡得索引降至零以下了，为了防止下标越界直接切状态
    check_index(rate);
    if (need_exit) return;

    // 发布航点并进入导航状态前先悬停，给建图留时间
    ROS_INFO("Hovering before navigating...");
    hovering(0.8, 3, false, rate);
    
    pub_searching_or_retrying_point(retry_point);

    // 进入导航状态
    while (ros::ok())
    {
        ros::spinOnce();

        // 如果ego-planner卡住了，放弃当前搜索点，同时暂时关闭导航
        se_check_stuck();

        // 检查到点和之后的状态切换
        se_check_arrival();

        if (need_exit) return;

        rate.sleep();
    }
}

void bomb_navigating(ros::Rate &rate)
{
    need_exit = false;

    pub_bomb_navigating_or_retrying_point();

    // 进入导航状态
    while (ros::ok())
    {
        ros::spinOnce();

        // 如果ego-planner卡住了，进入搜索状态并放弃当前搜索点，同时暂时关闭导航
        bn_check_stuck();

        // 到目标点后进入调整对准状态
        bn_check_arrival();

        if (need_exit) return;

        rate.sleep();
    }
}

void adjusting(ros::Rate &rate)
{
    need_exit = false;

    // adjust高度设置
    float adj_height = 0.6;
    if (is_return) adj_height = 0.9;

    // 开启视觉扫描
    vision_state_msg.data = true;
    vision_state_pub.publish(vision_state_msg);

    adjust_has_target = false;
    ROS_INFO("2nd time of visual scanning...");
    hovering(adj_height, 2, false, rate); // 稳定位姿
    hovering(adj_height, 4, true, rate);  // 等待扫描

    // 关闭视觉扫描
    vision_state_msg.data = false;        
    vision_state_pub.publish(vision_state_msg);

    // 判断视觉误识别并刷新目标点
    check_vision(adj_height);
    if(need_exit) return;

    // 临时减小距离阈值，并预先调整姿态
    ROS_INFO("Adjusting pose to target...");
    adjust_by_pose(rate, adj_height);

    if (is_return)
        mission_state = DESCENDING;
    else
        mission_state = BOMBING;
}

void bombing(ros::Rate &rate)
{
    // 边下降边投弹
    ROS_INFO("Start bombing...");
    vertically_set_vel_to_bomb(-2.0, 0.5, 0.4, rate);

    // 到点后悬停0.5秒,然后索引自增
    hovering(0.4, 0.5, false, rate);
    target_pose.pose.position.z = -1; // 防止视觉节点没有来得及发布新目标点或发布未找到目标点的消息导致重复导航和投弹

    // 速度控制较低速上升，防止吹跑已经投放好的弹
    ROS_INFO("Bombing %d done, rising to normal flight height.", target_index + 1);

    vertically_set_vel_to_move(0.2, 0.8, rate);

    if (++target_index >= 3)
        mission_state = OBSTACLE_AVOIDING;
    else if (target_pose.pose.position.z != -1)
        mission_state = BOMB_NAVIGATING;
    else
        mission_state = SEARCHING;
}

void obstacle_avoiding(ros::NodeHandle &nh, ros::Rate &rate)
{
    need_exit = false;
    
    // 若有弹未投完，把弹投完；关闭串口
    check_left_bombs(rate);

    // 参数热更新
    // update_params(nh);

    // 导航前先悬停，给建图留时间
    ROS_INFO("Hovering before navigating...");
    hovering(0.8, 5, false, rate);

    // 发布航点，更新导航时间
    if (obstacle_zone_index < obstacle_zone_points.size())
    {
        set_and_pub_nav(obstacle_zone_points[obstacle_zone_index].x, obstacle_zone_points[obstacle_zone_index].y, obstacle_zone_points[obstacle_zone_index].z);
    }

    // 进入导航状态
    while (ros::ok())
    {
        ros::spinOnce();

        // 到点时索引自增，若到达全部目标点，则进入降落模式
        oa_check_arrival();

        if (need_exit) return;

        rate.sleep();
    }
}

void descending(ros::Rate &rate)
{
    // 若串口未关闭，关闭串口
    if (ser.isOpen()) ser.close();
    
    ROS_INFO("Descending to landing height...");
    vertically_set_pose_to_move(0.3, rate);

    mission_state = LANDING;
}

void landing(ros::Rate &rate)
{
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    // 自动降落和锁桨
    auto_land_and_disarmed(land_set_mode);
}

void following(ros::Rate &rate)
{
    int follow_timer = 0;

    vision_state_msg.data = true; // 开启视觉扫描
    vision_state_pub.publish(vision_state_msg);

    ROS_INFO("Follow to target...");

    // 速度控制较低速上升，防止吹跑已经投放好的弹
    vertically_set_vel_to_move(0.7, 3.0, rate);

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
    need_exit = false;

    return_init();

    // 发布航点并更新导航时间
    set_and_pub_nav(0.0, 0.0, 1.0);

    // 进入导航状态
    while (ros::ok())
    {
        ros::spinOnce();

        // 如果返航的时候ego-planner卡住了，放弃导航并直接翻越所有障碍物返回起点
        return_check_stuck(rate);

        // 到目标点后进入调整状态，借助视觉进一步定位起点
        return_check_arrival();

        if (need_exit) return;
        rate.sleep();
    }
}

void detecting(ros::Rate &rate)
{
    static track::Learner compute;
    static track::Learner compute_v;
    compute_v.prm.learn_window_sec = 0.3;
    track::Endpoints establishedPoints;
    track::Endpoints velocityPoints;
    ros::Time switch_time = ros::Time::now();
    static bool edge = false; // 边沿触发标志
    static bool is_second_learning = false;

    enum DetectingState
    {
        APPROACHING,    // 接近轨道准备高空学习
        HIGH_LEARNING,  // 高空学习阶段
        REACH_ENDPOINT, // 建模后到达端点重新建模
        CYCLE_MODELING, // 循环建模阶段
        PREPARING,      // 准备投弹阶段
        DROP_BOMB,      // 直接投弹阶段
    } detecting_state = APPROACHING;

    // 稳定位姿
    vertically_set_vel_to_move(0.7, 3.0, rate);
    hovering(3.0, 2, false, rate);

    // 主循环：每步推进一次状态机
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        // 不在移动的时候喂入目标坐标
        if (detecting_state != APPROACHING && detecting_state != REACH_ENDPOINT && target_pose.pose.position.z != -1)
        {
            compute.feed(target_pose);
            target_pub.publish(target_pose);
        }

        switch (detecting_state)
        {
        case APPROACHING:
        {
            // 视觉扫描
            vision_state_msg.data = true;
            hovering(3.0, 5, true, rate);
            ROS_INFO("Start detecting for target line...");
            //移动到瞬时目标位置
            if (target_pose.pose.position.z != -1)
            {
                set_and_pub_pose(target_pose.pose.position.x, target_pose.pose.position.y, 3.0);
                while (distance(current_pose, pose.pose.position) > threshold_distance)
                {
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }
                // 稳定位姿
                hovering(3.0, 3, false, rate);
                // 切换到高空学习模式
                detecting_state = HIGH_LEARNING;
                switch_time = ros::Time::now();
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
            // 建立模型
            compute.compute();
            establishedPoints = compute.ep;
            // ROS_INFO("establishedPoints: A(%.2f, %.2f), B(%.2f, %.2f), L=%.2f, valid=%d",
            //          establishedPoints.A.x(), establishedPoints.A.y(),
            //          establishedPoints.B.x(), establishedPoints.B.y(),
            //          establishedPoints.L, establishedPoints.valid);

            // 发布生成的直线模型
            track::publish_endpoints(establishedPoints, target_pose);

            // ROS_INFO("Switch time is %.2f", ros::Time::now().toSec() - switch_time.toSec());
            if (ros::Time::now() - switch_time > ros::Duration(30.0))
            {
                // 初次学习之后移动到端点
                if (!is_second_learning)
                    detecting_state = REACH_ENDPOINT;
                else
                {
                    detecting_state = PREPARING;
                }
                switch_time = ros::Time::now();
            }
            break;
        }

        case REACH_ENDPOINT:
        {
            ROS_INFO("Getting to the endpoint A");
            set_and_pub_pose(establishedPoints.A[0], establishedPoints.A[1], 1.5);
            while (distance(current_pose, pose.pose.position) > threshold_distance)
            {
                ros::spinOnce();
                local_pos_pub.publish(pose);
                rate.sleep();
            }

            ROS_INFO("Enter second learning");
            hovering(1, 1.5, false, rate);
            is_second_learning = true;

            detecting_state = DROP_BOMB;
            switch_time = ros::Time::now();
            break;
        }

        case PREPARING:
        {
            // 进入投弹准备阶段
            ros::Time tA, tB;
            ros::Time window_centre;

            static double bomb_time;

            establishedPoints = compute.ep;
            // if (compute.predictPassTimes(ros::Time::now(), tA, tB))
            // {
            //     ROS_INFO("Next pass times: tA=%.2f, tB=%.2f", tA.toSec(), tB.toSec());
            // }

            double miss;
            if (1)
            {
                ROS_INFO("DirectRelease: t_rel=%.2f  miss≈%.2fm", window_centre.toSec(), miss);
                // 触发投弹：fabs(ros::Time::now().toSec() - t_rel.toSec()) < 0.02
            }

            // 等待循环建模稳定进入投弹预测
            if (1)
            {
                std::pair<ros::Time, ros::Time> window_out;
                // 仿真投弹是否成功
                if ((abs(ros::Time::now().toSec() - window_centre.toSec()) < 0.02) && !edge)
                {
                    bomb_time = window_centre.toSec();
                    // 保证仿真不会被重复触发
                    edge = true;
                }

                if ((abs(ros::Time::now().toSec() - 0.5 ) < 0.1) && edge)
                {
                    edge = false;
                    if (distance(current_pose, target_pose.pose.position) < 2 * threshold_distance)
                    {
                        ROS_INFO("BOMBING SUCCEEDED");
                        ROS_INFO("BOMBING SUCCEEDED");
                        ROS_INFO("BOMBING SUCCEEDED");
                    }
                    else
                    {
                        ROS_INFO("BOMBING FAILED,the distance is %.2f", distance(current_pose, target_pose.pose.position));
                        ROS_INFO("BOMBING FAILED,the distance is %.2f", distance(current_pose, target_pose.pose.position));
                        ROS_INFO("BOMBING FAILED,the distance is %.2f", distance(current_pose, target_pose.pose.position));
                    }
                }
            }
            break;
        }

        case DROP_BOMB:
        {
            // 直接投弹
            static bool tank_state = false;
            while (ros::ok() && ros::Time::now() - switch_time < ros::Duration(30))
            {
                ros::spinOnce();
                local_pos_pub.publish(pose); // 保持悬停
                rate.sleep();
                
                // 目标速度解算
                compute_v.feed(target_pose);
                geometry_msgs::Vector3 v;
                compute_v.fitLinearVelocityRANSAC(v.x,v.y,0.2,0.2);
                // visionCallback(target_pose);
                // v = computeAverageVelocity();
                // 计算投影距离
                geometry_msgs::PoseStamped tgt_pose = target_pose;
                tgt_pose.pose.position.z = current_pose.pose.position.z;
                // 用于判断小车是否从远处向无人机投影逼近
                if (distance(current_pose, tgt_pose.pose.position) >= 0.40)
                {
                    if (tank_state == false)
                        ROS_INFO("TANK_STATE gets into true.");
                    tank_state = true;
                    
                }

                double velocity = sqrt(v.x * v.x + v.y * v.y);
                velocityPoints.B = {v.x, v.y};
                velocityPoints.L = velocity;
                track::publish_endpoints(velocityPoints, target_pose);
                ROS_INFO("Speed is %2f", velocity);
                if (distance(current_pose, tgt_pose.pose.position) / velocity <= 1.6 && velocity != 0 && tank_state == true)
                {
                    follow_bombing(rate, 1);
                    ROS_INFO("Bomb now");
                    return; 
                }
            }
            break;
        }
        }
    }
}

void auto_offboard()
{
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    }
}

void auto_armed()
{
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && current_state.mode == "OFFBOARD")
    {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
    }
}

void record_takeoff()
{
    if (!is_takeoff)
        takeoff_request = ros::Time::now();
    if (current_state.mode == "AUTO.TAKEOFF" && !is_takeoff)
    {
        takeoff_request = ros::Time::now();
        is_takeoff = true;
    }
}

void check_takeoff()
{
    if (current_pose.pose.position.z <= 0.05 && (ros::Time::now() - takeoff_request > ros::Duration(5.0)))
    {
        mission_state = LANDING;
        ROS_INFO("Takeoff is wrong,position.z < 0.05");
        need_exit = true;
    }
}

void check_and_complete_takeoff()
{
    if (current_pose.pose.position.z >= 0.8)
    {
        mission_state = SEARCHING;
        ROS_INFO("Takeoff complete.");
        need_exit = true;
    }
}

void vertically_set_vel_to_move(float v, float h, ros::Rate &rate)
{
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = v;
    if(v > 0){
        while (ros::ok() && current_pose.pose.position.z <= h)
        {
            ros::spinOnce();
            local_vel_pub.publish(vel);
            rate.sleep();
        }
    }
    else{
        while (ros::ok() && current_pose.pose.position.z >= h)
        {
            ros::spinOnce();
            local_vel_pub.publish(vel);
            rate.sleep();
        }
    }
}

void vertically_set_pose_to_move(float h, ros::Rate &rate)
{
    set_and_pub_pose(current_pose.pose.position.x, current_pose.pose.position.y, h);
    while (ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance)
    {
        ros::spinOnce();
        local_pos_pub.publish(pose);
        rate.sleep();
    }
}

void check_bombing()
{
    if (target_pose.pose.position.z != -1)
    {
        mission_state = BOMB_NAVIGATING;
        need_exit = true;
    }
}

void set_index_and_retry(RetryPoint &retry_point)
{
    is_retrying_searching_point = false; // 表示这一次searching是不是在重试之前卡住的点
    if (!is_stuck)
    { // 上一次没有被卡住
        if (retry_searching_points.size() > 0)
        { // 存在需要重试的点
            retry_point = retry_searching_points.front();
            searching_index = retry_point.index; // 将索引重置为重试点对应的索引值
            retry_searching_points.pop();
            is_retrying_searching_point = true;
        }
        else
        {
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
}

void check_index(ros::Rate &rate)
{
    if (searching_index >= searching_points.size() || searching_index < 0)
    {
        is_done_msg.data = true;
        is_done_pub.publish(is_done_msg);
        last_request = ros::Time::now();
        hovering(0.8, 2, false, rate);
        if (target_pose.pose.position.z != -1){
            ROS_INFO("All searching points reached but still have targets. Navigating now.");
            mission_state = BOMB_NAVIGATING;
        }
        else
        {
            ROS_WARN("All searching points reached. Obstacle avoiding now.");
            mission_state = OBSTACLE_AVOIDING;
        }
        need_exit = true;
    }
}

void pub_searching_or_retrying_point(RetryPoint &retry_point)
{
    if (!is_retrying_searching_point)
    {
        // 发布航点,更新导航时间
        set_and_pub_nav(searching_points[searching_index].x, searching_points[searching_index].y, searching_points[searching_index].z);
        ROS_INFO("Searching for target %d...", searching_index + 1);
    }
    else
    {
        // 发布航点,更新导航时间
        set_and_pub_nav(retry_point.point.x, retry_point.point.y, retry_point.point.z);
        ROS_INFO("Searching for a retrying point...");
    }
}

void se_check_stuck()
{
    if (is_stuck)
    {
        ROS_WARN("Ego-planner is stuck. Trying navigating to last searching point and aborting current searching point...");
        searching_points[searching_index].z = -1;
        if (!is_retrying_searching_point)
        {
            RetryPoint retry_point;
            retry_point.point = searching_points[searching_index];
            retry_point.index = searching_index;
            retry_searching_points.push(retry_point);
        } // 如果没被重试过，那就存入重试队列
        nav_state_msg.data = false;
        nav_state_pub.publish(nav_state_msg);
        need_exit = true;
    }
}

void se_check_arrival()
{
    if (distance(current_pose, nav_pose.pose.position) < threshold_distance)
    {
        ROS_INFO("Reached searching point %d.", searching_index + 1);
        if (target_pose.pose.position.z != -1)
            mission_state = BOMB_NAVIGATING;
        else if (is_once_stuck)
        {
            mission_state = SEARCHING;
            is_once_stuck = false;
        }
        else
        {
            searching_index++;
            mission_state = OVERLOOKING;
        }
        nav_state_msg.data = false;
        nav_state_pub.publish(nav_state_msg);
        need_exit = true;
    }
}

void pub_bomb_navigating_or_retrying_point()
{
    is_retrying_bombing_point = false; // 表示这一次bomb_navigating是不是在重试之前卡住的点
    if (retry_navigating_points.size() > 0)
    { // 存在需要重试的点
        RetryPoint retry_point = retry_navigating_points.front();
        retry_navigating_points.pop();
        is_retrying_bombing_point = true;

        set_and_pub_nav(retry_point.point.x, retry_point.point.y, retry_point.point.z);
        first_target_point = nav_pose.pose.position;
        last_target_point = nav_pose.pose.position;
        ROS_INFO("Retrying navigating to retry point...");

        retrying_target_index = retry_point.index; // 接下来vision会锁定这个index
    }
    else
    {
        // 发布航点并更新导航时间,初始化第一个和上一个靶标点
        set_and_pub_nav(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        first_target_point = nav_pose.pose.position;
        last_target_point = nav_pose.pose.position;
        ROS_INFO("Navigating to target at (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    }
}

void bn_check_stuck()
{
    if (is_stuck)
    {
        ROS_WARN("Ego-planner is stuck. Trying navigating to last searching point and aborting current searching point...");
        mission_state = SEARCHING;
        searching_points[searching_index].z = -1;
        // 为什么bomb_navigating卡住了要放弃searching_points？是不是有BUG
        nav_state_msg.data = false;
        nav_state_pub.publish(nav_state_msg);
            
        RetryPoint retry_point(nav_pose.pose.position,current_index);
        if (!is_retrying_bombing_point)
            retry_navigating_points.push(retry_point); // 将当前导航点存入重试队列
        need_exit = true;
    }
}

void bn_check_arrival()
{
    if (distance(current_pose, nav_pose.pose.position) < threshold_distance)
    {
        ROS_INFO("Reached bomb target %d.", target_index + 1);
        nav_state_msg.data = false;
        nav_state_pub.publish(nav_state_msg);
        // 将“是否重试”状态设置为否
        is_retrying_bombing_point = false;
        retrying_target_index = -1;
        if (is_moving_target)
        {
            mission_state = DETECTING;
            ROS_INFO("The target is moving, detecting it.");
        }
        else
            mission_state = ADJUSTING;
        need_exit = true;
    }
}

void check_vision(float adj_height)
{
    if (!is_vision_right)
    {
        is_vision_right = true;
        ROS_WARN("High target bias. Vision scanning of ADJUSTING may be wrong. Bombing depending on OVERLOOKING...");
        set_and_pub_pose(first_target_point.x, first_target_point.y, adj_height);
    }
    else if (!adjust_has_target)
    {
        if (!is_return)
        {
            ROS_WARN("Adjusting stage hasn't scanned a target. Vision scanning of OVERLOOKING may be wrong. Directly turning to SEARCHING mode...");
            mission_state = SEARCHING;
            targetArray[current_index].isValid = false; // 放弃误识别的靶标
        }
        else
        {
            ROS_WARN("Adjusting stage hasn't scanned a target. Directly descending...");
            mission_state = DESCENDING;
        }
        need_exit = true;
    }
    else
    {
        set_and_pub_pose(target_pose.pose.position.x, target_pose.pose.position.y, adj_height);
    }
}

void adjust_by_pose(ros::Rate &rate, float adj_height)
{
    pose.pose.orientation = initial_pose.pose.orientation;
    while (distance(current_pose, pose.pose.position) > threshold_distance / 2.0 && ros::ok())
    {
        ros::spinOnce();
        set_and_pub_pose(target_pose.pose.position.x, target_pose.pose.position.y, adj_height);
        ROS_INFO_THROTTLE(1.0, "Adjusting to (%.2f, %.2f)...", pose.pose.position.x, pose.pose.position.y);
        rate.sleep();
    }
}

void vertically_set_vel_to_bomb(float v, float bomb_height, float lowest_height, ros::Rate &rate)
{
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = v;
    bool isBombed = false;
    while (ros::ok() && current_pose.pose.position.z >= lowest_height)
    {
        ros::spinOnce();
        local_vel_pub.publish(vel);
        if (current_pose.pose.position.z <= bomb_height && !isBombed)
        {
            ROS_INFO("Releasing bomb %d...", target_index + 1);
            command = std::to_string(target_index + 1) + std::to_string(0) + "\n";
            ser.write(command);
            ROS_INFO_STREAM("Sent command to servo" << target_index + 1 << ": " << command);
            isBombed = true;
            // 标记当前目标为已投掷
            targetArray[current_index].isBombed = true;
            // 重置移动靶标志
            if(is_moving_target) is_moving_target = false;
        }
        rate.sleep();
    }
}

void check_left_bombs(ros::Rate &rate)
{
    while (target_index < 3)
    {
        ROS_INFO("Still have bombs left, dropping now...");
        command = std::to_string(target_index + 1) + std::to_string(0) + "\n";
        ser.write(command);
        ROS_INFO_STREAM("Sent command to servo" << target_index + 1 << ": " << command);
        hovering(0.8, 0.5, false, rate);
        target_index++;
    }

    // 若串口未关闭，关闭串口
    if (ser.isOpen())
        ser.close();

}

void update_params(ros::NodeHandle &nh)
{
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
        param_set_pub.publish(param_set_msg);
    }
}

void oa_check_arrival()
{
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
            need_exit = true;
        }
}

void auto_land_and_disarmed(mavros_msgs::SetMode land_set_mode)
{
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
    }
}

void return_init()
{
    return_state_msg.data = true;
    return_state_pub.publish(return_state_msg);
    is_return = true;
    is_stuck = false;
}

void return_check_stuck(ros::Rate &rate)
{
    if (is_stuck)
    {
        ROS_WARN("Ego-planner is stuck. Trying directly getting over obstacles to return.");

        nav_state_msg.data = false;
        nav_state_pub.publish(nav_state_msg);

        // 速度控制较低速上升，防止吹跑已经投放好的弹
        vertically_set_vel_to_move(0.7, 3.0, rate);

        horizontally_set_pose_to_move(0, 0, rate);

        vertically_set_pose_to_move(0.8, rate);

        mission_state = ADJUSTING;
        need_exit = true;
    }
}

void return_check_arrival()
{
    if (distance(current_pose, nav_pose.pose.position) < threshold_distance)
    {
        ROS_INFO("Reached returning target.");

        nav_state_msg.data = false;
        nav_state_pub.publish(nav_state_msg);

        mission_state = ADJUSTING;
        need_exit = true;
    }
}

void horizontally_set_pose_to_move(double x, double y, ros::Rate &rate)
{
    set_and_pub_pose(x, y, current_pose.pose.position.z);
    while (distance(current_pose, pose.pose.position) > threshold_distance)
    {
        ros::spinOnce();
        local_pos_pub.publish(pose);
        rate.sleep();
    }
}

// 开视觉的投弹
void follow_bombing(ros::Rate &rate, float kp)
{
    // 边下降边投弹
    ROS_INFO("Start follow_bombing...");
    vel.linear.z = -2.0;
    bool isBombed = false;
    while (ros::ok() && current_pose.pose.position.z >= 0.4)
    {
        ros::spinOnce();
        vel.linear.x = kp * target_pose.pose.position.x - current_pose.pose.position.x;
        vel.linear.y = kp * target_pose.pose.position.y - current_pose.pose.position.y;
        local_vel_pub.publish(vel);
        if (current_pose.pose.position.z <= 0.5 && !isBombed)
        {
            ROS_INFO("Releasing bomb %d...", target_index + 1);
            command = std::to_string(target_index + 1) + std::to_string(0) + "\n";
            ser.write(command);
            ROS_INFO_STREAM("Sent command to servo" << target_index + 1 << ": " << command);
            isBombed = true;
            // 标记当前目标为已投掷
            targetArray[current_index].isBombed = true;
        }
        rate.sleep();
    }

    // 到点后悬停0.5秒,然后索引自增
    hovering(0.4, 0.5, false, rate);
    target_pose.pose.position.z = -1; // 防止视觉节点没有来得及发布新目标点或发布未找到目标点的消息导致重复导航和投弹

    // 速度控制较低速上升，防止吹跑已经投放好的弹
    ROS_INFO("Bombing %d done, rising to normal flight height.", target_index + 1);

    vertically_set_vel_to_move(0.2, 0.8, rate);

    if (++target_index >= 3)
        mission_state = OBSTACLE_AVOIDING;
    else if (target_pose.pose.position.z != -1)
        mission_state = BOMB_NAVIGATING;
    else
        mission_state = SEARCHING;
}
