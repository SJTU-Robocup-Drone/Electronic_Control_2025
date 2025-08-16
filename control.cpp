#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <queue>
#include <vector>
#include <deque>
// // ------------- 新增点云占据判断 -------------
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// #include <unordered_set>
// #include <mutex>
// #include <memory>
// #include <cmath>

enum MissionState {
    TAKEOFF,
    OVERLOOKING, //飞至3.2米悬停寻找靶标
    SEARCHING,  // 视觉没找到靶标时遍历地图
    BOMB_NAVIGATING, // 导航到靶标
    ADJUSTING, // 让飞机进一步对准靶标
    BOMBING,
    OBSTACLE_AVOIDING, // 导航到障碍区并避障
    DESCENDING,
    LANDING,
    FOLLOW,
    RETURNING
};
MissionState mission_state = TAKEOFF;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher nav_goal_pub;
ros::Publisher nav_state_pub; // 给导航节点发送开启或关闭的指令
ros::Publisher vision_state_pub; // 发布视觉开启或关闭扫描的指令
ros::Publisher return_state_pub;
ros::Publisher is_done_pub;
ros::Subscriber target_sub; //从视觉对接节点的话题接收当前靶标的坐标
ros::Publisher param_set_pub;
ros::Publisher manba_pub;
ros::Subscriber local_pos_sub;
ros::Subscriber state_sub;
ros::Subscriber nav_check_sub; // 检查ego-planner是否卡住
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

geometry_msgs::PoseStamped initial_pose; // 用于起飞时的初始姿态
geometry_msgs::PoseStamped pose; // 直接发送给飞控的目标点
geometry_msgs::Twist vel; // 直接发送给飞控的速度
geometry_msgs::PoseStamped target_pose; // 接收靶标位置，若找到则z坐标为避障飞行高度，z坐标为-1表示未找到靶标
geometry_msgs::PoseStamped nav_pose; // 发布到egoplanner的目标点
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_nav_pose; // 当前导航点
geometry_msgs::Point last_nav_point; // 上一个导航点
geometry_msgs::Point first_target_point; // OVERLOOKING识别出的靶标位置
geometry_msgs::Point last_target_point; // 上一个识别出的靶标位置
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
std_msgs::Bool nav_state_msg; // 发布导航状态的消息
std_msgs::Bool vision_state_msg; // 发布视觉状态的消息
std_msgs::Bool return_state_msg;
std_msgs::Bool is_done_msg;
std_msgs::Bool param_set_msg;
bool is_stuck = false; // 标记当前导航是否卡住
bool is_once_stuck = false;
bool is_return = false; // 标记当前是否返航状态
bool is_vision_right = true; // 标记视觉是否误识别
bool adjust_has_target = false; // 标记ADJUSTING阶段是否扫到目标点
int vision_bias_cnt = 0;
int target_index = 0; // 当前投弹索引，也可视作已经投弹的数量
bool is_param_set = false; // 标记是否将参数调整为障碍区所需
std_msgs::Int32 target_index_msg;
ros::Time last_request;
ros::Time takeoff_request; // 用于单独记录起飞请求的时间
ros::Time nav_request; // 用于记录上一次正常导航命令的时间
bool is_takeoff = false; // 标记是否已经起飞
double offset[3][2] = {{0, 0.18}, {0.18, 0}, {0, -0.18}}; // 用于存储每个弹相对于无人机中心的偏移量，需要根据实际情况测定
bool is_moving_target = false; // 标记当前靶标是否为移动靶

const double threshold_distance = 0.1; // 定义到达目标点的距离阈值


double distance(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::Point& point) {
    return std::sqrt(std::pow(current_pose.pose.position.x - point.x, 2) +
                     std::pow(current_pose.pose.position.y - point.y, 2) +
                     std::pow(current_pose.pose.position.z - point.z, 2));
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
    if(target_index < 3 && target_pose.pose.position.z != -1){
        ROS_INFO_THROTTLE(2.0, "Target found at (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        if((mission_state == ADJUSTING && !is_return) || mission_state == FOLLOW){
            target_pose.pose.position.x += offset[target_index][0];
            target_pose.pose.position.y += offset[target_index][1];
        }
    }
    if(target_pose.pose.position.z == 2.0) is_moving_target = true; // 如果z坐标为2.0，说明是移动靶

    if(mission_state == ADJUSTING) { // 如果调整阶段靶标位置连续两次偏差超过0.5米，则认为视觉误识别（假定OVERLOOKING的识别结果没有问题，因为之前没有出过错）
        if(distance(target_pose, last_target_point) > 0.5){
            if(++vision_bias_cnt >= 2){
                vision_bias_cnt = 0;
                is_vision_right = false;
            }
        }
        else vision_bias_cnt = 0;
        last_target_point = target_pose.pose.position;
        adjust_has_target = true;
    }
}

struct TimedPose
{
    geometry_msgs::Point pos;
    ros::Time stamp;
};

std::deque<TimedPose> history_;
const size_t MAX_HISTORY = 5; // 缓冲 5 帧

void visionCallback(const geometry_msgs::PoseStamped &msg)
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
    if (history_.size() < 2)
        return geometry_msgs::Vector3();

    double dt_total = 0.05;
    geometry_msgs::Vector3 dpos;
    dpos.x = history_.back().pos.x - history_.front().pos.x;
    dpos.y = history_.back().pos.y - history_.front().pos.y;
    dpos.z = history_.back().pos.z - history_.front().pos.z;

    geometry_msgs::Vector3 v;
    v.x = dpos.x / dt_total;
    v.y = dpos.y / dt_total;
    v.z = dpos.z / dt_total;
    return v;
}

geometry_msgs::Point predictNextPosition(double predict_dt)
{
    auto v = computeAverageVelocity();
    geometry_msgs::Point p_pred;
    auto &p_last = history_.back().pos;
    p_pred.x = p_last.x + v.x * predict_dt;
    p_pred.y = p_last.y + v.y * predict_dt;
    p_pred.z = p_last.z + v.z * predict_dt;
    return p_pred;
}
std::vector<geometry_msgs::Point> searching_points;
int searching_index = 0; // 当前搜索点的索引

std::vector<geometry_msgs::Point> obstacle_zone_points;
int obstacle_zone_index = 0;

// // --- 最小占据查询：固定读取 /world/grid_map/occupancy_inflate ---
// namespace occ_inflate
// {

//     static constexpr double kRes = 0.10; // << 固定网格分辨率（和 grid_map 一致时无需改）
//     static const char *const kTopic = "/world/grid_map/occupancy_inflate";

//     struct Key
//     {
//         long long x, y, z;
//         bool operator==(const Key &o) const { return x == o.x && y == o.y && z == o.z; }
//     };
//     struct KeyHasher
//     {
//         size_t operator()(const Key &k) const
//         {
//             size_t h = 1469598103934665603ULL;
//             auto mix = [&](long long v)
//             { h ^= std::hash<long long>{}(v); h *= 1099511628211ULL; };
//             mix(k.x);
//             mix(k.y);
//             mix(k.z);
//             return h;
//         }
//     };

//     class Store
//     {
//     public:
//         explicit Store(ros::NodeHandle &nh)
//         {
//             sub_ = nh.subscribe(kTopic, 1, &Store::cb, this);
//             ROS_INFO_STREAM("[occ] subscribe " << kTopic << " (res=" << kRes << " m)");
//         }

//         bool occupied(const geometry_msgs::Point &p) const
//         {
//             Key c = toKey(p.x, p.y, p.z);
//             std::lock_guard<std::mutex> lk(m_);
//             // 直接命中或检查 26 邻，抵御取整误差
//             if (vox_.count(c))
//                 return true;
//             for (int dx = -1; dx <= 1; ++dx)
//                 for (int dy = -1; dy <= 1; ++dy)
//                     for (int dz = -1; dz <= 1; ++dz)
//                         if (vox_.count({c.x + dx, c.y + dy, c.z + dz}))
//                             return true;
//             return false;
//         }

//     private:
//         static Key toKey(double x, double y, double z)
//         {
//             const double inv = 1.0 / kRes;
//             return Key{(long long)llround(x * inv),
//                        (long long)llround(y * inv),
//                        (long long)llround(z * inv)};
//         }

//         void cb(const sensor_msgs::PointCloud2ConstPtr &msg)
//         {
//             std::unordered_set<Key, KeyHasher> s;
//             s.reserve(msg->width * msg->height * 2 + 1024);
//             for (sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y"), it_z(*msg, "z");
//                  it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
//             {
//                 const float x = *it_x, y = *it_y, z = *it_z;
//                 if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
//                     continue;
//                 s.insert(toKey(x, y, z));
//             }
//             std::lock_guard<std::mutex> lk(m_);
//             vox_.swap(s);
//         }

//         ros::Subscriber sub_;
//         mutable std::mutex m_;
//         std::unordered_set<Key, KeyHasher> vox_;
//     };

//     static std::unique_ptr<Store> G;

// } // namespace occ_inflate

// // --- 对外函数 ---
// bool pointOccupied(const geometry_msgs::Point &p)
// {
//     if (!occ_inflate::G)
//         return false; // 尚未订阅完成前，按未占据
//     return occ_inflate::G->occupied(p);
// }

void nav_check_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    if(!nav_state_msg.data) nav_request = ros::Time::now(); // 不处于导航状态时，持续更新时间并直接返航
    else{
        current_nav_pose.header = msg -> header;
        current_nav_pose.pose.position = msg -> position;
        if(distance(current_nav_pose, last_nav_point) > 1e-2){ // 导航未卡住，持续更新时间和上一个导航点并直接返回
            last_nav_point = current_nav_pose.pose.position;
            nav_request = ros::Time::now();
        }
        else if(ros::Time::now() - nav_request > ros::Duration(10.0)){ // 若导航卡住超过十秒，标记导航已卡住并更新时间
            is_stuck = true;
            nav_request = ros::Time::now();
        }
    }
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    
    // occ_inflate::G.reset(new occ_inflate::Store(nh));  //点云占据查询初始化
    target_pose.pose.position.z = -1;

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    nav_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/control/move_base_simple/goal", 10);
    nav_state_pub = nh.advertise<std_msgs::Bool>("/nav_state", 10);
    return_state_pub = nh.advertise<std_msgs::Bool>("/return_state", 10);
    is_done_pub = nh.advertise<std_msgs::Bool>("/done_state", 10);
    param_set_pub = nh.advertise<std_msgs::Bool>("/obs_param_set", 10);
    manba_pub = nh.advertise<std_msgs::Int32>("/manba_input", 10);

    vision_state_pub = nh.advertise<std_msgs::Bool>("/vision_state", 10);
    target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target", 10, target_cb);

    local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/odom_high_freq", 10, pose_cb);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    nav_check_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50, nav_check_cb);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    std::vector<double> searching_x_points, searching_y_points, searching_z_points;
    if (nh.getParam("searching_points/x", searching_x_points) &&
        nh.getParam("searching_points/y", searching_y_points) &&
        nh.getParam("searching_points/z", searching_z_points)) {
        
        searching_points.clear();
        for (size_t i = 0; i < searching_x_points.size(); ++i) {
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
        nh.getParam("obstacle_points/z", obstacle_z_points)) {

        obstacle_zone_points.clear();
        for (size_t i = 0; i < obstacle_x_points.size(); ++i) {
            geometry_msgs::Point point;
            point.x = obstacle_x_points[i];
            point.y = obstacle_y_points[i];
            point.z = obstacle_z_points[i];
            obstacle_zone_points.push_back(point);
        }
    }

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
    initial_pose = current_pose; // 记录起飞前的初始位姿
    last_nav_point  = current_pose.pose.position; // 初始化ego-planner检查相关变量
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
                    // if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    //     if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    //         ROS_INFO("Offboard enabled");
                    //     }
                    //     last_request = ros::Time::now();
                    // }
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && current_state.mode == "OFFBOARD"){
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

                    if(!is_takeoff) takeoff_request = ros::Time::now();
                    if(current_state.mode == "AUTO.TAKEOFF" && !is_takeoff){
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
                        for (int i = 0; i < 100; i++)
                        {
                            ros::spinOnce();
                            local_pos_pub.publish(pose);
                            ros::Duration(0.01).sleep();
                        }
                        break; // 跳出循环，进入导航状态
                    }
                }

                break;
            }

            case OVERLOOKING:
            {
                vision_state_msg.data = true; // 开启视觉扫描
                ROS_INFO("Stablizing first...");
                pose = current_pose;
                last_request = ros::Time::now();
                while(ros::Time::now() - last_request < ros::Duration(2.0)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                ROS_INFO("Start overlooking for target. Rising to overlooking position...");
                pose.pose.position.z = 3.0; // 悬停高度
                // 速度控制较低速上升，防止吹跑已经投放好的弹
                vel.linear.x = 0.0;
                vel.linear.y = 0.0;
                vel.linear.z = 0.7;
                while(ros::ok() && current_pose.pose.position.z <= 3.0) {
                    ros::spinOnce();
                    local_vel_pub.publish(vel);
                    vision_state_pub.publish(vision_state_msg);
                    rate.sleep();
                }

                ROS_INFO("Reached overlooking position. Scanning for target...");
                last_request = ros::Time::now();
                while(ros::ok() && ros::Time::now() - last_request < ros::Duration(10.0)) { // 等待视觉识别靶标
                    ros::spinOnce();
                    if (target_pose.pose.position.z != -1)
                        break;
                    vision_state_pub.publish(vision_state_msg);
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
                    vision_state_pub.publish(vision_state_msg);
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
                if(!is_stuck){ // 正常导航时，跳过放弃的点向前导航
                    while(searching_points[searching_index].z == -1) searching_index++;
                }
                else{ // 不正常导航，即ego-planner卡住时，跳过放弃的点向后导航，同时重置标志位并记忆该信息
                    while(searching_points[searching_index].z == -1) searching_index--;
                    is_stuck = false;
                    is_once_stuck = true;
                }

                // 如果所有搜索点都已访问或ego-planner卡得索引降至零以下了，为了防止下标越界直接返航
                if(searching_index >= searching_points.size() || searching_index < 0) {
                    is_done_msg.data = true;
                    is_done_pub.publish(is_done_msg);
                    last_request = ros::Time::now();
                    pose = current_pose;
                    while(ros::Time::now() - last_request < ros::Duration(2.0)){ // 如果帐篷还没投，就投帐篷
                        ros::spinOnce();
                        local_pos_pub.publish(pose);
                        rate.sleep();
                    }
                    if(target_pose.pose.position.z != -1) mission_state = BOMB_NAVIGATING;
                    else{
                        ROS_WARN("All searching points reached. Navigating now.");
                        mission_state = OBSTACLE_AVOIDING;
                    }
                    break;
                }

                // 发布航点并进入导航状态前先悬停，给建图留时间
                ROS_INFO("Hovering before navigating...");
                last_request = ros::Time::now();
                pose = current_pose;
                pose.pose.position.z = 0.9;
                while(ros::Time::now() - last_request < ros::Duration(5.0)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                // 发布航点,更新导航时间
                nav_state_msg.data = true;
                nav_state_pub.publish(nav_state_msg);
                nav_pose.header.frame_id = "map";
                nav_pose.header.stamp = ros::Time::now();
                nav_pose.pose.position = searching_points[searching_index];
                // if(pointOccupied(nav_pose.pose.position)){
                //     ROS_WARN("Current Searching point in obstacle! Searching directly for next point.");
                //     searching_index++;
                //     break;
                // }
                nav_goal_pub.publish(nav_pose);
                nav_request = ros::Time::now();

                // 轨迹跟踪与检查
                ROS_INFO("Searching for target %d...", searching_index + 1);
                while(ros::ok()) {
                    ros::spinOnce();
                    // if(pointOccupied(nav_pose.pose.position)){
                    //     ROS_WARN("Current Searching point in obstacle! Searching directly for next point.");
                    //     searching_index++;
                    //     break;
                    // }
                    // local_pos_pub.publish(pose);
                    // ROS_INFO_THROTTLE(1.0, "Heading to(%.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                    
                    if(is_stuck){ // 如果ego-planner卡住了，放弃当前搜索点，同时暂时关闭导航
                        ROS_WARN("Ego-planner is stuck. Trying navigating to last searching point and aborting current searching point...");
                        searching_points[searching_index].z = -1;
                        nav_state_msg.data = false;
                        nav_state_pub.publish(nav_state_msg);
                        break;
                    }

                    if(distance(current_pose, nav_pose.pose.position) < threshold_distance) {
                        ROS_INFO("Reached searching point %d.", searching_index + 1);
                        searching_index++; 
                        if(target_pose.pose.position.z != -1) mission_state = BOMB_NAVIGATING;
                        else if(is_once_stuck){
                            mission_state = SEARCHING;
                            is_once_stuck = false;
                        }
                        else mission_state = OVERLOOKING;
                        nav_state_msg.data = false;
                        nav_state_pub.publish(nav_state_msg);
                        break;
                    }

                    rate.sleep();
                }

                break;
            }

            case BOMB_NAVIGATING:
            {
                ROS_INFO("Hovering before navigating...");
                last_request = ros::Time::now();
                pose = current_pose;
                pose.pose.position.z = 0.9;
                while(ros::Time::now() - last_request < ros::Duration(5.0)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                // 发布航点并更新导航时间,初始化第一个和上一个靶标点
                nav_state_msg.data = true;
                nav_state_pub.publish(nav_state_msg);
                nav_pose.header.frame_id = "map";
                nav_pose.header.stamp = ros::Time::now();
                nav_pose.pose.position = target_pose.pose.position;
                nav_goal_pub.publish(nav_pose);
                nav_request = ros::Time::now();
                first_target_point = nav_pose.pose.position;
                last_target_point = nav_pose.pose.position;
                ROS_INFO("Navigating to target at (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

                // 轨迹跟踪与检查
                while(ros::ok() && distance(current_pose, nav_pose.pose.position) > threshold_distance) {
                    ros::spinOnce();
                    // local_pos_pub.publish(pose);
                    // ROS_INFO_THROTTLE(1.0, "Heading to(%.2f, %.2f, %.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                    if(is_stuck){ // 如果ego-planner卡住了，进入搜索状态并放弃当前搜索点，同时暂时关闭导航
                        ROS_WARN("Ego-planner is stuck. Trying navigating to last searching point and aborting current searching point...");
                        mission_state = SEARCHING;
                        searching_points[searching_index].z = -1;
                        nav_state_msg.data = false;
                        nav_state_pub.publish(nav_state_msg);
                        break;
                    }

                    // 到目标点后进入调整对准状态
                    if(distance(current_pose, nav_pose.pose.position) < threshold_distance) {
                        ROS_INFO("Reached bomb target %d.", target_index + 1);
                        nav_state_msg.data = false;
                        nav_state_pub.publish(nav_state_msg);
                        if(is_moving_target)  {
                            mission_state = FOLLOW;
                            ROS_INFO("The target is moving, following it.");
                            is_moving_target = false; // 重置标志
                        }
                        else mission_state = ADJUSTING;
                        break;
                    }

                    rate.sleep();
                }

                break;
            }

            case ADJUSTING:
            {
                /*旧版本的adjusting
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
                */
                vision_state_msg.data = true; // 开启视觉扫描
                adjust_has_target = false;
                ROS_INFO("2nd time of visual scanning...");
                pose.header.frame_id = "map";
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 1.0; 
                last_request = ros::Time::now();
                while(ros::ok() && ros::Time::now() - last_request < ros::Duration(5.0)) { // 等待视觉识别靶标
                    ros::spinOnce();
                    vision_state_pub.publish(vision_state_msg);
                    pose.header.stamp = ros::Time::now();// 更新时间戳
                    local_pos_pub.publish(pose); // 保持悬停
                    rate.sleep();
                }
               
                vision_state_msg.data = false; // 关闭视觉扫描

                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                // 判断视觉误识别
                if(!is_vision_right){
                    is_vision_right = true;
                    ROS_WARN("High target bias. Vision scanning of ADJUSTING may be wrong. Bombing depending on OVERLOOKING...");
                    pose.pose.position = first_target_point;
                    break;
                }
                else if(!adjust_has_target){
                    ROS_WARN("Adjusting stage hasn't scanned a target. Vision scanning of OVERLOOKING may be wrong. Directly turning to SEARCHING mode...");
                    mission_state = SEARCHING;
                    break;
                }
                else{
                    pose.pose.position.x = target_pose.pose.position.x;
                    pose.pose.position.y = target_pose.pose.position.y;
                    pose.pose.position.z = target_pose.pose.position.z;
                }
                ROS_INFO("Adjusting position to target...");
                while (distance(current_pose, pose.pose.position) > threshold_distance && ros::ok()) { // 临时减小距离阈值
                    ros::spinOnce();
                    pose.header.stamp = ros::Time::now();
                    vision_state_pub.publish(vision_state_msg);
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }
                if(is_return) mission_state = DESCENDING;
                else mission_state = BOMBING; // 调整完成后进入投弹状态
                break;            
            }

            case BOMBING:
            {
                ROS_INFO("Start bombing...");
                // 先下降到较低高度，并调整姿态后再投弹
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                // pose.pose.position.x = current_pose.pose.position.x;
                // pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 0.2;
                pose.pose.orientation = initial_pose.pose.orientation;
                while(ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance/2.0){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                //到点后悬停0.5秒并投弹，然后索引自增
                last_request = ros::Time::now();
                target_index_msg.data = target_index;
                manba_pub.publish(target_index_msg);
                while(ros::ok() && ros::Time::now() - last_request < ros::Duration(0.5)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }
                target_pose.pose.position.z = -1; // 防止视觉节点没有来得及发布新目标点或发布未找到目标点的消息导致重复导航和投弹

                ROS_INFO("Bombing %d done, rising to normal flight height.", target_index + 1);
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = 1.0; 
                // 速度控制较低速上升，防止吹跑已经投放好的弹
                vel.linear.x = 0.0;
                vel.linear.y = 0.0;
                vel.linear.z = 0.2;
                while(ros::ok() && current_pose.pose.position.z <= 1.0){
                    ros::spinOnce();
                    local_vel_pub.publish(vel);
                    rate.sleep();
                }

                if(++target_index >= 3) mission_state = OBSTACLE_AVOIDING;
                else if(target_pose.pose.position.z == -1) mission_state = SEARCHING;
                else mission_state = BOMB_NAVIGATING;
                break;
            }

            case OBSTACLE_AVOIDING: // TODO: 这里可以添加一个类似投弹后的adjusting状态，以保证精准降落
            {
                if(!is_param_set && obstacle_zone_index >= 1){
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

                ROS_INFO("Hovering before navigating...");
                last_request = ros::Time::now();
                pose = current_pose;
                pose.pose.position.z = 0.7;
                while(ros::Time::now() - last_request < ros::Duration(5.0)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }
                
                // 发布航点
                if (obstacle_zone_index < obstacle_zone_points.size()) {
                    nav_state_msg.data = true;
                    nav_state_pub.publish(nav_state_msg);
                    nav_pose.header.frame_id = "map";
                    nav_pose.header.stamp = ros::Time::now();
                    nav_pose.pose.position = obstacle_zone_points[obstacle_zone_index];
                    nav_goal_pub.publish(nav_pose);
                    nav_request = ros::Time::now();
                }

                ROS_INFO("EGO thinking...");
                last_request = ros::Time::now();
                pose = current_pose;
                pose.pose.position.z = 0.7;
                while(ros::Time::now() - last_request < ros::Duration(3.0)){
                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();
                }

                // 轨迹跟踪与检查
                while(ros::ok()) {
                    ros::spinOnce();
                    // 到点时索引自增，若到达全部目标点，则进入降落模式
                    if(distance(current_pose, obstacle_zone_points[obstacle_zone_index]) < threshold_distance) {
                        ROS_INFO("reached mid point!");
                        if (++obstacle_zone_index >= obstacle_zone_points.size()) {
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

                // 自动降落和锁桨
                if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                    ROS_INFO_THROTTLE(2.0, "Landing initiated");
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

            case FOLLOW:
            {
                int follow_timer=0;
                vision_state_msg.data = true; // 开启视觉扫描
                ROS_INFO("Follow to target...");
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
                while(ros::ok() && ros::Time::now() - last_request < ros::Duration(5.0)) { // 等待视觉识别靶标
                    ros::spinOnce();
                    if (target_pose.pose.position.z != -1)
                        break;
                    vision_state_pub.publish(vision_state_msg);
                    local_pos_pub.publish(pose); // 保持悬停
                    rate.sleep();
                }

                while(follow_timer<=20){
                    vision_state_pub.publish(vision_state_msg);

                    if(target_pose.pose.position.z != -1) { // 找到了靶标，进入靶标跟随状态
                        ROS_INFO_THROTTLE(2.0, "Target found, keeping following to target.");
                    } else {
                        mission_state = SEARCHING; // 没有找到靶标，进入搜索状态
                        ROS_INFO("No target found, starting searching.");
                        break;
                    }
                    
                    if (target_pose.pose.position.z != -1)        //接受有效点后放入缓存
                        visionCallback(target_pose);

                    pose.pose.position = predictNextPosition(0.05);  //计算预测坐标

                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.z = 2;

                    //                    pose.pose.position.z = target_pose.pose.position.z;
                    if(distance(current_pose, pose.pose.position) < threshold_distance / 2.0 && ros::ok())       //记录成功跟上的次数
                        follow_timer++;
                    else
                        follow_timer=0;

                    ros::spinOnce();
                    local_pos_pub.publish(pose);
                    rate.sleep();

                    if(follow_timer>=20) {
                        ROS_INFO("FOLLOW success (%d cycles), switching to BOMBING", follow_timer);
                        mission_state = BOMBING;// 确认跟随后进入投弹状态                        
                    }
                }
                break;
            }
            
            case RETURNING:
            {
                // int return_target = 0;
                // int point_num = 5;
                // searching_index = 11;
                // return_state_msg.data = true;
                // return_state_pub.publish(return_state_msg);
                // is_return = true;

                // ROS_INFO("Complete bombing.Start waiting for returning. ");
                // pose.header.frame_id = "map";
                // pose.header.stamp = ros::Time::now();
                // pose.pose.position.x = current_pose.pose.position.x;
                // pose.pose.position.y = current_pose.pose.position.y;
                // pose.pose.position.z = 1; // 悬停高度

                // last_request = ros::Time::now();
                // while (ros::ok() && ros::Time::now() - last_request < ros::Duration(5.0))
                // { 
                //     ros::spinOnce();
                //     local_pos_pub.publish(pose); // 保持悬停
                //     rate.sleep();
                // }

                // ROS_INFO("Start returning to landing space. ");
                // for (int i = 0; i < point_num; i++){
                //     pose.header.frame_id = "map";
                //     pose.header.stamp = ros::Time::now();
                //     pose.pose.position = searching_points[searching_index];

                //     while (ros::ok() && distance(current_pose, pose.pose.position) > threshold_distance)
                //     {
                //         ros::spinOnce();
                //         rate.sleep();
                //         // 到目标点后进入调整对准状态
                //         if (distance(current_pose, pose.pose.position) < threshold_distance)
                //         {
                //             return_target++;
                //             searching_index--;
                //             ROS_INFO("Reached returning target %d.", return_target + 1);
                //             break;
                //         }
                //     }
                //     mission_state = BOMB_NAVIGATING;

                // 给视觉节点发返航消息，同时标记返航状态，并先重置标志位
                return_state_msg.data = true;
                return_state_pub.publish(return_state_msg);
                is_return = true;
                is_stuck = false;

                // 发布航点并更新导航时间
                nav_state_msg.data = true;
                nav_state_pub.publish(nav_state_msg);
                nav_pose.header.frame_id = "map";
                nav_pose.header.stamp = ros::Time::now();
                nav_pose.pose.position.x = 0.0;
                nav_pose.pose.position.y = 0.0;
                nav_pose.pose.position.z = 1.0;
                nav_goal_pub.publish(nav_pose);
                nav_request = ros::Time::now();

                // 轨迹跟踪与检查
                while(ros::ok()) {
                    ros::spinOnce();
                    // 如果返航的时候ego-planner卡住了，放弃导航并直接翻越所有障碍物返回起点
                    if(is_stuck){
                        ROS_WARN("Ego-planner is stuck. Trying directly getting over obstacles to return.");
                        
                        nav_state_msg.data = false;
                        nav_state_pub.publish(nav_state_msg);

                        pose = current_pose;
                        pose.pose.position.z = 3.2;
                        while(current_pose.pose.position.z <= 3.0){
                            ros::spinOnce();
                            local_pos_pub.publish(pose);
                            rate.sleep();
                        }
                        pose.pose.position.x = 0.0;
                        pose.pose.position.y = 0.0;
                        while(distance(current_pose, pose.pose.position) > threshold_distance){
                            ros::spinOnce();
                            local_pos_pub.publish(pose);
                            rate.sleep();
                        }

                        mission_state = ADJUSTING;
                        break;
                    }

                    // 到目标点后进入调整状态，借助视觉进一步定位起点
                    if(distance(current_pose, nav_pose.pose.position) < threshold_distance) {
                        ROS_INFO("Reached returning target.");

                        nav_state_msg.data = false;
                        nav_state_pub.publish(nav_state_msg);

                        mission_state = ADJUSTING;
                        break;
                    }
                    rate.sleep();
                }

                break;
            }    
        }
    }
    return 0;
}

