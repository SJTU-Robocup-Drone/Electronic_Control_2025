#include "function.h"
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/Point.h>

geometry_msgs::Point createPoint(double x, double y, double z) {
   geometry_msgs::Point p;
   p.x = x; p.y = y; p.z = z;
   return p;
}
// 定义点集（会在全局范围内用到）
std::vector<geometry_msgs::Point> searching_points;
std::vector<geometry_msgs::Point> obstacle_zone_points;

void hovering(float z, float time, bool if_exit, ros::Rate &rate)
{
    last_request = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = current_pose.pose.position.x;
    pose.pose.position.y = current_pose.pose.position.y;
    pose.pose.position.z = z;
    pose.pose.orientation = initial_pose.pose.orientation; // 将姿态恢复为初始朝向
    last_request = ros::Time::now();
    ROS_INFO("Hovering at height %.2f for %.2f seconds.", z, time);
    while (ros::ok() && ros::Time::now() - last_request < ros::Duration(time))
    {
        ros::spinOnce();
        local_pos_pub.publish(pose);          // 保持悬停
        rate.sleep();
        if (if_exit == true)
        {
            vision_state_pub.publish(vision_state_msg);
            if (target_pose.pose.position.z != -1)
                break;
        }
    }
}

void set_and_pub_pose(float x, float y, float z)
{
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    local_pos_pub.publish(pose);
}

void set_and_pub_nav(float x, float y, float z)
{
    nav_state_msg.data = true;
    nav_state_pub.publish(nav_state_msg);
    nav_pose.header.frame_id = "map";
    nav_pose.header.stamp = ros::Time::now();
    nav_pose.pose.position.x = x;
    nav_pose.pose.position.y = y;
    nav_pose.pose.position.z = z;
    nav_goal_pub.publish(nav_pose);
    nav_request = ros::Time::now();
}

double distance(const geometry_msgs::PoseStamped &current_pose_, const geometry_msgs::Point &point)
{
    const auto &p = current_pose_.pose.position;
    const double dx = p.x - point.x, dy = p.y - point.y, dz = p.z - point.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 读取参数到既有变量名
void init_params(ros::NodeHandle &nh)
{
    std::string filename = "/home/amov/board_ws/src/board_ros/points/param.txt";
    std::string line;
    std::string valid_str;
    int pos1 = 0;
    int pos2 = 0;

    std::ifstream infile(filename);
    if(!infile.is_open()){
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

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

    infile.close();

    // std::vector<double> searching_x_points, searching_y_points, searching_z_points;
    // if (nh.getParam("searching_points/x", searching_x_points) &&
    //     nh.getParam("searching_points/y", searching_y_points) &&
    //     nh.getParam("searching_points/z", searching_z_points))
    // {

    //     searching_points.clear();
    //     for (size_t i = 0; i < searching_x_points.size(); ++i)
    //     {
    //         geometry_msgs::Point point;
    //         point.x = searching_x_points[i];
    //         point.y = searching_y_points[i];
    //         point.z = searching_z_points[i];
    //         searching_points.push_back(point);
    //     }
    // }

    // std::vector<double> obstacle_x_points, obstacle_y_points, obstacle_z_points;
    // if (nh.getParam("obstacle_points/x", obstacle_x_points) &&
    //     nh.getParam("obstacle_points/y", obstacle_y_points) &&
    //     nh.getParam("obstacle_points/z", obstacle_z_points))
    // {

    //     obstacle_zone_points.clear();
    //     for (size_t i = 0; i < obstacle_x_points.size(); ++i)
    //     {
    //         geometry_msgs::Point point;
    //         point.x = obstacle_x_points[i];
    //         point.y = obstacle_y_points[i];
    //         point.z = obstacle_z_points[i];
    //         obstacle_zone_points.push_back(point);
    //     }
    // }
}

// 速度限幅
void velocity_limited(double &vx, double &vy, double vmax)
{
    const double n = std::sqrt(vx * vx + vy * vy);
    if (n > vmax && n > 1e-6)
    {
        const double s = vmax / n;
        vx *= s;
        vy *= s;
    }
}

// 数值死区函数
double clip(double v, double low, double high)
{
    return std::max(low, std::min(high, v));
}

// 2D单位化（零向量时给一个默认方向）
static void unit_dir(double x, double y, double &ux, double &uy)
{
    const double n = std::sqrt(x * x + y * y);
    if (n < 1e-6)
    {
        ux = 1.0;
        uy = 0.0;
        return;
    }
    ux = x / n;
    uy = y / n;
}

void computeOverheadVelocityCmd(const geometry_msgs::Vector3 &tgt_vel, // 目标速度（来自视觉/KF）
                                ros::Time now_sec,
                                ros::Time target_stamp_sec,          // 目标状态时间戳
                                geometry_msgs::Twist &output_vel,    // 输出：XY 速度 + Z 速度
                                geometry_msgs::Point &output_ref_xy) // 输出：参考点（调试/可视化）
{
    FollowParams p; // 可在外层设参；这里用默认值即可（高度=1.5）
    const bool fresh = (now_sec- target_stamp_sec) < ros::Duration(p.lost_timeout);

    // === 1) 直线几何建模：沿线/垂线分解（在线学习） ============================
    // 在不新增全局的前提下，用 static 保留少量状态（仅在本进程内有效）
    static bool inited = false;
    static double ema_vx = 0.0, ema_vy = 0.0; // 速度方向的EMA
    static double ux = 1.0, uy = 0.0;         // 直线切向单位向量（沿线方向）
    static double s_min = +1e9, s_max = -1e9; // 观测到的端点（沿线标量）范围
    static double x0 = 0.0, y0 = 0.0;         // 沿线坐标的参考原点（首次见到目标的位置）

    // 目标当前位置（来自外层变量）
    const double xt = target_pose.pose.position.x;
    const double yt = target_pose.pose.position.y;

    if (!inited)
    {
        inited = true;
        x0 = xt;
        y0 = yt;
        ema_vx = tgt_vel.x;
        ema_vy = tgt_vel.y;
        unit_dir(ema_vx, ema_vy, ux, uy); // 以首次目标速度方向作为初始直线方向
        s_min = s_max = 0.0;              // 首点投影为0
    }

    // 指数滑动平均，平滑速度与方向
    if (fresh)
    {
        ema_vx = (1.0 - p.ema_alpha) * ema_vx + p.ema_alpha * tgt_vel.x;
        ema_vy = (1.0 - p.ema_alpha) * ema_vy + p.ema_alpha * tgt_vel.y;
        unit_dir(ema_vx, ema_vy, ux, uy); // 更新切向单位向量
    }

    // 将位置投影到当前直线方向，得到 along-track 标量 s
    const double dx0 = xt - x0, dy0 = yt - y0;
    const double s_now = dx0 * ux + dy0 * uy;

    // 在线学习端点：维护观测到的 s 的最小/最大（带轻微迟滞）
    const double hyster = 0.05; // 5cm 迟滞，抑制抖动误更新
    if (s_now < s_min - hyster)
        s_min = s_now;
    if (s_now > s_max + hyster)
        s_max = s_now;

    // 计算距最近端点的 along-track 距离（用于“掉头段”识别）
    const double dist_to_min = std::fabs(s_now - s_min);
    const double dist_to_max = std::fabs(s_now - s_max);
    const double dist_to_end = std::min(dist_to_min, dist_to_max);

    // === 2) 参考点：沿线超前 + 垂线纠偏 =======================================
    //   基础参考（含常规超前）：r0 = p_t + v_t * tau_eff
    //   再叠加垂线纠偏：r = r0 - k_perp * e_perp * n_hat
    //
    //   e_perp = (p_u - p_t) 在 n_hat 方向上的距离（n_hat 是 ux,uy 逆时针旋转90°）
    //   掉头附近( dist_to_end < d_turn 或 |v|<v_thresh ) 使用更小 tau_turn，并限速
    //
    double tau_eff = p.tau;
    const double vt_norm = std::sqrt(tgt_vel.x * tgt_vel.x + tgt_vel.y * tgt_vel.y);
    const bool near_turn = (dist_to_end < p.d_turn) || (vt_norm < p.v_thresh);
    if (near_turn)
    {
        // 线性混合：越靠近端点，越接近 tau_turn
        double w = 0.0;
        if (dist_to_end < p.d_turn)
            w = 1.0 - clip(dist_to_end / p.d_turn, 0.0, 1.0);
        if (vt_norm < p.v_thresh)
            w = std::max(w, 1.0); // 速度很慢，认定强掉头
        tau_eff = (1.0 - w) * p.tau + w * p.tau_turn;
    }

    // 参考点（沿切向）
    output_ref_xy.x = xt + tgt_vel.x * tau_eff;
    output_ref_xy.y = yt + tgt_vel.y * tau_eff;
    output_ref_xy.z = 1.5; // 固定高度（你的需求）

    // 垂直方向单位向量（切向旋转 90°）
    const double nx = -uy, ny = ux;

    // UAV 当前位姿
    const double xu = current_pose.pose.position.x;
    const double yu = current_pose.pose.position.y;

    // 垂向偏差：把 UAV 拉回直线（而不是只追目标瞬时点）
    const double e_perp = ((xu - xt) * nx + (yu - yt) * ny);
    output_ref_xy.x -= p.k_perp * e_perp * nx;
    output_ref_xy.y -= p.k_perp * e_perp * ny;

    // === 3) 速度命令（前馈 + P），掉头段自适应限速 ==============================
    double vx_cmd = 0.0, vy_cmd = 0.0;

    if (fresh)
    {
        const double ex = output_ref_xy.x - xu;
        const double ey = output_ref_xy.y - yu;

        // 前馈 + P
        vx_cmd = tgt_vel.x + p.kp_xy * ex;
        vy_cmd = tgt_vel.y + p.kp_xy * ey;

        // 掉头段自适应限速：让 UAV 在端点附近“收油”，不过冲
        if (near_turn)
        {
            velocity_limited(vx_cmd, vy_cmd, std::min(p.vmax_xy, p.v_slow_cap));
        }
        else
        {
            velocity_limited(vx_cmd, vy_cmd, p.vmax_xy);
        }
    }
    else
    {
        // 目标暂丢：水平停住，等下一帧
        vx_cmd = 0.0;
        vy_cmd = 0.0;
    }

    // 固定高度：不下发垂直速度
    output_vel = geometry_msgs::Twist();
    output_vel.linear.x = vx_cmd;
    output_vel.linear.y = vy_cmd;
    output_vel.linear.z = 0.0; // 固高 1.5m 建议在 MAVROS PositionTarget 的 position.z 体现
}