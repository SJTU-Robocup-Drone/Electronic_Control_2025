#include "function.h"
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>

geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}
// 定义点集（会在全局范围内用到）
std::vector<geometry_msgs::Point> searching_points;
std::vector<geometry_msgs::Point> obstacle_zone_points;
std::queue<RetryPoint> retry_searching_points;            // 针对searching点的重试队列
std::queue<RetryPoint> retry_navigating_points; // 针对避障点的重试队列

Target targetArray[7]; // 储存靶标信息的数组



/* ============ 全局变量（静态，外部不可见） ============ */
static double x_[4] = {0};        // [x, y, vx, vy]
static double P_[4][4] = {{0}};   // 协方差矩阵
static bool  firstCall_ = true;  // 自动用第一次观测做初始化
/* ======================================================= */

/* 4x4 矩阵乘法：C = A*B */
static void mul44(const double A[4][4], const double B[4][4], double C[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/* 4x4 矩阵加常数对角：A += diag(v) */
static void addDiag44(double A[4][4], double v)
{
    for (int i = 0; i < 4; ++i) A[i][i] += v;
}

/* 求 2x2 逆矩阵 */
static bool inv22(const double M[2][2], double Minv[2][2])
{
    double det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
    if (std::fabs(det) < 1e-6f) return false;
    double invDet = 1.0f / det;
    Minv[0][0] =  M[1][1] * invDet;
    Minv[0][1] = -M[0][1] * invDet;
    Minv[1][0] = -M[1][0] * invDet;
    Minv[1][1] =  M[0][0] * invDet;
    return true;
}

// 卡尔曼滤波
/*
使用示例：
double x, y, vx, vy;
KalmanUpdate(obsX, obsY, x, y, vx, vy);
printf("pos=(%.2f,%.2f)  vel=(%.2f,%.2f)\n", x, y, vx, vy);
*/
void KalmanUpdate(double obsX, double obsY, double& outX, double& outY,double& outVx, double& outVy)
{
    /* 第一次调用：用观测初始化状态，协方差置单位阵 */
    if (firstCall_)
    {
        x_[0] = obsX;  x_[1] = obsY;  x_[2] = 0;  x_[3] = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                P_[i][j] = (i == j) ? 10.0f : 0.0f;
        firstCall_ = false;
    }

    /* 1. 预测（恒定速度模型，dt=1） */
    double F[4][4] = {{1,0,1,0}, {0,1,0,1}, {0,0,1,0}, {0,0,0,1}};
    double Q[4][4] = {{0}};

    //TODO:目前通过速度判断靶标是否为静止靶标或者移动靶，之后需要将速度条件判断变成靶标类别判断，使得KF适用于静止和移动
    static double qStill = 1e-4f;     // 静止用
    static double qMove  = 0.1f;      // 运动用
    double spd = std::fabs(x_[2]) + std::fabs(x_[3]);
    double qScale = (spd < 0.02f) ? qStill : qMove;   // 速度阈值 0.02 可改
    addDiag44(Q, qScale);          // 过程噪声强度可调

    /* x = F*x */
    double tmp[4];
    for (int i = 0; i < 4; ++i)
    {
        tmp[i] = 0;
        for (int k = 0; k < 4; ++k) tmp[i] += F[i][k] * x_[k];
    }
    for (int i = 0; i < 4; ++i) x_[i] = tmp[i];

    /* P = F*P*F' + Q */
    double FP[4][4];
    mul44(F, P_, FP);
    double FT[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) FT[i][j] = F[j][i];
    mul44(FP, FT, P_);
    addDiag44(P_, 0.1f);   // 即 Q 的对角

    /* 2. 更新 */
    double H[2][4] = {{1,0,0,0}, {0,1,0,0}};
    double R[2][2] = {{1,0}, {0,1}};   // 观测噪声强度可调

    /* y = z - H*x */
    double y[2] = { obsX - x_[0], obsY - x_[1] };

    /* S = H*P*H' + R */
    double HP[2][4];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j)
        {
            HP[i][j] = 0;
            for (int k = 0; k < 4; ++k) HP[i][j] += H[i][k] * P_[k][j];
        }
    double S[2][2] = {{0}};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
        {
            S[i][j] = R[i][j];
            for (int k = 0; k < 4; ++k) S[i][j] += HP[i][k] * H[j][k];
        }

    /* K = P*H'*inv(S) */
    double Sinv[2][2];
    if (!inv22(S, Sinv)) return;          // 奇异则跳过更新
    double K[4][2] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                K[i][j] += P_[i][k] * H[0][k] * Sinv[k][j]   // H' 按列展开
                         +  P_[i][k+2] * H[1][k] * Sinv[k][j];

    /* x = x + K*y */
    for (int i = 0; i < 4; ++i) x_[i] += K[i][0]*y[0] + K[i][1]*y[1];

    /* P = (I - K*H)*P */
    double KH[4][4] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 2; ++k)
                KH[i][j] += K[i][k] * H[k][j];
    double IKH[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            IKH[i][j] = (i==j) - KH[i][j];
    double tmpP[4][4];
    mul44(IKH, P_, tmpP);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) P_[i][j] = tmpP[i][j];

    /* 3. 输出 */
    outX = x_[0];
    outY = x_[1];
    outVx = x_[2];
    outVy = x_[3];
}

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
        local_pos_pub.publish(pose); // 保持悬停
        rate.sleep();
        if (if_exit == true)
        {
            if ((target_pose.pose.position.z != -1 && mission_state != ADJUSTING) || (adjust_has_target && mission_state == ADJUSTING))
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
    int num_of_searching_points;
    nh.getParam("/num_of_searching_points",num_of_searching_points);
    for (int i = 0; i < num_of_searching_points; ++i)
    {
        float point[3];
        nh.getParam("/searching_point_" + std::to_string(i) + "_x", point[0]);
        nh.getParam("/searching_point_" + std::to_string(i) + "_y", point[1]);
        nh.getParam("/searching_point_" + std::to_string(i) + "_z", point[2]);
        searching_points.push_back(createPoint(point[0], point[1], point[2]));
    }

    int num_of_obstacle_zone_points;
    nh.getParam("/num_of_obstacle_zone_points",num_of_obstacle_zone_points);
    for (int i = 0; i < num_of_obstacle_zone_points; ++i)
    {
        float point[3];
        nh.getParam("/obstacle_zone_point_" + std::to_string(i) + "_x", point[0]);
        nh.getParam("/obstacle_zone_point_" + std::to_string(i) + "_y", point[1]);
        nh.getParam("/obstacle_zone_point_" + std::to_string(i) + "_z", point[2]);
        obstacle_zone_points.push_back(createPoint(point[0], point[1], point[2]));
    }

    // 新增功能：设置是否需要投弹
    for (int i = 0; i < 7; ++i)
    {
        bool isNeedForBomb;
        nh.getParam("/target_" + std::to_string(i),isNeedForBomb);
        targetArray[i].isNeedForBomb = isNeedForBomb;
    }
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
    const bool fresh = (now_sec - target_stamp_sec) < ros::Duration(p.lost_timeout);

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

// 加入一个 PoseStamped，并清理过期的
void addPose(const geometry_msgs::PoseStamped &pose, std::deque<geometry_msgs::PoseStamped> &g_pose_buffer, ros::Duration g_window_len)
{
    g_pose_buffer.push_back(pose);

    ros::Time now = pose.header.stamp;
    while (!g_pose_buffer.empty() && (now - g_pose_buffer.front().header.stamp) > g_window_len)
    {
        g_pose_buffer.pop_front();
    }
}

// 查找某个时间戳附近的 PoseStamped
bool getPoseAt(const ros::Time &t, geometry_msgs::PoseStamped &out, std::deque<geometry_msgs::PoseStamped> g_pose_buffer, ros::Duration g_window_len)
{
    if (g_pose_buffer.empty())
    {
        ROS_INFO("Buffer empty");
        return false;
    }
    // 时间不在缓存范围内
    if (t < g_pose_buffer.front().header.stamp || t > g_pose_buffer.back().header.stamp)
    {
        if (t < g_pose_buffer.front().header.stamp)
            ROS_INFO("Time too early");
        else
            ROS_INFO("Time too late");
        return false;
    }

    // 找最近邻
    for (size_t i = 0; i + 1 < g_pose_buffer.size(); i++)
    {
        const auto &a = g_pose_buffer[i];
        const auto &b = g_pose_buffer[i + 1];
        if (a.header.stamp <= t && t <= b.header.stamp)
        {
            if ((t - a.header.stamp) < (b.header.stamp - t))
                out = a;
            else
                out = b;
            return true;
        }
    }

    return false;
}
