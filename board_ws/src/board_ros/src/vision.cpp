#include "offboard/vision.h"
#include <algorithm>
#include <cmath>

// 定义视觉缓存
std::deque<TimedPose> history_;
const size_t MAX_HISTORY = 5;

void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose = *msg;
    if (target_index < 3 && target_pose.pose.position.z != -1)
    {
        ROS_INFO_THROTTLE(2.0, "Target found at (%.2f, %.2f, %.2f)",
                          target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        if ((mission_state == ADJUSTING && !is_return) || mission_state == FOLLOW)
        {
            target_pose.pose.position.x += offset[target_index][0];
            target_pose.pose.position.y += offset[target_index][1];
        }
    }
    if (target_pose.pose.position.z == 2.0)
        is_moving_target = true;

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
