#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include "offboard.h"
#include <deque>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <cmath>

namespace track
    {

        // ===== 直线端点与方向 =====
        struct Endpoints
        {
            Eigen::Vector2d A{0, 0}; // 端点 A
            Eigen::Vector2d B{0, 0}; // 端点 B
            Eigen::Vector2d u{1, 0}; // 单位方向向量 A->B
            double L{0.0};           // 端点间长度
            bool valid{false};
        };

        // ===== 学习阶段参数（RANSAC→PCA）=====
        struct LearnParams
        {
            double learn_window_sec = 30.0;  // 学习时间窗（秒）
            std::size_t min_points = 150;    // 拟合最少点数（越大越稳）
            int ransac_iters = 300;          // RANSAC 迭代次数
            double inlier_thresh_m = 1.5;    // 内点阈值（点到线“面积距离”）
            double refine_trim_ratio = 0.15; // 精修前剔除最差比例
            bool verbose_log = false;        // 调试日志
        };

        struct Line2D
        {
            Eigen::Vector2d p0{0, 0};
            Eigen::Vector2d u{1, 0}; // 单位方向
        };

        // 通用 RANSAC + PCA 拟合二维直线
        bool ransacLinePCA(const std::vector<Eigen::Vector2d> &pts,
                           int iters,
                           double inlier_thresh,
                           double trim_ratio,
                           Line2D &best_line,
                           std::vector<int> *inlier_idx = nullptr,
                           std::mt19937 *rng_ext = nullptr);

        // 根据直线方向求斜率 dy/dx
        bool lineSlope_dy_dx(const Line2D &L, double &slope);

        // ==================== Learner ====================
        // 目标运动建模器（直线 + 速度估计）
        struct Learner
        {
            // 变量枚举（用于生成二维对）
            enum class Var
            {
                T,
                X,
                Y
            };

            // 构造与复位
            Learner();
            void reset();

            // 喂入一帧观测
            void feed(const geometry_msgs::PoseStamped &ps);

            // 用 RANSAC 对 x–y 平面建模，得到端点/方向
            bool compute();

            // 从缓存点生成 (a,b) 对，例如 (T,X)、(T,Y)、(X,Y)
            bool buildPairs(std::vector<Eigen::Vector2d> &out, Var a, Var b) const;

            // 用 RANSAC 分别拟合 x(t)、y(t)，返回斜率（速度分量）
            bool fitLinearVelocityRANSAC(double &vx_slope,
                                         double &vy_slope,
                                         double thr_tx,
                                         double thr_ty,
                                         int iters = 200,
                                         double trim_ratio = 0.2);

            // 成员
            LearnParams prm; // 参数
            Endpoints ep;    // 建模结果

        private:
            struct Obs
            {
                ros::Time t;
                Eigen::Vector2d p;
            };
            std::deque<Obs> buf;
            std::mt19937 rng{0xC0FFEE};
        };


        // 发布端点及方向
        inline void publish_endpoints(const Endpoints &ep,
                                      const geometry_msgs::PoseStamped &ref_pose,
                                      bool publish_direction = true,
                                      const std::string &endpoints_topic = "/track/endpoints_posearray",
                                      const std::string &direction_topic = "/track/endpoints_dir")
        {
            static ros::Publisher pub_endpoints;
            static ros::Publisher pub_direction;
            static std::string last_endpoints_topic;
            static std::string last_direction_topic;
            static bool inited = false;

            if (!inited || endpoints_topic != last_endpoints_topic || direction_topic != last_direction_topic)
            {
                ros::NodeHandle nh;
                pub_endpoints = nh.advertise<geometry_msgs::PoseArray>(endpoints_topic, 1, /*latch=*/true);
                pub_direction = nh.advertise<geometry_msgs::Vector3Stamped>(direction_topic, 1, /*latch=*/true);
                last_endpoints_topic = endpoints_topic;
                last_direction_topic = direction_topic;
                inited = true;
            }

            // 发布端点
            geometry_msgs::PoseArray pa;
            pa.header = ref_pose.header; // 带上 frame_id 和时间
            pa.poses.resize(2);
            pa.poses[0].position.x = ep.A.x();
            pa.poses[0].position.y = ep.A.y();
            pa.poses[1].position.x = ep.B.x();
            pa.poses[1].position.y = ep.B.y();
            pub_endpoints.publish(pa);

            // 发布方向（如果需要）
            if (publish_direction)
            {
                geometry_msgs::Vector3Stamped vs;
                vs.header = ref_pose.header;
                vs.vector.x = ep.u.x();
                vs.vector.y = ep.u.y();
                vs.vector.z = 0.0;
                pub_direction.publish(vs);
            }
        }

    } // namespace track
