#pragma once
// compute.h — 统一模块对外接口
// 功能：
//  1) 高空学习（直线拟合，得到端点 A/B、方向 u、长度 L）
//  2) 往返节拍建模（α-β 滤波、端点事件、速度/驻留中位数、预测下一次经过 A/B 的时间）
//  3) 投弹时机解算（输出在端点处触发投弹的可行时间区间）
// 用法：
//  1) 构造 TrackerDropper（可传参配置）
//  2) 每帧调用 feed(PoseStamped)
//  3) hasModel() 为真后，可读取 endpoints()、predictPassTimes()、computeReleaseWindow()

#include <utility>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace board_ros
{
    namespace track
    {

        struct Endpoints
        {
            Eigen::Vector2d A{0, 0};
            Eigen::Vector2d B{0, 0};
            Eigen::Vector2d u{1, 0};
            double L{0.0};
            bool valid{false};
        };

        struct LearnParams
        {
            double learn_window_sec = 30.0;  // 学习时间窗（秒）
            std::size_t min_points = 150;    // 拟合所需最少点数
            int ransac_iters = 300;          // RANSAC 迭代次数
            double inlier_thresh_m = 1.5;    // 内点判定阈值（m）
            double refine_trim_ratio = 0.15; // 精修前丢掉最差多少比例的点
            bool verbose_log = false;        // 是否打印调试日志
        };

        struct CycleParams
        {
            double alpha = 0.6;           // α-β α
            double beta = 0.2;            // α-β β
            double endpoint_tol_s = 0.08; // 端点邻域（m）
            double low_speed_th = 0.03;   // 掉头低速阈（m/s）
            double min_turn_time = 0.10;  // 驻留最短时间（s）
            double event_cooldown = 0.25; // 事件冷却（s）
            int max_history = 20;         // 统计窗口
            bool verbose_log = false;
        };

        struct DropParams
        {
            double T_drop{0.50};      // 释放→落地时间（s）
            double sys_delay{0.08};   // 系统总延迟（s）
            double base_jitter{0.02}; // 基础抖动（s）
            double window_k{2.0};     // 时窗 = center ± k*σ
        };

        struct Uncertainty
        {
            double sigma_v_A2B{0.01}; // m/s
            double sigma_v_B2A{0.01};
            double sigma_turn_A{0.05}; // s
            double sigma_turn_B{0.05};
        };

        struct Config
        {
            LearnParams learn;
            CycleParams cycle;
            DropParams drop;
            Uncertainty unc;
        };

        class TrackerDropper
        {
        public:
            explicit TrackerDropper(const Config &cfg = Config());
            void reset();

            // 喂入实时小车位置（世界系）
            void feed(const geometry_msgs::PoseStamped &ps);

            // 是否已有有效模型
            bool hasModel() const;

            // 端点结果（A/B/u/L/valid）
            Endpoints endpoints() const;

            // 预测：下一次经过 A/B 的绝对时间戳（返回 false 表示模型未就绪）
            bool predictPassTimes(ros::Time now, ros::Time &tA, ros::Time &tB) const;

            // 计算当前所在端点（传该端点世界坐标）下的投弹时窗 [t_min, t_max]；可返回中心时刻
            bool computeReleaseWindow(const Eigen::Vector2d &endpoint_pos,
                                      ros::Time now,
                                      std::pair<ros::Time, ros::Time> &window_out,
                                      ros::Time *t_center_out = nullptr) const;

            // 访问/修改配置
            const Config &config() const;
            void setConfig(const Config &cfg);

        private:
            struct Impl; // pimpl 隐藏实现
            Impl *impl_;
        };

        // 发布端点：A=poses[0], B=poses[1]
        inline void publish_endpoints_posearray(const Endpoints &ep,
                                                const geometry_msgs::PoseStamped &ref_pose,
                                                const std::string &topic = "/track/endpoints_posearray")
        {
            static ros::Publisher pub;
            static std::string last_topic;
            static bool inited = false;
            if (!inited || topic != last_topic)
            {
                ros::NodeHandle nh;
                pub = nh.advertise<geometry_msgs::PoseArray>(topic, 1, /*latch=*/true);
                last_topic = topic;
                inited = true;
            }

            geometry_msgs::PoseArray pa;
            pa.header = ref_pose.header; // 带上 frame_id 和时间
            pa.poses.resize(2);
            pa.poses[0].position.x = ep.A.x();
            pa.poses[0].position.y = ep.A.y();
            pa.poses[1].position.x = ep.B.x();
            pa.poses[1].position.y = ep.B.y();
            pub.publish(pa);
        }

        // 发布方向：单位向量 u（A->B）
        inline void publish_direction_u(const Endpoints &ep,
                                        const geometry_msgs::PoseStamped &ref_pose,
                                        const std::string &topic = "/track/endpoints_dir")
        {
            static ros::Publisher pub;
            static std::string last_topic;
            static bool inited = false;
            if (!inited || topic != last_topic)
            {
                ros::NodeHandle nh;
                pub = nh.advertise<geometry_msgs::Vector3Stamped>(topic, 1, /*latch=*/true);
                last_topic = topic;
                inited = true;
            }

            geometry_msgs::Vector3Stamped vs;
            vs.header = ref_pose.header;
            vs.vector.x = ep.u.x();
            vs.vector.y = ep.u.y();
            vs.vector.z = 0.0;
            pub.publish(vs);
        }

    }
} // namespace board_ros::track
