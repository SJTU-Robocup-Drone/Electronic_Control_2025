#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <utility>


namespace board_ros
{
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

        // ===== 循环往返节拍参数 =====
        struct CycleParams
        {
            double alpha = 0.6;           // α-β滤波：位置增益 α
            double beta = 0.2;            // α-β滤波：速度增益 β
            double endpoint_tol_s = 0.08; // 端点邻域（投影坐标阈）
            double low_speed_th = 0.03;   // 掉头时“低速”阈（m/s）
            double min_turn_time = 0.10;  // 驻留最小时间（s）
            double event_cooldown = 0.25; // 事件冷却时间（s）
            int max_history = 20;         // 统计窗口长度
            bool verbose_log = false;
        };

        // ===== 投弹/延迟参数 =====
        struct DropParams
        {
            double T_drop{0.50};      // 释放→落地时间（s）
            double sys_delay{0.08};   // 控制/通信/机械等系统延迟（s）
            double base_jitter{0.02}; // 额外时间不确定性（s）
            double window_k{2.0};     // 时窗半宽 = k * σ
        };

        // ===== 不确定性（统计标准差）=====
        struct Uncertainty
        {
            double sigma_v_A2B{0.01};  // A→B 速度不确定性（m/s）
            double sigma_v_B2A{0.01};  // B→A 速度不确定性（m/s）
            double sigma_turn_A{0.05}; // A 端掉头驻留不确定性（s）
            double sigma_turn_B{0.05}; // B 端掉头驻留不确定性（s）
        };

        // ===== 直解（Direct Release）参数 =====
        // 用于“不建节拍器、直接用瞬时速度”解算释放时机
        struct DirectParams
        {
            double v_alpha{0.25};       // 速度指数平滑系数（0~1，越大越敏感）
            double trigger_radius{0.0}; // 触发半径（m），<=0 则不启用距离触发
            double max_horizon{5.0};    // 允许的最大提前量（s），太远的时刻作废
        };

        // ===== 总配置 =====
        struct Config
        {
            LearnParams learn;
            CycleParams cycle;
            DropParams drop;
            Uncertainty unc;
            DirectParams direct; // 新增
        };

        // ===== 主类 =====
        class TrackerDropper
        {
        public:
            explicit TrackerDropper(const Config &cfg = Config());
            void reset();

            // 投喂一帧观测（目标 PoseStamped）
            void feed(const geometry_msgs::PoseStamped &ps);

            // 学习好的轨迹信息
            bool hasModel() const;
            Endpoints endpoints() const;

            // 节拍器：预测下次经过 A/B 的时间
            bool predictPassTimes(ros::Time now, ros::Time &tA, ros::Time &tB) const;

            // 投弹时机（节拍模型）：给定端点坐标，返回一个可投弹的时间区间
            bool computeReleaseWindow(const Eigen::Vector2d &endpoint_pos,
                                      ros::Time now,
                                      std::pair<ros::Time, ros::Time> &out,
                                      ros::Time *center = nullptr) const;

            // 直解释放：当目标经过 UAV 正下方时投（内部取 current_pose）
            // 成功返回 t_release；可选返回命中误差估计（地面平面距离）
            bool solveDirectRelease(const ros::Time &now,
                                    ros::Time &t_release,
                                    double *miss_dist = nullptr) const;

            // 配置
            const Config &config() const;
            void setConfig(const Config &cfg);

        private:
            struct Impl;
            Impl *impl_{nullptr};
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

    
    } // namespace track
} // namespace board_ros
