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

        // ===== 学习阶段参数（Learner） =====
        struct LearnParams
        {
            double learn_window_sec = 30.0;
            // 学习时间窗（秒）。滑动缓冲区长度。
            // 表示最多保留多少秒内的小车位置点用于轨迹拟合。
            // 越大：能覆盖更长的往返轨迹，但响应慢；越小：响应快但轨迹长度可能不完整。

            std::size_t min_points = 150;
            // 拟合所需的最少点数。
            // 少于这个点数，Learner 不会尝试拟合直线。
            // 保证拟合结果的稳定性。

            int ransac_iters = 300;
            // RANSAC 随机采样迭代次数。
            // 越多，直线拟合的鲁棒性越强，但计算开销增加。

            double inlier_thresh_m = 1.5;
            // 内点判定阈值（米）。
            // 判定某个点到拟合直线的垂直距离小于该值，才认为是“内点”。
            // 越小：直线拟合更“紧”，但可能丢掉端点处抖动的点；
            // 越大：能容忍更多噪声点，但可能把偏离较远的点也算进去。

            double refine_trim_ratio = 0.15;
            // 精修前丢掉最差的点的比例。
            // 比如 0.15 表示丢掉 15% 离直线最远的点，再做 PCA 精修。
            // 调大：抗噪声强，但可能缩短线段；
            // 调小或 0.0：保留所有点，端点更远，但容易受噪声影响。

            bool verbose_log = false;
            // 是否打印 Learner 阶段的调试日志（内点数、拟合长度等）。
        };

        // ===== 节拍建模参数（Cycler） =====
        struct CycleParams
        {
            double alpha = 0.6;
            // α-β 滤波器中的 α，控制位置修正的比重。
            // 越大：s 的观测更新更快，对噪声敏感；
            // 越小：更平滑，但反应慢。

            double beta = 0.2;
            // α-β 滤波器中的 β，控制速度修正的比重。
            // 越大：速度响应快，对噪声敏感；
            // 越小：速度估计更稳，但滞后大。

            double endpoint_tol_s = 0.08;
            // 端点邻域（米）。
            // 判断是否到达 A/B 端点的判定范围（投影坐标 s 与端点 0/L 的差距小于这个值）。
            // 太小：可能漏判端点；太大：容易误判。

            double low_speed_th = 0.03;
            // 掉头低速阈（米/秒）。
            // 速度低于这个值，认为小车正在掉头/驻留。
            // 太小：掉头不被识别；太大：平常慢速也会误判为掉头。

            double min_turn_time = 0.05;
            // 驻留最短时间（秒）。
            // 到达端点必须至少停留这么久，才算一次有效掉头事件。
            // 防止瞬间抖动多次触发。

            double event_cooldown = 0.25;
            // 事件冷却时间（秒）。
            // 两次相同端点事件之间必须间隔至少这么久。
            // 防止端点附近的抖动多次触发。

            int max_history = 20;
            // 统计窗口长度。
            // v_A2B、v_B2A、turn_A、turn_B 等历史样本最多存多少个，用中位数去估计。
            // 越大：统计更稳健，但收敛慢；越小：收敛快但容易跳动。

            bool verbose_log = false;
            // 是否打印 Cycler 阶段的调试日志（速度估计、掉头时间等）。
        };

        // ===== 投弹时机解算参数（Dropper） =====
        struct DropParams
        {
            double T_drop{0.50};
            // 炸弹从释放到落地的时间（秒）。
            // 需要由实验或物理模型标定。直接决定投弹提前量。

            double sys_delay{0};
            // 系统总延迟（秒）。
            // 包括视觉到控制、飞控处理、舵机动作延迟等。
            // 会作为投弹的额外提前量。

            double base_jitter{0.02};
            // 基础抖动（秒）。
            // 用于给投弹窗口计算增加一个不可避免的不确定性基准。

            double window_k{2.0};
            // 投弹时窗 = 中心时刻 ± k * σ。
            // k 越大：时窗更宽，成功概率更高，但精度差；
            // k 越小：时窗更窄，精度高，但容错差。
        };

        // ===== 不确定性模型参数 =====
        struct Uncertainty
        {
            double sigma_v_A2B{0.01};
            // A→B 运动段速度估计的不确定性（标准差，米/秒）。
            // 越大：窗口 σ 更大 → 投弹时窗更宽。

            double sigma_v_B2A{0.01};
            // B→A 段速度估计不确定性。

            double sigma_turn_A{0.05};
            // 在端点 A 掉头驻留时间的不确定性（秒）。

            double sigma_turn_B{0.05};
            // 在端点 B 掉头驻留时间的不确定性（秒）。
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
