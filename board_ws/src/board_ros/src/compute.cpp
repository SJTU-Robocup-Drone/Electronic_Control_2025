#include "compute.h"
#include <deque>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <cmath>

// ===== 这里声明你工程里的全局 UAV 位姿 =====

namespace board_ros
{
    namespace track
    {

        // ===== 小工具 =====
        static inline double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
        {
            return a.x() * b.y() - a.y() * b.x();
        }

        // ===== 内部：学习阶段（RANSAC→PCA） =====
        struct Learner
        {
            LearnParams prm;
            std::mt19937 rng{0xC0FFEE};

            struct Obs
            {
                ros::Time t;
                Eigen::Vector2d p;
            };
            std::deque<Obs> buf;
            Endpoints ep;

            void reset()
            {
                buf.clear();
                ep = Endpoints{};
            }

            void feed(const geometry_msgs::PoseStamped &ps)
            {
                Obs o;
                o.t = ps.header.stamp;
                o.p << ps.pose.position.x, ps.pose.position.y;
                buf.push_back(o);
                const ros::Time t_now = ps.header.stamp;
                while (!buf.empty() && (t_now - buf.front().t).toSec() > prm.learn_window_sec)
                    buf.pop_front();
            }

            bool compute()
            {
                if (buf.size() < prm.min_points)
                    return false;

                // 收集滑窗点
                std::vector<Eigen::Vector2d> pts;
                pts.reserve(buf.size());
                for (auto &o : buf)
                    pts.push_back(o.p);

                // ==== RANSAC：过整体均值的方向线 ====
                struct Line
                {
                    Eigen::Vector2d p0, u;
                } best;
                int best_inliers = -1;

                auto meanOf = [&](const std::vector<Eigen::Vector2d> &v)
                {
                    Eigen::Vector2d m(0, 0);
                    for (auto &p : v)
                        m += p;
                    return m / double(v.size());
                };
                auto pointLineAbsDist = [&](const Eigen::Vector2d &p,
                                            const Eigen::Vector2d &p0,
                                            const Eigen::Vector2d &u)
                {
                    // 这里隐含用单位 u，值≈点到线的“面积距离”
                    return std::fabs(cross2(u, p - p0));
                };

                const Eigen::Vector2d mean = meanOf(pts);
                auto makeLine = [&](const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
                {
                    Line L;
                    Eigen::Vector2d d = p2 - p1;
                    double n = d.norm();
                    L.u = (n < 1e-9) ? Eigen::Vector2d(1, 0) : d / n;
                    L.p0 = mean;
                    return L;
                };

                std::uniform_int_distribution<int> uni(0, int(pts.size()) - 1);
                for (int it = 0; it < prm.ransac_iters; ++it)
                {
                    int i = uni(rng), j = uni(rng);
                    if (i == j)
                    {
                        --it;
                        continue;
                    }
                    Line L = makeLine(pts[i], pts[j]);
                    int cnt = 0;
                    for (auto &p : pts)
                        if (pointLineAbsDist(p, L.p0, L.u) <= prm.inlier_thresh_m)
                            ++cnt;
                    if (cnt > best_inliers)
                    {
                        best_inliers = cnt;
                        best = L;
                    }
                }
                if (best_inliers < int(0.5 * prm.min_points))
                    return false;

                // ==== 收集内点并可选裁剪 ====
                std::vector<Eigen::Vector2d> inliers;
                inliers.reserve(best_inliers);
                for (auto &p : pts)
                    if (pointLineAbsDist(p, best.p0, best.u) <= prm.inlier_thresh_m)
                        inliers.push_back(p);

                if ((int)inliers.size() < prm.min_points)
                    return false;

                if (inliers.size() >= 4 && prm.refine_trim_ratio > 0.0)
                {
                    std::vector<std::pair<double, Eigen::Vector2d>> dv;
                    dv.reserve(inliers.size());
                    for (auto &p : inliers)
                        dv.emplace_back(pointLineAbsDist(p, best.p0, best.u), p);
                    std::sort(dv.begin(), dv.end(), [](auto &a, auto &b)
                              { return a.first < b.first; });
                    size_t keep = std::max<size_t>(2, std::round((1.0 - prm.refine_trim_ratio) * dv.size()));
                    inliers.clear();
                    for (size_t i = 0; i < keep && i < dv.size(); ++i)
                        inliers.push_back(dv[i].second);
                }

                // ==== PCA 精修方向 ====
                Eigen::Vector2d m(0, 0);
                for (auto &p : inliers)
                    m += p;
                m /= double(inliers.size());

                Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
                for (auto &p : inliers)
                {
                    auto d = p - m;
                    C += d * d.transpose();
                }
                C /= double(inliers.size());

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
                Eigen::Vector2d u = es.eigenvectors().col(1).normalized();

                // ==== 端点：沿主方向的极值 ====
                double smin = std::numeric_limits<double>::infinity();
                double smax = -std::numeric_limits<double>::infinity();
                Eigen::Vector2d pmin, pmax;
                for (auto &p : inliers)
                {
                    double s = u.dot(p - m);
                    if (s < smin)
                    {
                        smin = s;
                        pmin = p;
                    }
                    if (s > smax)
                    {
                        smax = s;
                        pmax = p;
                    }
                }

                Eigen::Vector2d AB = pmax - pmin;
                double L = AB.norm();
                if (L < 1e-6)
                    return false;

                ep.A = pmin;
                ep.B = pmax;
                ep.u = AB / L;
                ep.L = L;
                ep.valid = true;

                if (prm.verbose_log)
                {
                    ROS_INFO_STREAM("[Learner] inliers=" << inliers.size() << " L=" << L);
                }
                return true;
            }
        };

        // ===== 内部：节拍建模 =====
        struct Cycler
        {
            CycleParams prm;
            Endpoints ep;

            struct State
            {
                double s{0}, v{0};
                ros::Time stamp;
                bool has{false};
            } st;

            std::vector<ros::Time> tA, tB; // 端点通过时间序列
            std::vector<double> vA2B, vB2A, turnA, turnB;
            double v_A2B{0.16}, v_B2A{0.16}, turn_A{0.0}, turn_B{0.0};
            char last_event = '?';
            ros::Time last_event_time;
            bool atA = false, atB = false;
            ros::Time enterA, enterB;

            void reset(const Endpoints &nep)
            {
                ep = nep;
                st = State{};
                tA.clear();
                tB.clear();
                vA2B.clear();
                vB2A.clear();
                turnA.clear();
                turnB.clear();
                last_event = '?';
            }

            static inline double projectS(const Eigen::Vector2d &P, const Endpoints &ep)
            {
                return ep.u.dot(P - ep.A); // 投影坐标 s：A 为 0，B 为 L
            }

            static inline double med(std::vector<double> v)
            {
                if (v.empty())
                    return 0.0;
                std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                return v[v.size() / 2];
            }

            template <typename T>
            void pushBounded(std::vector<T> &v, const T &x)
            {
                v.push_back(x);
                if ((int)v.size() > prm.max_history)
                    v.erase(v.begin());
            }

            void feed(const geometry_msgs::PoseStamped &ps)
            {
                if (!ep.valid)
                    return;

                Eigen::Vector2d P(ps.pose.position.x, ps.pose.position.y);
                double s_meas = projectS(P, ep);
                ros::Time t = ps.header.stamp;

                if (!st.has)
                {
                    st.s = s_meas;
                    st.v = 0;
                    st.stamp = t;
                    st.has = true;
                    return;
                }
                double dt = (t - st.stamp).toSec();
                if (dt <= 0)
                    dt = 1e-3;

                // α-β
                double s_pred = st.s + st.v * dt;
                double v_pred = st.v;
                double r = s_meas - s_pred;
                st.s = s_pred + prm.alpha * r;
                st.v = v_pred + (prm.beta / dt) * r;
                st.stamp = t;

                detect(t);
            }

            void detect(const ros::Time &t)
            {
                double s = st.s, v = st.v;
                bool nearA = std::fabs(s - 0.0) <= prm.endpoint_tol_s;
                bool nearB = std::fabs(s - ep.L) <= prm.endpoint_tol_s;
                bool slow = std::fabs(v) <= prm.low_speed_th;

                if (nearA && !atA)
                {
                    atA = true;
                    enterA = t;
                }
                if (!nearA && atA)
                {
                    atA = false;
                }
                if (nearB && !atB)
                {
                    atB = true;
                    enterB = t;
                }
                if (!nearB && atB)
                {
                    atB = false;
                }

                if (nearA && slow)
                    tryEmit('A', t, enterA);
                if (nearB && slow)
                    tryEmit('B', t, enterB);
            }

            void tryEmit(char where, const ros::Time &now, const ros::Time &enter)
            {
                if ((now - enter).toSec() < prm.min_turn_time)
                    return;
                if (last_event == where && (now - last_event_time).toSec() < prm.event_cooldown)
                    return;

                last_event = where;
                last_event_time = now;

                if (where == 'A')
                {
                    pushBounded(turnA, (now - enterA).toSec());
                    tA.push_back(now);
                    if ((int)tA.size() > prm.max_history)
                        tA.erase(tA.begin());
                }
                else
                {
                    pushBounded(turnB, (now - enterB).toSec());
                    tB.push_back(now);
                    if ((int)tB.size() > prm.max_history)
                        tB.erase(tB.begin());
                }

                // 速度分段（用端点对的时间差更新）
                int nAB = std::min(tA.size(), tB.size());
                if (nAB >= 1)
                {
                    double dt = (tB[nAB - 1] - tA[nAB - 1]).toSec();
                    if (dt > 0.2)
                        pushBounded(vA2B, ep.L / dt);
                }
                if (tB.size() >= 1 && tA.size() >= 2)
                {
                    double dt = (tA.back() - tB.back()).toSec();
                    if (dt > 0.2)
                        pushBounded(vB2A, ep.L / dt);
                }

                // 中位数统计
                v_A2B = med(vA2B);
                v_B2A = med(vB2A);
                turn_A = med(turnA);
                turn_B = med(turnB);

                if (prm.verbose_log)
                {
                    ROS_INFO("[Cycler] %c event: vA2B=%.3f vB2A=%.3f turnA=%.2f turnB=%.2f",
                             where, v_A2B, v_B2A, turn_A, turn_B);
                }
            }

            ros::Time predictPassTime(char endpoint, const ros::Time &now) const
            {
                double sE = (endpoint == 'A') ? 0.0 : ep.L;
                auto clamp_v = [](double v)
                { return std::max(0.02, std::fabs(v)); };

                bool heading = ((sE - st.s) * st.v) >= 0.0 && std::fabs(st.v) > 1e-3;
                if (heading)
                {
                    double vdir = (sE > st.s) ? v_A2B : v_B2A;
                    double dt = std::fabs(sE - st.s) / clamp_v(vdir);
                    return now + ros::Duration(dt);
                }
                else
                {
                    // 反向：到另一端、掉头、再回到该端
                    if (endpoint == 'A')
                    {
                        double dt1 = std::fabs(ep.L - st.s) / clamp_v(st.v >= 0 ? v_A2B : v_B2A);
                        double dt2 = ep.L / clamp_v(v_B2A);
                        return now + ros::Duration(dt1 + turn_B + dt2);
                    }
                    else
                    {
                        double dt1 = std::fabs(0.0 - st.s) / clamp_v(st.v >= 0 ? v_A2B : v_B2A);
                        double dt2 = ep.L / clamp_v(v_A2B);
                        return now + ros::Duration(dt1 + turn_A + dt2);
                    }
                }
            }
        };

        // ===== 内部：时窗计算小工具 =====
        static inline char classifyEndpoint(const Endpoints &ep, const Eigen::Vector2d &endpoint_pos)
        {
            return ((endpoint_pos - ep.A).norm() <= (endpoint_pos - ep.B).norm()) ? 'A' : 'B';
        }
        static inline double sigma_t_travel(double d, double v_mean, double sigma_v)
        {
            double v = std::max(0.02, std::fabs(v_mean));
            return (d / (v * v)) * std::fabs(sigma_v); // dt/dv 近似
        }
        static inline double rss2(double a, double b) { return std::sqrt(a * a + b * b); }
        static inline double rss3(double a, double b, double c) { return std::sqrt(a * a + b * b + c * c); }

        // ===== Pimpl 实现 =====
        struct TrackerDropper::Impl
        {
            Config cfg;
            Learner learner;
            Cycler cycler;
            Endpoints ep;
            bool model_ready{false};

            // —— 直解释放相关（速度估计）——
            DirectParams direct_prm;
            geometry_msgs::PoseStamped last_obs; // 最近一帧目标观测
            bool has_last_obs{false};

            // 沿直线投影 s 的历史，用于瞬时速度估计
            double prev_s{0.0};
            ros::Time prev_t;
            bool has_prev_s{false};

            // 平滑后的速度估计（沿 u 方向的投影速度）
            double v_s_est{0.0};
            bool v_s_has{false};

            Impl(const Config &c) : cfg(c)
            {
                learner.prm = cfg.learn;
                cycler.prm = cfg.cycle;
                direct_prm = cfg.direct;
            }

            void reset()
            {
                learner.reset();
                cycler.reset(Endpoints{});
                ep = Endpoints{};
                model_ready = false;

                has_last_obs = false;
                has_prev_s = false;
                v_s_has = false;
                v_s_est = 0.0;

                learner.prm = cfg.learn;
                cycler.prm = cfg.cycle;
                direct_prm = cfg.direct;
            }

            // ========== 核心：投喂观测 ==========
            void feed(const geometry_msgs::PoseStamped &ps)
            {
                // 先缓存原始观测（供直解使用）
                last_obs = ps;
                has_last_obs = true;

                // 学习器维护滑窗
                learner.feed(ps);

                // 条件满足则尝试更新轨迹模型
                bool ok = learner.compute();

                if (!model_ready)
                {
                    if (ok)
                    {
                        ep = learner.ep;
                        model_ready = ep.valid;
                        if (model_ready)
                        {
                            cycler.reset(ep);
                            // 轨迹刚建立：清空速度估计的历史
                            has_prev_s = false;
                            v_s_has = false;
                            v_s_est = 0.0;
                        }
                    }
                }
                else
                {
                    if (ok && learner.ep.valid)
                    {
                        // 如果端点或方向变化超过阈值，则刷新并重置节拍器
                        const double eps_pos = 0.03; // 3cm
                        const double eps_dir = 0.02; // ~1.1°
                        bool shift_big = ((ep.A - learner.ep.A).norm() > eps_pos) ||
                                         ((ep.B - learner.ep.B).norm() > eps_pos);
                        double cosang = std::max(-1.0, std::min(1.0, ep.u.dot(learner.ep.u)));
                        bool rot_big = std::acos(cosang) > eps_dir;
                        if (shift_big || rot_big)
                        {
                            ep = learner.ep;
                            cycler.reset(ep);
                            // 方向/端点变化大：重置直解速度历史
                            has_prev_s = false;
                            v_s_has = false;
                            v_s_est = 0.0;
                        }
                    }
                }

                // 节拍器推进
                if (model_ready)
                {
                    cycler.feed(ps);
                }

                // ===== 直解用：沿直线的瞬时速度估计（指数平滑）=====
                if (model_ready)
                {
                    Eigen::Vector2d P(ps.pose.position.x, ps.pose.position.y);
                    double s_meas = ep.u.dot(P - ep.A); // 当前投影坐标 s

                    if (!has_prev_s)
                    {
                        prev_s = s_meas;
                        prev_t = ps.header.stamp;
                        has_prev_s = true;
                    }
                    else
                    {
                        double dt = (ps.header.stamp - prev_t).toSec();
                        if (dt > 0.0)
                        {
                            double v_inst = (s_meas - prev_s) / dt; // 瞬时速度
                            double a = std::min(1.0, std::max(0.0, direct_prm.v_alpha));
                            if (!v_s_has)
                            {
                                v_s_est = v_inst;
                                v_s_has = true;
                            }
                            else
                            {
                                // 一阶 EMA：抑制速度突变
                                v_s_est = (1.0 - a) * v_s_est + a * v_inst;
                            }
                            prev_s = s_meas;
                            prev_t = ps.header.stamp;
                        }
                    }
                }
            }

            // ========== 节拍：预测下次通过 A/B ==========
            bool predictPassTimes(ros::Time now, ros::Time &tA, ros::Time &tB) const
            {
                if (!model_ready || !ep.valid)
                    return false;
                tA = cycler.predictPassTime('A', now);
                tB = cycler.predictPassTime('B', now);
                return true;
            }

            // ========== 节拍：计算投弹时窗 ==========
            bool computeReleaseWindow(const Eigen::Vector2d &endpoint_pos,
                                      ros::Time now,
                                      std::pair<ros::Time, ros::Time> &out,
                                      ros::Time *center) const
            {
                if (!model_ready || !ep.valid)
                    return false;

                const char E = classifyEndpoint(ep, endpoint_pos);
                const double sE = (E == 'A') ? 0.0 : ep.L;

                const auto t_pass = cycler.predictPassTime(E, now);

                // 不确定度（速度段/驻留）合成
                const bool heading = ((sE - cycler.st.s) * cycler.st.v) >= 0.0 && std::fabs(cycler.st.v) > 1e-3;
                double sigma_t = 0.0;
                if (heading)
                {
                    const double d = std::fabs(sE - cycler.st.s);
                    const bool dirA2B = (sE > cycler.st.s);
                    const double vbar = dirA2B ? cycler.v_A2B : cycler.v_B2A;
                    const double sv = dirA2B ? cfg.unc.sigma_v_A2B : cfg.unc.sigma_v_B2A;
                    sigma_t = sigma_t_travel(d, vbar, sv);
                }
                else
                {
                    const bool leg1_A2B = (cycler.st.v >= 0.0);
                    const double d1 = std::fabs((E == 'A' ? ep.L : 0.0) - cycler.st.s);
                    const double v1 = leg1_A2B ? cycler.v_A2B : cycler.v_B2A;
                    const double sv1 = leg1_A2B ? cfg.unc.sigma_v_A2B : cfg.unc.sigma_v_B2A;
                    const double s1 = sigma_t_travel(d1, v1, sv1);

                    const double sturn = (E == 'A') ? cfg.unc.sigma_turn_B : cfg.unc.sigma_turn_A;

                    const bool leg2_A2B = (E == 'B');
                    const double d2 = ep.L;
                    const double v2 = leg2_A2B ? cycler.v_A2B : cycler.v_B2A;
                    const double sv2 = leg2_A2B ? cfg.unc.sigma_v_A2B : cfg.unc.sigma_v_B2A;
                    const double s2 = sigma_t_travel(d2, v2, sv2);

                    sigma_t = rss3(s1, sturn, s2);
                }
                sigma_t = rss2(sigma_t, cfg.drop.base_jitter);

                // 时窗中心：通过端点时刻减去“提前量”
                const double advance = cfg.drop.T_drop + cfg.drop.sys_delay;
                ros::Time t_center = t_pass - ros::Duration(advance);
                const double half = cfg.drop.window_k * sigma_t;

                out = {t_center - ros::Duration(half), t_center + ros::Duration(half)};
                if (center)
                    *center = t_center;
                return true;
            }

            // ========== 直解释放：不依赖节拍 ==========
            // 基于“当前估计速度”与“UAV 当前位置”，求最佳释放时刻
            bool solveDirectRelease_impl(const ros::Time &now,
                                         ros::Time &t_release,
                                         double *miss_dist) const
            {
                if (!model_ready || !ep.valid || !has_last_obs || !v_s_has)
                    return false;

                // UAV xy（直接读取全局 current_pose）
                Eigen::Vector2d uav_xy(current_pose.pose.position.x,
                                       current_pose.pose.position.y);

                // 目标最近位置
                const ros::Time t0 = (last_obs.header.stamp.isZero() ? now : last_obs.header.stamp);
                const Eigen::Vector2d p0(last_obs.pose.position.x, last_obs.pose.position.y);

                // 估计的地面速度向量（沿直线）
                const Eigen::Vector2d v_vec = ep.u * v_s_est;
                const double vnorm = v_vec.norm();
                if (vnorm < 1e-3)
                    return false;

                // 可选触发半径：太远就不触发（避免提前太久）
                if (direct_prm.trigger_radius > 0.0)
                {
                    if ((uav_xy - p0).norm() > direct_prm.trigger_radius)
                        return false;
                }

                // 仅在目标“朝 UAV 方向”运动时考虑触发
                const Eigen::Vector2d dU = uav_xy - p0;
                if (dU.dot(v_vec) <= 0.0)
                    return false;

                // 释放→落地总提前量
                const double Ta = cfg.drop.T_drop + cfg.drop.sys_delay;

                // 目标线性模型：p(t0 + Δt + Ta) ≈ uav_xy
                // 最小二乘投影得到理想 Δt（从 t0 起算）
                double dt_from_t0 = dU.dot(v_vec) / std::max(1e-6, v_vec.squaredNorm()) - Ta;

                // 折算到 now：若 now 晚于 t0，需要再减去 (now - t0)
                double dt_now = dt_from_t0 - std::max(0.0, (now - t0).toSec());
                if (dt_now < 0.0)
                    dt_now = 0.0; // 不能为负
                if (dt_now > direct_prm.max_horizon)
                    return false; // 太远作废

                t_release = now + ros::Duration(dt_now);

                if (miss_dist)
                {
                    // 估计落地时刻目标位置 vs UAV 投影的偏差
                    Eigen::Vector2d p_land = p0 + v_vec * (dt_from_t0 + Ta);
                    *miss_dist = (p_land - uav_xy).norm();
                }
                return true;
            }
        };

        // ===== 对外类实现 =====
        TrackerDropper::TrackerDropper(const Config &cfg) : impl_(new Impl(cfg)) {}
        void TrackerDropper::reset() { impl_->reset(); }
        void TrackerDropper::feed(const geometry_msgs::PoseStamped &ps) { impl_->feed(ps); }
        bool TrackerDropper::hasModel() const { return impl_->model_ready && impl_->ep.valid; }
        Endpoints TrackerDropper::endpoints() const { return impl_->ep; }
        bool TrackerDropper::predictPassTimes(ros::Time now, ros::Time &tA, ros::Time &tB) const { return impl_->predictPassTimes(now, tA, tB); }
        bool TrackerDropper::computeReleaseWindow(const Eigen::Vector2d &endpoint_pos, ros::Time now, std::pair<ros::Time, ros::Time> &out, ros::Time *center) const
        {
            return impl_->computeReleaseWindow(endpoint_pos, now, out, center);
        }
        bool TrackerDropper::solveDirectRelease(const ros::Time &now, ros::Time &t_release, double *miss_dist) const
        {
            return impl_->solveDirectRelease_impl(now, t_release, miss_dist);
        }
        const Config &TrackerDropper::config() const { return impl_->cfg; }
        void TrackerDropper::setConfig(const Config &cfg)
        {
            impl_->cfg = cfg;
            impl_->learner.prm = cfg.learn;
            impl_->cycler.prm = cfg.cycle;
            impl_->direct_prm = cfg.direct;
        }

    } // namespace track
} // namespace board_ros
