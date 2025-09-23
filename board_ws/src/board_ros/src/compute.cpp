#include "compute.h"

namespace track
{

    // ==================== 通用工具 ====================
    static inline double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
    {
        return a.x() * b.y() - a.y() * b.x();
    }
    // RANSAC + (可选裁剪) + PCA 精修
    // 传入二维点集 pts（任选两维构成，比如 (t,x) / (t,y) / (x,y)），
    // 参数：iters、inlier_thresh、trim_ratio。
    // 返回：true 表示成功，输出 best_line（过整体均值）、inlier_idx（可选）。
    inline bool ransacLinePCA(const std::vector<Eigen::Vector2d> &pts,
                              int iters, double inlier_thresh, double trim_ratio,
                              Line2D &best_line, std::vector<int> *inlier_idx,
                              std::mt19937 *rng_ext)
    {
        if (pts.size() < 2)
            return false;

        // 预计算整体均值（RANSAC线过此点，数值稳定，和你之前一致）
        Eigen::Vector2d mean(0, 0);
        for (auto &p : pts)
            mean += p;
        mean /= double(pts.size());

        auto mkLine = [&](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
        {
            Line2D L;
            Eigen::Vector2d d = b - a;
            double n = d.norm();
            L.u = (n < 1e-12) ? Eigen::Vector2d(1, 0) : d / n;
            L.p0 = mean;
            return L;
        };
        auto pointLineAbsDist = [&](const Eigen::Vector2d &p, const Line2D &L)
        {
            return std::fabs(cross2(L.u, p - L.p0));
        };

        // RANSAC 选模型（最大内点数）
        std::mt19937 rng_local{0xC0FFEE};
        std::mt19937 &rng = rng_ext ? *rng_ext : rng_local;
        std::uniform_int_distribution<int> uni(0, int(pts.size()) - 1);

        int best_inliers = -1;
        Line2D pre;
        for (int it = 0; it < iters; ++it)
        {
            int i = uni(rng), j = uni(rng);
            if (i == j)
            {
                --it;
                continue;
            }
            Line2D L = mkLine(pts[i], pts[j]);
            int cnt = 0;
            for (auto &p : pts)
                if (pointLineAbsDist(p, L) <= inlier_thresh)
                    ++cnt;
            if (cnt > best_inliers)
            {
                best_inliers = cnt;
                pre = L;
            }
        }
        if (best_inliers <= 1)
            return false;

        // 收集内点
        std::vector<Eigen::Vector2d> inliers;
        inliers.reserve(best_inliers);
        std::vector<int> idx;
        idx.reserve(best_inliers);
        for (int k = 0; k < (int)pts.size(); ++k)
        {
            if (pointLineAbsDist(pts[k], pre) <= inlier_thresh)
            {
                inliers.push_back(pts[k]);
                idx.push_back(k);
            }
        }
        if (inliers.size() < 2)
            return false;

        // 可选裁剪（按残差从小到大，保留前 keep 个）
        if (inliers.size() >= 4 && trim_ratio > 0.0)
        {
            std::vector<std::tuple<double, Eigen::Vector2d, int>> lst;
            lst.reserve(inliers.size());
            for (size_t i = 0; i < inliers.size(); ++i)
            {
                double d = pointLineAbsDist(inliers[i], pre);
                lst.emplace_back(d, inliers[i], idx[i]);
            }
            std::sort(lst.begin(), lst.end(),
                      [](auto &a, auto &b)
                      { return std::get<0>(a) < std::get<0>(b); });
            size_t keep = std::max<size_t>(2, std::lround((1.0 - trim_ratio) * lst.size()));
            inliers.clear();
            idx.clear();
            for (size_t i = 0; i < keep; ++i)
            {
                inliers.push_back(std::get<1>(lst[i]));
                idx.push_back(std::get<2>(lst[i]));
            }
        }

        // PCA 精修方向（过内点均值）
        Eigen::Vector2d m(0, 0);
        for (auto &p : inliers)
            m += p;
        m /= double(inliers.size());

        Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
        for (auto &p : inliers)
        {
            Eigen::Vector2d d = p - m;
            C += d * d.transpose();
        }
        C /= double(inliers.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
        Eigen::Vector2d u = es.eigenvectors().col(1).normalized(); // 最大特征值对应方向

        best_line.p0 = m;
        best_line.u = u;
        if (inlier_idx)
            *inlier_idx = idx;
        return true;
    }

    // 根据二维直线（在 x 横轴、y 纵轴坐标系）求斜率 dy/dx
    inline bool lineSlope_dy_dx(const Line2D &L, double &slope)
    {
        if (std::fabs(L.u.x()) < 1e-12)
            return false; // 垂直线，无穷大
        slope = L.u.y() / L.u.x();
        return std::isfinite(slope);
    }

    // ==================== Learner：仅保留建模 + RANSAC 拟合速度 ====================

    Learner::Learner() { ep = Endpoints{}; }

    void Learner::reset()
    {
        buf.clear();
        ep = Endpoints{};
    }

    void Learner::feed(const geometry_msgs::PoseStamped &ps)
    {
        Obs o;
        o.t = ps.header.stamp;
        o.p << ps.pose.position.x, ps.pose.position.y;
        buf.push_back(o);

        // 维护时间窗
        const ros::Time t_now = ps.header.stamp;
        while (!buf.empty() && (t_now - buf.front().t).toSec() > prm.learn_window_sec)
            buf.pop_front();
    }

    // —— 用通用 RANSAC 做建模（x-y 平面）——
    bool Learner::compute()
    {
        if ((int)buf.size() < prm.min_points)
            return false;

        std::vector<Eigen::Vector2d> xy;
        xy.reserve(buf.size());
        for (auto &o : buf)
            xy.emplace_back(o.p.x(), o.p.y());

        Line2D line;
        std::vector<int> inliers;
        if (!ransacLinePCA(xy, prm.ransac_iters, prm.inlier_thresh_m, prm.refine_trim_ratio,
                           line, &inliers, &rng))
            return false;

        // 端点：沿主方向的极值
        double smin = std::numeric_limits<double>::infinity();
        double smax = -std::numeric_limits<double>::infinity();
        Eigen::Vector2d pmin, pmax;
        for (int idx : inliers)
        {
            const auto &p = xy[idx];
            double s = line.u.dot(p - line.p0);
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
            ROS_INFO_STREAM("[Learner] RANSAC inliers=" << inliers.size()
                                                        << " L=" << L);
        }
        return true;
    }
    // 从 buf 生成二维样本对：例如 (T,X)、(T,Y)、(X,Y)
    // 注意：时间使用相对时间（减去首帧），提升数值稳定性
    bool Learner::buildPairs(std::vector<Eigen::Vector2d> &out, Var a, Var b) const
    {
        out.clear();
        if (buf.size() < 2)
            return false;
        const double t0 = buf.front().t.toSec();
        out.reserve(buf.size());
        for (const auto &o : buf)
        {
            double vA = 0.0, vB = 0.0;
            switch (a)
            {
            case Var::T:
                vA = o.t.toSec() - t0;
                break;
            case Var::X:
                vA = o.p.x();
                break;
            case Var::Y:
                vA = o.p.y();
                break;
            }
            switch (b)
            {
            case Var::T:
                vB = o.t.toSec() - t0;
                break;
            case Var::X:
                vB = o.p.x();
                break;
            case Var::Y:
                vB = o.p.y();
                break;
            }
            out.emplace_back(vA, vB);
        }
        return true;
    }

    // ======== 新增：用 RANSAC 对 x(t)、y(t) 做直线拟合，返回斜率（速度） ========
    // 参数：
    //   thr_tx, thr_ty：RANSAC 内点阈值（单位与对应二维的刻度一致）。
    //                   例如 t-x 的阈值可设为 prm.inlier_thresh_m（对 x 方向）
    //                   或者根据你的 t 尺度调整（如果 t 的数值远大/远小，建议先归一化）
    // 返回：
    //   vx_slope = dx/dt （m/s），vy_slope = dy/dt
    bool Learner::fitLinearVelocityRANSAC(double &vx_slope, double &vy_slope,
                                          double thr_tx, double thr_ty,
                                          int iters, double trim_ratio)
    {
        if (buf.size() < 3)
            return false;

        // 1) x(t) —— 在 (t,x) 平面拟合直线，斜率为 dx/dt
        std::vector<Eigen::Vector2d> tx;
        if (!buildPairs(tx, Var::T, Var::X))
            return false;

        Line2D lx;
        if (!ransacLinePCA(tx, iters, thr_tx, trim_ratio, lx, nullptr, &rng))
            return false;

        double slope_tx = 0.0; // dx/dt = dy/dx in (t,x) with x as "y" and t as "x"
        {
            // 在 (横轴=t, 纵轴=x) 坐标系，直线方向为 (ut, ux)，斜率 = ux/ut
            if (std::fabs(lx.u.x()) < 1e-12)
                return false; // ut ~ 0，近似垂直，不可求
            slope_tx = lx.u.y() / lx.u.x();
        }

        // 2) y(t) —— 在 (t,y) 平面拟合直线，斜率为 dy/dt
        std::vector<Eigen::Vector2d> ty;
        if (!buildPairs(ty, Var::T, Var::Y))
            return false;

        Line2D ly;
        if (!ransacLinePCA(ty, iters, thr_ty, trim_ratio, ly, nullptr, &rng))
            return false;

        double slope_ty = 0.0; // dy/dt
        {
            if (std::fabs(ly.u.x()) < 1e-12)
                return false;
            slope_ty = ly.u.y() / ly.u.x();
        }

        vx_slope = slope_tx;
        vy_slope = slope_ty;
        return std::isfinite(vx_slope) && std::isfinite(vy_slope);
    }
};
