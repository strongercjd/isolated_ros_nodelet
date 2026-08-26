#include "base_navi_nodelet.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{

double normalizeAngle(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

double clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

double yawFromQuat(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

} // namespace

namespace base_navi_nodelet
{

BaseNaviNodelet::~BaseNaviNodelet()
{
    stop_ = true;
    cv_.notify_all();
    if (worker_.joinable())
        worker_.join();
    publishZeroVel(); // 尽力停住仿真小车（保持式 cmd_vel）
}

void BaseNaviNodelet::onInit()
{
    ros::NodeHandle pnh = getPrivateNodeHandle();

    double control_hz = 10.0;
    pnh.param("control_hz", control_hz, 10.0);
    control_period_ = control_hz > 0.1 ? 1.0 / control_hz : 0.1;
    pnh.param("v_max", v_max_, 0.25);
    pnh.param("w_max", w_max_, 0.8);
    pnh.param("stop_count_limit", stop_count_limit_, 10);
    pnh.param("occ_free_th", frontier_params_.occ_free_th, 25);
    pnh.param("occ_occ_th", frontier_params_.occ_occ_th, 65);
    pnh.param("robot_radius", robot_radius_, 0.15);
    pnh.param("goal_tolerance", goal_tolerance_, 0.25);
    pnh.param("stuck_target_time_s", stuck_target_time_s_, 30.0);
    pnh.param("state_period", state_period_, 1.0);
    // 通行性窗口：覆盖机器人直径（至少 5x5）
    const int pass_half = std::max(2, static_cast<int>(std::ceil(robot_radius_ / 0.05)));
    frontier_params_.pass_half_window = pass_half;

    // 话题走 remap（plugins.json: map→/slam2d/map, pose→/slam2d/pose,
    //   scan→/mycar/base_scan, odom→/mycar/odom, cmd_vel→/mycar/cmd_vel）
    map_sub_ = pnh.subscribe("map", 1, &BaseNaviNodelet::mapCallback, this);
    pose_sub_ = pnh.subscribe("pose", 5, &BaseNaviNodelet::poseCallback, this);
    scan_sub_ = pnh.subscribe("scan", 5, &BaseNaviNodelet::scanCallback, this);
    odom_sub_ = pnh.subscribe("odom", 10, &BaseNaviNodelet::odomCallback, this);
    cmd_sub_ = pnh.subscribe("cmd", 5, &BaseNaviNodelet::cmdCallback, this);

    vel_pub_ = pnh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    state_pub_ = pnh.advertise<custom_msgs::NaviState>("state", 5);
    goal_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("goal", 5);

    worker_ = std::thread(&BaseNaviNodelet::workerLoop, this);
    NODELET_INFO("BaseNaviNodelet initialized: hz=%.0f v_max=%.2f w_max=%.2f stop_limit=%d "
                 "occ_th=(%d,%d) pass_half=%d tol=%.2fm",
                 1.0 / control_period_, v_max_, w_max_, stop_count_limit_,
                 frontier_params_.occ_free_th, frontier_params_.occ_occ_th,
                 frontier_params_.pass_half_window, goal_tolerance_);
}

// ---- 回调：只拷贝快照（manager spin 线程）----

void BaseNaviNodelet::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_map_ = *msg; // 4MB 拷贝 ≈ 1-2ms @1Hz
        map_new_ = true;
        ++map_count_;
    }
    cv_.notify_one();
}

void BaseNaviNodelet::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    pose_.x = msg->pose.position.x;
    pose_.y = msg->pose.position.y;
    pose_.yaw = yawFromQuat(msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z, msg->pose.orientation.w);
    pose_stamp_ = msg->header.stamp;
}

void BaseNaviNodelet::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    latest_scan_ = *msg;
    have_scan_ = true;
}

void BaseNaviNodelet::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;
    odom_pose_.yaw = yawFromQuat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void BaseNaviNodelet::cmdCallback(const custom_msgs::NaviCmd::ConstPtr &msg)
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd_queue_.emplace_back(msg->cmd, msg->seq);
    }
    cv_.notify_one();
}

// ---- worker：10Hz 控制环 ----

void BaseNaviNodelet::workerLoop()
{
    const auto period = std::chrono::duration<double>(control_period_);
    while (!stop_)
    {
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, period, [this] { return stop_.load() || !cmd_queue_.empty(); });
        }
        if (stop_)
            break;
        try
        {
            controlOnce();
        }
        catch (const std::exception &e)
        {
            NODELET_ERROR("control loop exception: %s", e.what());
        }
    }
}

void BaseNaviNodelet::controlOnce()
{
    const ros::Time now = ros::Time::now();

    // 1. 命令队列
    std::deque<std::pair<uint8_t, uint32_t>> cmds;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmds.swap(cmd_queue_);
    }
    for (const auto &c : cmds)
        handleCmd(c.first, c.second);

    if (state_ == NaviRunState::IDLE)
        return; // 未在任务中：不发布速度/状态

    // 2. 快照
    nav_msgs::OccupancyGrid map;
    bool map_new = false;
    Pose2d pose, odom_pose;
    ros::Time pose_stamp;
    bool have_pose = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (map_new_)
        {
            std::swap(map, latest_map_); // O(1)，回调侧下次重新赋值
            map_new_ = false;
            map_new = true;
        }
        pose = pose_;
        odom_pose = odom_pose_;
        pose_stamp = pose_stamp_;
        have_pose = !pose_stamp.isZero();
    }

    // 位姿降级：slam2d 位姿 >1s 未更新则退回 odom（SLAM 修正后 odom 系会偏，仅作兜底）
    static ros::Time last_pose_warn;
    if (have_pose && (now - pose_stamp).toSec() > 1.0)
    {
        pose = odom_pose;
        if ((now - last_pose_warn).toSec() > 10.0)
        {
            NODELET_WARN("slam pose stale (%.1fs), falling back to odom", (now - pose_stamp).toSec());
            last_pose_warn = now;
        }
    }

    // 3. 新地图：frontier 检测 + 目标维护 + 无目标计数（只随地图累计，1Hz）
    if (map_new)
    {
        ++map_seen_;
        explored_area_m2_ = freeAreaM2(map, frontier_params_.occ_free_th);
        if (state_ == NaviRunState::RUNNING)
        {
            std::vector<FrontierGoal> goals = detectFrontiers(map, frontier_params_);
            last_frontier_count_ = static_cast<uint32_t>(goals.size());
            maintainTarget(map, goals, now);
            if (has_target_)
                no_target_count_ = 0;
            else
                // 对齐 explorestatestopcount：连续选不出可达目标即累计。
                // 候选非空但全被拉黑同样属于"无可达目标"（否则机器人零速悬停到 watchdog）
                ++no_target_count_;
        }
    }

    // 4. 运动控制
    if (state_ == NaviRunState::RUNNING)
    {
        if (!have_pose)
        {
            publishZeroVel(); // 无位姿先不动
        }
        else if (map_seen_ == 0)
        {
            publishZeroVel(); // 开局等待地图（对齐原 WAITING 语义），不计失败
            no_target_count_ = 0;
        }
        else if (has_target_)
        {
            no_target_count_ = 0;
            const double dist = std::hypot(target_x_ - pose.x, target_y_ - pose.y);
            if (dist < goal_tolerance_)
            {
                NODELET_INFO("target reached (%.2f, %.2f)", target_x_, target_y_);
                has_target_ = false;
                publishZeroVel();
            }
            else
            {
                double v = 0.0, w = 0.0;
                computeVelocity(pose, v, w);

                // 顶死检测：持有目标且未到达，3s 内 odom 位移 < 1cm 即脱困。
                // 不看 v 指令——避障硬停（v=0）时的贴墙摆动同样属于顶死。
                if (stuck_check_time_.isZero())
                {
                    stuck_check_time_ = now;
                    stuck_check_pose_ = odom_pose;
                }
                else if ((now - stuck_check_time_).toSec() > 3.0)
                {
                    const double moved = std::hypot(odom_pose.x - stuck_check_pose_.x,
                                                    odom_pose.y - stuck_check_pose_.y);
                    if (moved < 0.01)
                    {
                        ++stuck_count_;
                        escape_until_ = now + ros::Duration(1.2); // 倒退+转向脱困
                        hard_avoid_dir_ = 0;                       // 脱困后重判避障方向
                        NODELET_WARN("stuck #%d (moved %.3fm in 3s), escaping", stuck_count_, moved);
                        if (stuck_count_ >= 3 && has_target_)
                        {
                            blacklistTarget(target_x_, target_y_);
                            has_target_ = false;
                            stuck_count_ = 0;
                        }
                    }
                    stuck_check_time_ = now;
                    stuck_check_pose_ = odom_pose;
                }

                publishVelocity(v, w);
            }
        }
        else
        {
            // 无目标：等待下一张地图重选；连续无可达候选到限即 FINISH
            if (no_target_count_ > static_cast<uint32_t>(stop_count_limit_))
            {
                state_ = NaviRunState::FINISHED;
                publishZeroVel();
                publishZeroVel(); // 保持式 cmd_vel，双发兜底
                NODELET_INFO("exploration FINISH: no reachable frontier for %u rounds, area=%.1f m2, "
                             "cost=%.0f s",
                             no_target_count_, explored_area_m2_, (now - start_time_).toSec());
                publishState(true);
                return;
            }
            publishZeroVel();
        }
    }
    else if (state_ == NaviRunState::PAUSED)
    {
        publishZeroVel();
    }

    // 5. 状态心跳
    publishState(false);
}

void BaseNaviNodelet::handleCmd(uint8_t cmd, uint32_t seq)
{
    switch (cmd)
    {
    case custom_msgs::NaviCmd::CMD_START:
        if (state_ == NaviRunState::RUNNING)
        {
            NODELET_WARN("START(seq=%u) ignored: already running", seq);
            return;
        }
        state_ = NaviRunState::RUNNING;
        has_target_ = false;
        no_target_count_ = 0;
        last_frontier_count_ = 0;
        explored_area_m2_ = 0.0;
        target_first_pick_ = ros::Time();
        target_picks_ = 0;
        blacklist_.clear();
        stuck_count_ = 0;
        stuck_check_time_ = ros::Time();
        escape_until_ = ros::Time();
        hard_avoid_dir_ = 0;
        start_time_ = ros::Time::now();
        NODELET_INFO("exploration START (seq=%u)", seq);
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_PAUSE:
        if (state_ != NaviRunState::RUNNING)
            return;
        state_ = NaviRunState::PAUSED;
        publishZeroVel();
        NODELET_INFO("exploration PAUSE (seq=%u)", seq);
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_RESUME:
        if (state_ != NaviRunState::PAUSED)
            return;
        state_ = NaviRunState::RUNNING;
        NODELET_INFO("exploration RESUME (seq=%u)", seq);
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_STOP:
        state_ = NaviRunState::IDLE;
        has_target_ = false;
        publishZeroVel();
        publishZeroVel();
        NODELET_INFO("exploration STOP (seq=%u), robot parked in place", seq);
        return;
    default:
        NODELET_WARN("unknown NaviCmd %u (seq=%u)", cmd, seq);
        return;
    }
}

// 目标维护：失效判定 / 超时拉黑 / 重选（距离最近优先，同距簇大优先，直线可达优先）
void BaseNaviNodelet::maintainTarget(const nav_msgs::OccupancyGrid &map,
                                     const std::vector<FrontierGoal> &goals, const ros::Time &now)
{
    if (has_target_)
    {
        // 超时拉黑：持有同一目标长时间未到达即拉黑（"看得见够不着"防御）。
        // 不依赖重选次数：目标一直持有时 picks 恒为 1，若以 picks 为条件将永不触发。
        if ((now - target_first_pick_).toSec() > stuck_target_time_s_)
        {
            NODELET_WARN("target (%.2f, %.2f) unreachable for %.0fs (picks=%d), blacklisted",
                         target_x_, target_y_, (now - target_first_pick_).toSec(), target_picks_);
            blacklistTarget(target_x_, target_y_);
            has_target_ = false;
            return;
        }
        // 失效判定：候选簇里已找不到 1m 内的对应簇（簇消失/被并入占用区）
        double nearest = 1e9;
        for (const auto &g : goals)
            nearest = std::min(nearest, std::hypot(g.wx - target_x_, g.wy - target_y_));
        if (nearest > 1.0)
        {
            NODELET_INFO("target (%.2f, %.2f) vanished from frontier set", target_x_, target_y_);
            has_target_ = false;
        }
        return; // 目标仍有效：保持方向惯性，不因新图换目标
    }

    // 重选：剔除拉黑区，最近优先；直线穿墙的候选重罚（+8m 虚拟距离），
    // 避免选中墙对面目标后纯跟踪控制律一路顶死。全候选被挡时惩罚均摊，退化为旧行为
    Pose2d pose;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        pose = pose_;
    }
    const FrontierGoal *best = nullptr;
    double best_key = 1e9;
    bool best_direct = false;
    for (const auto &g : goals)
    {
        if (isBlacklisted(g.wx, g.wy))
            continue;
        const double dist = std::hypot(g.wx - pose.x, g.wy - pose.y);
        const bool direct = lineReachable(map, pose.x, pose.y, g.wx, g.wy, frontier_params_);
        double key = dist - g.size * 0.001; // 距离为主，簇大小做次级偏好
        if (!direct)
            key += 8.0;
        if (key < best_key)
        {
            best_key = key;
            best = &g;
            best_direct = direct;
        }
    }
    if (!best)
        return;

    const bool same = !target_first_pick_.isZero() &&
                      std::hypot(best->wx - target_x_, best->wy - target_y_) < 1.0;
    if (!same)
    {
        target_first_pick_ = now;
        target_picks_ = 0;
    }
    target_x_ = best->wx;
    target_y_ = best->wy;
    ++target_picks_;
    has_target_ = true;
    NODELET_INFO("new target (%.2f, %.2f) size=%zu dist=%.1fm picks=%d via=%s",
                 target_x_, target_y_, best->size,
                 std::hypot(best->wx - pose.x, best->wy - pose.y), target_picks_,
                 best_direct ? "line" : "detour");
    publishGoal();
}

// 控制律：比例跟踪 + 扫描避障 + 脱困覆盖
void BaseNaviNodelet::computeVelocity(const Pose2d &pose, double &v, double &w)
{
    const ros::Time now = ros::Time::now();

    // 脱困动作优先（顶死后倒退+转向）
    if (!escape_until_.isZero() && now < escape_until_)
    {
        v = -0.15;
        w = 0.6;
        diagCtrl("escape", 0.0, -1.0, v, w);
        return;
    }

    const double e = normalizeAngle(std::atan2(target_y_ - pose.y, target_x_ - pose.x) - pose.yaw);
    const bool spinning = std::fabs(e) > 0.5;
    if (spinning)
    {
        v = 0.0;                                  // 原地转向
        w = clamp(1.5 * e, -w_max_, w_max_);
    }
    else
    {
        v = v_max_ * std::max(0.0, 1.0 - 1.2 * std::fabs(e)); // 弧线前进
        w = clamp(2.0 * e, -w_max_, w_max_);
    }

    // 扫描避障：前向 ±60° 锥内最近距离
    sensor_msgs::LaserScan scan;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!have_scan_)
        {
            diagCtrl("no-scan", e, -1.0, v, w);
            return;
        }
        scan = latest_scan_;
    }
    const double half_cone = M_PI / 3.0;
    double nearest = 1e9, left_sum = 0.0, right_sum = 0.0;
    int left_n = 0, right_n = 0;
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double ang = scan.angle_min + i * scan.angle_increment;
        if (std::fabs(ang) > half_cone)
            continue;
        const float r = scan.ranges[i];
        if (!(r >= scan.range_min && r <= scan.range_max) || !std::isfinite(r))
            continue;
        nearest = std::min(nearest, static_cast<double>(r));
        if (ang < 0)
        {
            right_sum += r;
            ++right_n;
        }
        else
        {
            left_sum += r;
            ++left_n;
        }
    }
    if (nearest > 1e8)
    {
        diagCtrl(spinning ? "spin" : "free", e, -1.0, v, w); // 前锥内无有效回波，不受避障约束
        return;
    }

    if (nearest < 0.22)
    {
        // 硬停 + 朝更空侧转向；方向带滞回（对称贴墙时左右平均相近，逐周期翻转会在原地摆动）
        // 阈值=车身半径+7cm 余量：窄通道内 0.30m 处即停会把走廊全程拖成 v=0（顶死循环）
        v = 0.0;
        if (hard_avoid_dir_ == 0)
        {
            const double left_avg = left_n ? left_sum / left_n : 0.0;
            const double right_avg = right_n ? right_sum / right_n : 0.0;
            // 仅在一侧明显更空（1.2 倍）时才取该侧；两侧相近则朝目标侧转，
            // 避免避障转向与跟踪方向持续反号（转向半天气不收敛）
            if (left_avg > right_avg * 1.2)
                hard_avoid_dir_ = 1;
            else if (right_avg > left_avg * 1.2)
                hard_avoid_dir_ = -1;
            else
                hard_avoid_dir_ = (e >= 0) ? 1 : -1;
        }
        w = hard_avoid_dir_ * w_max_ * 0.7;
        diagCtrl("hard-stop", e, nearest, v, w);
    }
    else
    {
        hard_avoid_dir_ = 0; // 前方净空，清除滞回方向
        if (nearest < 0.40 && !spinning)
            v *= 0.6; // 减速接近（窄通道常态 0.28-0.55m，过宽的减速带会把巡航拖到 0.4×v_max）
        diagCtrl(spinning ? "spin" : (nearest < 0.40 ? "slow" : "arc"), e, nearest, v, w);
    }
}

// 控制环诊断日志（2Hz 限频）：决策模式/朝向误差/前锥最近障碍/输出指令。
// 排查"走得慢/顶死"类问题时看这条：escape/spin/hard-stop 的占比与 near 距离一望即知。
// near=-1 表示无雷达数据或前锥内无回波；e 仅在跟踪分支有意义
void BaseNaviNodelet::diagCtrl(const char *mode, double e, double near_m, double v, double w)
{
    const ros::Time now = ros::Time::now();
    if (!diag_time_.isZero() && (now - diag_time_).toSec() < 0.5)
        return;
    diag_time_ = now;
    NODELET_INFO("ctrl: mode=%-9s e=%+.2f near=%5.2f v=%+.3f w=%+.2f", mode, e, near_m, v, w);
}

void BaseNaviNodelet::publishVelocity(double v, double w)
{
    geometry_msgs::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    vel_pub_.publish(msg);
}

void BaseNaviNodelet::publishZeroVel()
{
    publishVelocity(0.0, 0.0);
}

void BaseNaviNodelet::publishState(bool force)
{
    const ros::Time now = ros::Time::now();
    if (!force && (now - state_pub_time_).toSec() < state_period_)
        return;
    state_pub_time_ = now;

    custom_msgs::NaviState msg;
    switch (state_)
    {
    case NaviRunState::RUNNING:
        msg.state = custom_msgs::NaviState::RUNNING;
        break;
    case NaviRunState::FINISHED:
        msg.state = custom_msgs::NaviState::FINISH;
        break;
    case NaviRunState::PAUSED:
        msg.state = custom_msgs::NaviState::PAUSED;
        break;
    case NaviRunState::ABORTED:
        msg.state = custom_msgs::NaviState::ABORT;
        break;
    default:
        return; // IDLE 不上报
    }

    Pose2d pose;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        pose = pose_;
    }
    msg.pose_x = pose.x;
    msg.pose_y = pose.y;
    msg.pose_yaw = pose.yaw;
    msg.target_x = has_target_ ? target_x_ : std::nanf("");
    msg.target_y = has_target_ ? target_y_ : std::nanf("");
    msg.frontier_count = last_frontier_count_;
    msg.no_target_count = no_target_count_;
    msg.explored_area_m2 = explored_area_m2_;
    msg.stamp = now;
    state_pub_.publish(msg);
}

void BaseNaviNodelet::publishGoal()
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_x_;
    msg.pose.position.y = target_y_;
    msg.pose.orientation.w = 1.0;
    goal_pub_.publish(msg);
}

void BaseNaviNodelet::blacklistTarget(double x, double y)
{
    blacklist_.emplace_back(x, y);
    if (blacklist_.size() > 16)
        blacklist_.erase(blacklist_.begin()); // 上限保护：淘汰最旧
}

bool BaseNaviNodelet::isBlacklisted(double x, double y) const
{
    for (const auto &b : blacklist_)
        if (std::hypot(b.first - x, b.second - y) < 1.0)
            return true;
    return false;
}

} // namespace base_navi_nodelet

PLUGINLIB_EXPORT_CLASS(base_navi_nodelet::BaseNaviNodelet, nodelet::Nodelet)
