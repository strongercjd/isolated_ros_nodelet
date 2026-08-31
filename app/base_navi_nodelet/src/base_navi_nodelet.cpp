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
    control_period_ = control_hz > 0.1 ? 1.0 / control_hz : 0.1; // 控制频率（Hz）
    pnh.param("v_max", v_max_, 0.25);
    pnh.param("w_max", w_max_, 0.8);
    pnh.param("goal_tolerance", goal_tolerance_, 0.25);
    pnh.param("stuck_target_time_s", stuck_target_time_s_, 30.0);
    pnh.param("state_period", state_period_, 1.0);

    pose_sub_ = pnh.subscribe("/slam2d/pose", 5, &BaseNaviNodelet::poseCallback, this);//接收机器人位姿
    scan_sub_ = pnh.subscribe("/mycar/base_scan", 5, &BaseNaviNodelet::scanCallback, this);//接收激光雷达数据
    odom_sub_ = pnh.subscribe("/mycar/odom", 10, &BaseNaviNodelet::odomCallback, this);//接收里程计数据
    cmd_sub_ = pnh.subscribe("/base_navi/cmd", 5, &BaseNaviNodelet::cmdCallback, this);//接收导航指令（含 CMD_GOAL 目标点）

    vel_pub_ = pnh.advertise<geometry_msgs::Twist>("/mycar/cmd_vel", 5);
    state_pub_ = pnh.advertise<custom_msgs::NaviState>("/base_navi/state", 5);
    goal_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("/base_navi/goal", 5);//发布目标点

    worker_ = std::thread(&BaseNaviNodelet::workerLoop, this);
    NODELET_INFO("BaseNaviNodelet initialized: hz=%.0f v_max=%.2f w_max=%.2f tol=%.2fm "
                 "goal_timeout=%.0fs",
                 1.0 / control_period_, v_max_, w_max_, goal_tolerance_, stuck_target_time_s_);
}

// ---- 回调：只拷贝快照（manager spin 线程）----

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
        cmd_queue_.push_back(*msg);
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
/**
 * @brief 控制循环
 */
void BaseNaviNodelet::controlOnce()
{
    const ros::Time now = ros::Time::now();

    // 1. 命令队列
    std::deque<custom_msgs::NaviCmd> cmds;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        cmds.swap(cmd_queue_);
    }
    for (const auto &c : cmds)
        handleCmd(c);

    if (state_ == NaviRunState::IDLE)
        return; // 未使能：不发布速度/状态

    // 2. 快照
    Pose2d pose, odom_pose;
    ros::Time pose_stamp;
    bool have_pose = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
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

    // 3. 运动控制
    if (state_ == NaviRunState::RUNNING)
    {
        if (!have_pose)
        {
            publishZeroVel(); // 无位姿先不动
        }
        else if (!has_target_)
        {
            publishZeroVel(); // 已使能无目标：零速待命，等任务层 CMD_GOAL
        }
        else
        {
            // 目标超时："看得见够不着"防御。拉黑决定权在任务层，这里只报 ABORT。
            // 不依赖重选次数：目标一直持有时 picks 恒为 1，以首拣时间为准。
            if ((now - target_first_pick_).toSec() > stuck_target_time_s_)
            {
                abortCurrentGoal(custom_msgs::NaviState::REASON_GOAL_LOST, "goal timeout");
                publishZeroVel();
                publishState(false);
                return;
            }

            const double dist = std::hypot(target_x_ - pose.x, target_y_ - pose.y);
            if (dist < goal_tolerance_)
            {
                NODELET_INFO("target reached (%.2f, %.2f)", target_x_, target_y_);
                has_target_ = false;
                state_ = NaviRunState::REACHED;
                publishZeroVel();
                publishState(true);
                return;
            }

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
                        abortCurrentGoal(custom_msgs::NaviState::REASON_STUCK, "repeatedly stuck");
                        publishZeroVel();
                        publishState(false);
                        return;
                    }
                }
                stuck_check_time_ = now;
                stuck_check_pose_ = odom_pose;
            }

            publishVelocity(v, w);
        }
    }
    else if (state_ == NaviRunState::PAUSED)
    {
        publishZeroVel();
    }
    // REACHED / ABORTED：零速驻留待命（到达/放弃时已发过一次，心跳在下方补发）

    // 4. 状态心跳
    publishState(false);
}

void BaseNaviNodelet::handleCmd(const custom_msgs::NaviCmd &msg)
{
    switch (msg.cmd)
    {
    case custom_msgs::NaviCmd::CMD_START:
        if (state_ == NaviRunState::RUNNING)
        {
            NODELET_WARN("START(seq=%u) ignored: already running", msg.seq);
            return;
        }
        state_ = NaviRunState::RUNNING;
        has_target_ = false;
        stuck_count_ = 0;
        stuck_check_time_ = ros::Time();
        escape_until_ = ros::Time();
        hard_avoid_dir_ = 0;
        start_time_ = ros::Time::now();
        NODELET_INFO("navi START (seq=%u), waiting for goal", msg.seq);
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_GOAL:
        // RUNNING/REACHED/ABORT 均接受：REACHED→接续下一目标，ABORT→换点重试。
        // IDLE（未使能）/PAUSED（暂停语义）不接受，由任务层保证时序。
        if (state_ != NaviRunState::RUNNING && state_ != NaviRunState::REACHED &&
            state_ != NaviRunState::ABORTED)
        {
            NODELET_WARN("GOAL(seq=%u) ignored in state %d", msg.seq, static_cast<int>(state_));
            return;
        }
        state_ = NaviRunState::RUNNING;
        has_target_ = true;
        target_x_ = msg.goal_x;
        target_y_ = msg.goal_y;
        if (msg.tolerance > 0.0)
            goal_tolerance_ = msg.tolerance;
        target_first_pick_ = ros::Time::now();
        stuck_count_ = 0; // 换目标重置顶死计数
        stuck_check_time_ = ros::Time();
        NODELET_INFO("navi GOAL (seq=%u) -> (%.2f, %.2f) tol=%.2fm",
                     msg.seq, target_x_, target_y_, goal_tolerance_);
        publishGoal();
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_PAUSE:
        if (state_ != NaviRunState::RUNNING)
            return;
        state_ = NaviRunState::PAUSED;
        publishZeroVel();
        NODELET_INFO("navi PAUSE (seq=%u)", msg.seq);
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_RESUME:
        if (state_ != NaviRunState::PAUSED)
            return;
        state_ = NaviRunState::RUNNING;
        NODELET_INFO("navi RESUME (seq=%u)", msg.seq);
        publishState(true);
        break;
    case custom_msgs::NaviCmd::CMD_STOP:
        state_ = NaviRunState::IDLE;
        has_target_ = false;
        publishZeroVel();
        publishZeroVel();
        NODELET_INFO("navi STOP (seq=%u), robot parked in place (cost %.0fs)",
                     msg.seq, (ros::Time::now() - start_time_).toSec());
        return;
    default:
        NODELET_WARN("unknown NaviCmd %u (seq=%u)", msg.cmd, msg.seq);
        return;
    }
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

// 放弃当前目标：转 ABORTED 驻留，拉黑决定权在任务层（据 msg->target 拉黑后换点）
void BaseNaviNodelet::abortCurrentGoal(uint8_t reason, const char *why)
{
    abort_reason_ = reason;
    state_ = NaviRunState::ABORTED;
    has_target_ = false;
    stuck_count_ = 0;
    NODELET_WARN("navi ABORT goal (%.2f, %.2f): %s", target_x_, target_y_, why);
}

/**
 * @brief 发布导航状态
 */
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
    case NaviRunState::REACHED:
        msg.state = custom_msgs::NaviState::REACHED;
        break;
    case NaviRunState::PAUSED:
        msg.state = custom_msgs::NaviState::PAUSED;
        break;
    case NaviRunState::ABORTED:
        msg.state = custom_msgs::NaviState::ABORT;
        msg.reason = abort_reason_;
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
    // REACHED/ABORT 驻留时上报刚处理完的目标（任务层据此匹配/拉黑）；仅 RUNNING 无目标时 NaN
    const bool report_target = has_target_ || state_ == NaviRunState::REACHED ||
                               state_ == NaviRunState::ABORTED;
    msg.target_x = report_target ? target_x_ : std::nanf("");
    msg.target_y = report_target ? target_y_ : std::nanf("");
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

} // namespace base_navi_nodelet

PLUGINLIB_EXPORT_CLASS(base_navi_nodelet::BaseNaviNodelet, nodelet::Nodelet)
