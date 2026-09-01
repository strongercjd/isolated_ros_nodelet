#include "fastbuild_task_nodelet.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace
{

double yawFromQuat(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// 两位小数的浮点格式化（决策日志 JSON 用，避免引入 nlohmann 依赖）
std::string fmt2(double v)
{
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.2f", v);
    return buf;
}

} // namespace

namespace fastbuild_task_nodelet
{
void FastbuildTaskNodelet::onInit()
{
    ros::NodeHandle pnh = getPrivateNodeHandle();

    pnh.param("max_duration_s", max_duration_s_, 600.0);//600s 超时时间 ros参数
    pnh.param("occ_free_th", frontier_params_.occ_free_th, 25);
    pnh.param("occ_occ_th", frontier_params_.occ_occ_th, 65);
    pnh.param("min_cluster", frontier_params_.min_cluster, 3);
    pnh.param("stop_count_limit", stop_count_limit_, 10);
    pnh.param("robot_radius", robot_radius_, 0.15);
    // 通行性窗口：覆盖机器人直径（至少 5x5）。
    // 与 base_navi 的避障阈值同源（车体半径），改车体需两侧同步
    frontier_params_.pass_half_window =
        std::max(2, static_cast<int>(std::ceil(robot_radius_ / 0.05)));

    cmd_sub_ = pnh.subscribe("/fastbuild_task/cmd", 5, &FastbuildTaskNodelet::taskCmdCallback, this);
    navi_state_sub_ = pnh.subscribe("/base_navi/state", 5, &FastbuildTaskNodelet::naviStateCallback, this);
    map_sub_ = pnh.subscribe("/slam2d/map", 1, &FastbuildTaskNodelet::mapCallback, this);//接收地图（选点用）
    pose_sub_ = pnh.subscribe("/slam2d/pose", 5, &FastbuildTaskNodelet::poseCallback, this);//接收位姿（选点用）

    navi_cmd_pub_ = pnh.advertise<custom_msgs::NaviCmd>("/base_navi/cmd", 5);
    vel_pub_ = pnh.advertise<geometry_msgs::Twist>("/mycar/cmd_vel", 5);
    state_pub_ = pnh.advertise<custom_msgs::TaskState>("/fastbuild_task/state", 5, true /*latch：迟订阅者也能拿到终态*/);
    decision_pub_ = pnh.advertise<custom_msgs::FastBuildDecision>("/fastbuild_task/decision", 5);

    decision_timer_ = pnh.createTimer(ros::Duration(1.0), &FastbuildTaskNodelet::decisionTimer, this);//1Hz 选点决策
    watchdog_ = pnh.createTimer(ros::Duration(5.0), &FastbuildTaskNodelet::watchdogTimer, this);//5s看门狗
    NODELET_INFO("FastbuildTaskNodelet initialized: max_duration_s=%.0f occ_th=(%d,%d) pass_half=%d "
                 "stop_limit=%d",
                 max_duration_s_, frontier_params_.occ_free_th, frontier_params_.occ_occ_th,
                 frontier_params_.pass_half_window, stop_count_limit_);
}

/**
 * @brief 处理快建任务命令的回调函数 主要是启动和取消任务
 * @param msg 接收到的快建命令消息指针
 */
void FastbuildTaskNodelet::taskCmdCallback(const custom_msgs::FastBuildCmd::ConstPtr &msg)
{
    if (msg->cmd == custom_msgs::FastBuildCmd::CMD_START)
    {
        if (state_ == TaskRunState::RUNNING)
        {
            NODELET_WARN("START(seq=%u) ignored: task already running", msg->seq); // 对齐"禁重复启动"
            return;
        }
        if (msg->type != "build_map")
        {
            NODELET_WARN("unsupported type '%s' (seq=%u), only build_map in this env",
                         msg->type.c_str(), msg->seq);
            return;
        }
        state_ = TaskRunState::RUNNING;
        t0_ = ros::Time::now();
        latest_area_m2_ = 0.0;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            no_target_count_ = 0;
            goal_in_flight_ = false;
            blacklist_.clear();
            last_pick_count_ = map_count_; // 已收到的旧图不用于首轮选点，等下一张
        }
        NODELET_INFO("task START (seq=%u type=%s map_type=%d from_charger=%d)", msg->seq,
                     msg->type.c_str(), msg->map_type, msg->start_from_charger ? 1 : 0);
        publishTaskState(custom_msgs::TaskState::STATE_RUNNING, 255, 0.0, 0.0);//发布快建任务状态
        publishDecision({}, false, 0.0, 0.0, custom_msgs::TaskState::STATE_RUNNING);//任务开始边界标记
        sendNaviCmd(custom_msgs::NaviCmd::CMD_START);//使能导航（执行器进入待命，等 CMD_GOAL）
    }
    else if (msg->cmd == custom_msgs::FastBuildCmd::CMD_CANCEL)
    {
        if (state_ != TaskRunState::RUNNING)
        {
            NODELET_WARN("CANCEL(seq=%u) ignored: task not running", msg->seq);
            return;
        }
        finishTask(custom_msgs::TaskState::REASON_INTERRUPT, "cancelled by command");//取消任务
    }
    else
    {
        NODELET_WARN("unknown FastBuildCmd %u (seq=%u)", msg->cmd, msg->seq);
    }
}

/**
 * @brief 处理导航状态消息的回调函数。
 * 执行器驻留态驱动决策：REACHED→接续选点；ABORT→拉黑换点。
 * 目标匹配（<1m，对齐原"同目标"语义）做幂等：陈旧心跳不触发重复选点。
 * @param msg 接收到的导航状态消息指针
 */
void FastbuildTaskNodelet::naviStateCallback(const custom_msgs::NaviState::ConstPtr &msg)
{
    if (state_ != TaskRunState::RUNNING)
        return;

    const bool reached = msg->state == custom_msgs::NaviState::REACHED;
    const bool aborted = msg->state == custom_msgs::NaviState::ABORT;
    if (!reached && !aborted)
        return;

    bool matched = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        // NaN 目标（异常态）hypot 为 NaN，比较为 false，自然落入不匹配分支
        matched = goal_in_flight_ &&
                  std::hypot(msg->target_x - goal_x_, msg->target_y - goal_y_) < 1.0;
        if (matched)
            goal_in_flight_ = false;
    }
    if (!matched)
        return;

    if (aborted)
    {
        NODELET_WARN("navi ABORT (%s) at (%.2f, %.2f), blacklisted",
                     msg->reason == custom_msgs::NaviState::REASON_STUCK ? "stuck" : "goal_lost",
                     msg->target_x, msg->target_y);
        blacklistTarget(msg->target_x, msg->target_y);
    }
    pickAndSendGoal(); // 不等 1Hz timer：到达/放弃后立即接续，减少停顿
}

void FastbuildTaskNodelet::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    latest_map_ = *msg; // 4MB 拷贝 ≈ 1-2ms @1Hz
    map_new_ = true;
    ++map_count_;
}

void FastbuildTaskNodelet::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    pose_ = *msg;
}

/**
 * @brief 1Hz 选点决策：新地图上做 frontier 检测、在途目标失效判定、无可达目标计数
 * @param event 定时器事件
 */
void FastbuildTaskNodelet::decisionTimer(const ros::TimerEvent &)
{
    if (state_ != TaskRunState::RUNNING)
        return;

    nav_msgs::OccupancyGrid map;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!map_new_)
            return; // 无新地图：决策只随地图走（对齐原 1Hz 地图节拍）
        map = latest_map_; // 拷贝出快照（≈1-2ms），计算放锁外
        map_new_ = false;
    }
    latest_area_m2_ = freeAreaM2(map, frontier_params_.occ_free_th);
    maintainGoalOnMap(map);
}

/**
 * @brief 新地图上的目标维护：在途目标失效判定；无在途目标则选点下发。
 * 目标超时/顶死由 base_navi 判（ABORT 上报），此处不重复计时。
 * @param map 本轮新地图快照
 */
void FastbuildTaskNodelet::maintainGoalOnMap(const nav_msgs::OccupancyGrid &map)
{
    if (goal_in_flight_)
    {
        // 失效判定：候选簇里已找不到 1m 内的对应簇（簇消失/被并入占用区）
        const std::vector<FrontierGoal> goals = detectFrontiers(map, frontier_params_);
        double nearest = 1e9;
        for (const auto &g : goals)
            nearest = std::min(nearest, std::hypot(g.wx - goal_x_, g.wy - goal_y_));
        if (nearest <= 1.0)
            return; // 目标仍有效：保持方向惯性，不因新图换目标

        NODELET_INFO("goal (%.2f, %.2f) vanished from frontier set", goal_x_, goal_y_);
        {
            std::lock_guard<std::mutex> lk(mtx_);
            goal_in_flight_ = false;
        }
    }
    pickAndSendGoal();
}

/**
 * @brief 在当前地图上选点并下发给 base_navi。
 * 同一张地图只选一次（防 REACHED 快速重触发下同图反复选点）；
 * 选不出可达目标时随新地图累计 no_target_count，到限即任务自然结束。
 */
void FastbuildTaskNodelet::pickAndSendGoal()
{
    nav_msgs::OccupancyGrid map;
    geometry_msgs::PoseStamped pose;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (map_count_ == 0)
            return; // 开局等待地图（对齐原 WAITING 语义），不计失败
        if (map_count_ == last_pick_count_)
            return; // 本图已选过：等下一张
        map = latest_map_;
        pose = pose_;
        last_pick_count_ = map_count_;
    }

    std::vector<FrontierGoal> goals = detectFrontiers(map, frontier_params_);

    // 重选：剔除拉黑区，最近优先；直线穿墙的候选重罚（+8m 虚拟距离），
    // 避免选中墙对面目标后纯跟踪控制律一路顶死。全候选被挡时惩罚均摊，退化为旧行为
    const FrontierGoal *best = nullptr;
    double best_key = 1e9;
    bool best_direct = false;
    for (auto &g : goals)
    {
        if (isBlacklisted(g.wx, g.wy))
            continue;
        const double dist = std::hypot(g.wx - pose.pose.position.x, g.wy - pose.pose.position.y);
        const bool direct = lineReachable(map, pose.pose.position.x, pose.pose.position.y,
                                          g.wx, g.wy, frontier_params_);
        g.via = direct ? 0 : 1; // 记录给决策日志（flat_sim_viewer 画点区分）
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
    {
        // 对齐 explorestatestopcount：连续选不出可达目标即累计。
        // 候选非空但全被拉黑同样属于"无可达目标"（否则机器人零速悬停到 watchdog）
        publishDecision(goals, false, 0.0, 0.0, custom_msgs::TaskState::STATE_RUNNING);
        ++no_target_count_;
        NODELET_WARN("no reachable goal for %u/%d rounds (candidates=%zu)",
                     no_target_count_, stop_count_limit_, goals.size());
        if (no_target_count_ > static_cast<uint32_t>(stop_count_limit_))
            finishTask(custom_msgs::TaskState::REASON_IS_FINISH, "no reachable frontier");
        return;
    }

    no_target_count_ = 0;
    sendGoal(best->wx, best->wy);
    NODELET_INFO("new goal (%.2f, %.2f) size=%zu dist=%.1fm via=%s",
                 best->wx, best->wy, best->size,
                 std::hypot(best->wx - pose.pose.position.x, best->wy - pose.pose.position.y),
                 best_direct ? "line" : "detour");
    publishDecision(goals, true, best->wx, best->wy, custom_msgs::TaskState::STATE_RUNNING);
}

/**
 * @brief 看门狗定时器回调函数，主要是检测任务是否超时
 * @param event 定时器事件
 */
void FastbuildTaskNodelet::watchdogTimer(const ros::TimerEvent &)
{
    if (state_ != TaskRunState::RUNNING)
        return;
    if ((ros::Time::now() - t0_).toSec() > max_duration_s_)
        finishTask(custom_msgs::TaskState::REASON_ERROR, "watchdog timeout");//
}


// 收尾序列：先置 IDLE（防重入），再停 navi、零速兜底、发终态（对齐原 setComplete + clearSpeed）
void FastbuildTaskNodelet::finishTask(uint8_t reason, const char *why)
{
    state_ = TaskRunState::IDLE;
    const double cost = (ros::Time::now() - t0_).toSec();

    sendNaviCmd(custom_msgs::NaviCmd::CMD_STOP);
    publishZeroVel(); // 停在原地：保持式 cmd_vel，双发兜底

    publishTaskState(custom_msgs::TaskState::STATE_DONE, reason, latest_area_m2_, cost);
    NODELET_INFO("task DONE: reason=%u (%s) area=%.1f m2 cost=%.0f s", reason, why,
                 latest_area_m2_, cost);
    publishDecision({}, false, 0.0, 0.0, custom_msgs::TaskState::STATE_DONE);//任务结束边界标记
}
/**
 * @brief 发送导航命令
 * @param cmd 导航命令
 */
void FastbuildTaskNodelet::sendNaviCmd(uint8_t cmd)
{
    custom_msgs::NaviCmd msg;
    msg.cmd = cmd;
    msg.seq = ++seq_;
    msg.stamp = ros::Time::now();
    navi_cmd_pub_.publish(msg);
}

/**
 * @brief 下发目标点（CMD_GOAL）并记为在途
 * @param x 目标 x（map 系）
 * @param y 目标 y（map 系）
 */
void FastbuildTaskNodelet::sendGoal(double x, double y)
{
    custom_msgs::NaviCmd msg;
    msg.cmd = custom_msgs::NaviCmd::CMD_GOAL;
    msg.seq = ++seq_;
    msg.stamp = ros::Time::now();
    msg.goal_x = static_cast<float>(x);
    msg.goal_y = static_cast<float>(y);
    msg.tolerance = 0.0; // 用执行器默认容差
    navi_cmd_pub_.publish(msg);
    std::lock_guard<std::mutex> lk(mtx_);
    goal_in_flight_ = true;
    goal_x_ = x;
    goal_y_ = y;
}
/**
 * @brief 发布一次决策：ROS 话题 /fastbuild_task/decision + 落盘 FASTBUILD_DECISION 行。
 * 供 flat_sim_viewer 在线显示（订阅话题）与日志回放（解析 custom_ros_nodelet.log，
 * 按时间戳与 map_log bag 匹配）。候选传全量（含被拉黑剔除的），选中点由调用方给。
 * @param goals       本轮全部候选点位（frontier 检测结果）
 * @param has_selected 是否选出目标
 * @param sel_x       最终选中的目标点 x（has_selected 为真时有效）
 * @param sel_y       最终选中的目标点 y
 * @param task_state  任务状态（对齐 TaskState：1=RUNNING 4=DONE）
 */
void FastbuildTaskNodelet::publishDecision(const std::vector<FrontierGoal> &goals, bool has_selected,
                                           double sel_x, double sel_y, uint8_t task_state)
{
    std::vector<std::pair<double, double>> blacklist;
    double area;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        blacklist = blacklist_;
        area = latest_area_m2_;
    }

    custom_msgs::FastBuildDecision msg;
    msg.seq = ++seq_;
    msg.stamp = ros::Time::now();
    msg.has_selected = has_selected;
    msg.selected_x = static_cast<float>(sel_x);
    msg.selected_y = static_cast<float>(sel_y);
    msg.area_m2 = static_cast<float>(area);
    msg.task_state = task_state;
    for (const auto &g : goals)
    {
        custom_msgs::FastBuildPoint pt;
        pt.x = static_cast<float>(g.wx);
        pt.y = static_cast<float>(g.wy);
        pt.theta = 0.0f;
        msg.candidates.push_back(pt);
        msg.candidate_size.push_back(static_cast<uint32_t>(g.size));
        msg.candidate_via.push_back(g.via);
    }
    for (const auto &b : blacklist)
    {
        custom_msgs::FastBuildPoint pt;
        pt.x = static_cast<float>(b.first);
        pt.y = static_cast<float>(b.second);
        pt.theta = 0.0f;
        msg.blacklist.push_back(pt);
    }
    decision_pub_.publish(msg);

    // 落盘单行 JSON（数值型字段无转义问题）：sec/nsec 是与 slam bag 匹配的权威时间戳
    std::string j;
    j.reserve(128 + goals.size() * 48 + blacklist.size() * 32);
    j = "{\"seq\":" + std::to_string(msg.seq) + ",\"sec\":" + std::to_string(msg.stamp.sec) +
        ",\"nsec\":" + std::to_string(msg.stamp.nsec) + ",\"task_state\":" +
        std::to_string(msg.task_state) + ",\"area\":" + fmt2(msg.area_m2) +
        ",\"has_selected\":" + (has_selected ? "true" : "false") +
        ",\"sel_x\":" + fmt2(msg.selected_x) + ",\"sel_y\":" + fmt2(msg.selected_y) +
        ",\"candidates\":[";
    for (size_t i = 0; i < goals.size(); ++i)
    {
        if (i)
            j += ",";
        j += "{\"x\":" + fmt2(goals[i].wx) + ",\"y\":" + fmt2(goals[i].wy) +
             ",\"size\":" + std::to_string(goals[i].size) +
             ",\"via\":" + std::to_string(goals[i].via) + "}";
    }
    j += "],\"blacklist\":[";
    for (size_t i = 0; i < blacklist.size(); ++i)
    {
        if (i)
            j += ",";
        j += "{\"x\":" + fmt2(blacklist[i].first) + ",\"y\":" + fmt2(blacklist[i].second) + "}";
    }
    j += "]}";
    NODELET_INFO("FASTBUILD_DECISION %s", j.c_str());
}

/**
 * @brief 发布快建任务状态
 * @param state 任务状态
 * @param reason 任务结束原因
 * @param area_m2 任务覆盖面积
 * @param cost_time_s 任务耗时
 */
void FastbuildTaskNodelet::publishTaskState(uint8_t state, uint8_t reason,
                                            double area_m2, double cost_time_s)
{
    custom_msgs::TaskState msg;
    msg.state = state;
    msg.finish_reason = reason;
    msg.area_m2 = area_m2;
    msg.cost_time_s = cost_time_s;
    msg.seq = ++seq_;
    msg.stamp = ros::Time::now();
    state_pub_.publish(msg);
}

void FastbuildTaskNodelet::publishZeroVel()
{
    geometry_msgs::Twist msg;
    vel_pub_.publish(msg);
}

void FastbuildTaskNodelet::blacklistTarget(double x, double y)
{
    std::lock_guard<std::mutex> lk(mtx_);
    blacklist_.emplace_back(x, y);
    if (blacklist_.size() > 16)
        blacklist_.erase(blacklist_.begin()); // 上限保护：淘汰最旧
}

bool FastbuildTaskNodelet::isBlacklisted(double x, double y) const
{
    for (const auto &b : blacklist_)
        if (std::hypot(b.first - x, b.second - y) < 1.0)
            return true;
    return false;
}

} // namespace fastbuild_task_nodelet

PLUGINLIB_EXPORT_CLASS(fastbuild_task_nodelet::FastbuildTaskNodelet, nodelet::Nodelet)
