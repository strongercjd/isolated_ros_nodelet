#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <custom_msgs/NaviCmd.h>
#include <custom_msgs/NaviState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "frontier_detector.h"

namespace base_navi_nodelet
{

// 内部运行状态（对外映射为 NaviState 的 EXPLORESTATE 语义值）
enum class NaviRunState
{
    IDLE,     // 未收到 START
    RUNNING,  // 探索中
    PAUSED,   // 暂停（零速）
    FINISHED, // 连续无可达目标，自然结束（对齐 THREAD_FINISH）
    ABORTED   // 内部异常（对齐 THREAD_ABORT）
};

struct Pose2d
{
    double x = 0.0, y = 0.0, yaw = 0.0;
};

class BaseNaviNodelet : public nodelet::Nodelet
{
public:
    BaseNaviNodelet() = default;
    ~BaseNaviNodelet() override;

private:
    virtual void onInit();

    // ---- 回调（manager spin 线程，仅拷贝快照）----
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void cmdCallback(const custom_msgs::NaviCmd::ConstPtr &msg);

    // ---- worker 线程（10Hz 控制环）----
    void workerLoop();
    void controlOnce();
    void handleCmd(uint8_t cmd, uint32_t seq);
    void maintainTarget(const nav_msgs::OccupancyGrid &map,
                        const std::vector<FrontierGoal> &goals, const ros::Time &now);
    void computeVelocity(const Pose2d &pose, double &v, double &w);
    void diagCtrl(const char *mode, double e, double near_m, double v, double w);
    void publishVelocity(double v, double w);
    void publishZeroVel();
    void publishState(bool force);
    void publishGoal();
    void blacklistTarget(double x, double y);
    bool isBlacklisted(double x, double y) const;

    // ---- 参数（~control_hz 等，plugins.json args 或默认值）----
    double control_period_ = 0.1;   // ~control_hz(10) 的倒数
    double v_max_ = 0.25;           // m/s
    double w_max_ = 0.8;            // rad/s
    int stop_count_limit_ = 10;     // 连续无可达目标轮数上限（对齐 explorestatestopcount）
    double robot_radius_ = 0.15;    // m（通行性窗口）
    double goal_tolerance_ = 0.25;  // m，到达判定
    double stuck_target_time_s_ = 30.0; // 持有同一目标未到达的超时拉黑
    double state_period_ = 1.0;     // state 心跳周期（s）
    FrontierParams frontier_params_;

    // ---- 回调 → worker 的快照（受 mtx_ 保护）----
    nav_msgs::OccupancyGrid latest_map_;
    bool map_new_ = false;
    uint64_t map_count_ = 0;
    Pose2d pose_;
    ros::Time pose_stamp_;
    Pose2d odom_pose_;
    sensor_msgs::LaserScan latest_scan_;
    bool have_scan_ = false;
    std::deque<std::pair<uint8_t, uint32_t>> cmd_queue_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic_bool stop_{false};

    // ---- worker 私有状态（无锁）----
    NaviRunState state_ = NaviRunState::IDLE;
    ros::Time state_pub_time_;      // 上次心跳发布
    ros::Time start_time_;          // 本次任务起点（面积/耗时统计基准）
    uint64_t map_seen_ = 0;         // worker 已处理的地图数

    bool has_target_ = false;
    double target_x_ = 0.0, target_y_ = 0.0;
    ros::Time target_first_pick_;   // 同一目标首次选中时间
    int target_picks_ = 0;          // 同一目标（1m 内）被选中次数
    uint32_t no_target_count_ = 0;  // 连续无可达目标轮数（仅新地图时累计）
    uint32_t last_frontier_count_ = 0; // 最近一轮候选 frontier 聚类数
    double explored_area_m2_ = 0.0;
    std::vector<std::pair<double, double>> blacklist_; // 拉黑点（1m 邻域内不再选目标）

    // 顶死检测（持有目标但 odom 位移≈0，含避障硬停摆动的情形）
    ros::Time stuck_check_time_;
    Pose2d stuck_check_pose_;
    int stuck_count_ = 0;
    ros::Time escape_until_; // 脱困动作（倒退+转向）截止时间
    int hard_avoid_dir_ = 0; // 硬停避障转向方向滞回（±1，防对称摆动；净空后清零）
    ros::Time diag_time_;   // 控制环诊断日志限频（2Hz）

    // ---- ROS 接口（私有句柄 → /base_navi/...，输入输出经 remap）----
    ros::Subscriber map_sub_, pose_sub_, scan_sub_, odom_sub_, cmd_sub_;
    ros::Publisher vel_pub_, state_pub_, goal_pub_;
};

} // namespace base_navi_nodelet
