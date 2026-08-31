#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <custom_msgs/NaviCmd.h>
#include <custom_msgs/NaviState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

namespace base_navi_nodelet
{

// 内部运行状态。纯执行器语义：目标点由任务层下发（NaviCmd::CMD_GOAL），
// 目标选择/拉黑/探索结束判断都在任务层（fastbuild_task），本节点只负责"到点"。
enum class NaviRunState
{
    IDLE,    // 未收到 START
    RUNNING, // 执行中（无目标时零速待命，等 CMD_GOAL）
    REACHED, // 到达目标，零速待命（对齐 NaviState::REACHED）
    PAUSED,  // 暂停（零速）
    ABORTED  // 当前目标不可达（顶死/超时），零速待命（对齐 NaviState::ABORT）
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
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void cmdCallback(const custom_msgs::NaviCmd::ConstPtr &msg);

    // ---- worker 线程（10Hz 控制环）----
    void workerLoop();
    void controlOnce();
    void handleCmd(const custom_msgs::NaviCmd &msg);
    void computeVelocity(const Pose2d &pose, double &v, double &w);
    void diagCtrl(const char *mode, double e, double near_m, double v, double w);
    void publishVelocity(double v, double w);
    void publishZeroVel();
    void publishState(bool force);
    void publishGoal();
    void abortCurrentGoal(uint8_t reason, const char *why);

    // ---- 参数（~control_hz 等，plugins.json args 或默认值）----
    double control_period_ = 0.1;       // ~control_hz(10) 的倒数
    double v_max_ = 0.25;               // m/s
    double w_max_ = 0.8;                // rad/s
    double goal_tolerance_ = 0.25;      // m，到达判定（CMD_GOAL 可覆盖）
    double stuck_target_time_s_ = 30.0; // 持有同一目标未到达的超时，超时 ABORT
    double state_period_ = 1.0;         // state 心跳周期（s）

    // ---- 回调 → worker 的快照（受 mtx_ 保护）----
    Pose2d pose_;
    ros::Time pose_stamp_;
    Pose2d odom_pose_;
    sensor_msgs::LaserScan latest_scan_;
    bool have_scan_ = false;
    std::deque<custom_msgs::NaviCmd> cmd_queue_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic_bool stop_{false};

    // ---- worker 私有状态（无锁）----
    NaviRunState state_ = NaviRunState::IDLE;
    ros::Time state_pub_time_;    // 上次心跳发布
    ros::Time start_time_;        // 本次使能起点（耗时日志基准）
    uint8_t abort_reason_ = 0;    // ABORTED 时对齐 NaviState::REASON_*

    bool has_target_ = false;
    double target_x_ = 0.0, target_y_ = 0.0;
    ros::Time target_first_pick_; // 当前目标首次下发时间

    // 顶死检测（持有目标但 odom 位移≈0，含避障硬停摆动的情形）
    ros::Time stuck_check_time_;
    Pose2d stuck_check_pose_;
    int stuck_count_ = 0;
    ros::Time escape_until_; // 脱困动作（倒退+转向）截止时间
    int hard_avoid_dir_ = 0; // 硬停避障转向方向滞回（±1，防对称摆动；净空后清零）
    ros::Time diag_time_;   // 控制环诊断日志限频（2Hz）

    // ---- ROS 接口（私有句柄 → /base_navi/...，输入输出经 remap）----
    ros::Subscriber pose_sub_, scan_sub_, odom_sub_, cmd_sub_;
    ros::Publisher vel_pub_, state_pub_, goal_pub_;
};

} // namespace base_navi_nodelet
