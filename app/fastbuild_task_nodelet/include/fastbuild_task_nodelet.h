#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <custom_msgs/FastBuildCmd.h>
#include <custom_msgs/NaviCmd.h>
#include <custom_msgs/NaviState.h>
#include <custom_msgs/TaskState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

#include "frontier_detector.h"

namespace fastbuild_task_nodelet
{

// 快建任务节点：决策层。目标选择（frontier 检测/拉黑/直线可达惩罚）与
// 探索结束判断（连续无可达目标轮数）都在这里；base_navi 只负责"到点"。
class FastbuildTaskNodelet : public nodelet::Nodelet
{
public:
    FastbuildTaskNodelet() = default;
    ~FastbuildTaskNodelet() override = default; // 停车由 navi 析构兜底

private:
    virtual void onInit();

    // ---- 回调 / 定时器（manager spin 线程）----
    void taskCmdCallback(const custom_msgs::FastBuildCmd::ConstPtr &msg);
    void naviStateCallback(const custom_msgs::NaviState::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void decisionTimer(const ros::TimerEvent &); // 1Hz：新地图上选点/失效判定/结束判断
    void watchdogTimer(const ros::TimerEvent &); // 5s：任务总时长兜底

    // ---- 决策 ----
    void pickAndSendGoal();                 // 在当前地图上选点并下发（同图只选一次）
    void maintainGoalOnMap(const nav_msgs::OccupancyGrid &map); // 新地图：在途目标失效判定 / 无目标选点
    void finishTask(uint8_t reason, const char *why);
    void sendNaviCmd(uint8_t cmd);
    void sendGoal(double x, double y);
    void publishTaskState(uint8_t state, uint8_t reason, double area_m2, double cost_time_s);
    void publishZeroVel();
    void blacklistTarget(double x, double y);
    bool isBlacklisted(double x, double y) const;

    enum class TaskRunState
    {
        IDLE,    // 等待启动命令
        RUNNING, // 任务进行中
    };

    TaskRunState state_ = TaskRunState::IDLE;
    ros::Time t0_;                  // 任务起点（耗时统计）
    uint32_t seq_ = 0;              // TaskState 序号
    double latest_area_m2_ = 0.0;   // 最近一张地图的已探索面积（收尾上报）
    double max_duration_s_ = 600.0; // 看门狗：任务总时长（frontier 不收敛兜底）

    // ---- 决策参数（原 base_navi 的选点参数随逻辑一起上移）----
    FrontierParams frontier_params_; // occ 阈值/通行窗口/聚类参数
    double robot_radius_ = 0.15;     // m，通行性窗口（与 base_navi 避障阈值同源：车体半径）
    int stop_count_limit_ = 10;      // 连续无可达目标轮数上限（对齐 explorestatestopcount）

    // ---- 决策状态（timer/回调同在 spin 线程，mutex 防 manager 多线程 spin）----
    std::mutex mtx_;
    nav_msgs::OccupancyGrid latest_map_;
    bool map_new_ = false;
    uint64_t map_count_ = 0;         // 已收地图张数（开局等待判定）
    uint64_t last_pick_count_ = 0;   // 最近一次选点所用地图的张数（同图只选一次）
    geometry_msgs::PoseStamped pose_;// 选点用的机器人位姿（slam2d）
    uint32_t no_target_count_ = 0;   // 连续无可达目标轮数（仅新地图时累计）
    bool goal_in_flight_ = false;    // 有已下发、未到达/未放弃的目标
    double goal_x_ = 0.0, goal_y_ = 0.0; // 在途（或最后处理完的）目标
    std::vector<std::pair<double, double>> blacklist_; // 拉黑点（1m 邻域内不再选目标）

    ros::Subscriber cmd_sub_, navi_state_sub_, map_sub_, pose_sub_;
    ros::Publisher navi_cmd_pub_, vel_pub_, state_pub_;
    ros::Timer decision_timer_, watchdog_;
};

} // namespace fastbuild_task_nodelet
