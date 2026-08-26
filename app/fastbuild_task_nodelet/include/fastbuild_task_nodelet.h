#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <custom_msgs/FastBuildCmd.h>
#include <custom_msgs/NaviCmd.h>
#include <custom_msgs/NaviState.h>
#include <custom_msgs/TaskState.h>
#include <geometry_msgs/Twist.h>

namespace fastbuild_task_nodelet
{

class FastbuildTaskNodelet : public nodelet::Nodelet
{
public:
    FastbuildTaskNodelet() = default;
    ~FastbuildTaskNodelet() override = default; // 停车由 navi 析构兜底

private:
    virtual void onInit();

    void taskCmdCallback(const custom_msgs::FastBuildCmd::ConstPtr &msg);
    void naviStateCallback(const custom_msgs::NaviState::ConstPtr &msg);
    void watchdogTimer(const ros::TimerEvent &);

    void finishTask(uint8_t reason, const char *why);
    void sendNaviCmd(uint8_t cmd);
    void publishTaskState(uint8_t state, uint8_t reason, double area_m2, double cost_time_s);
    void publishZeroVel();

    enum class TaskRunState
    {
        IDLE,    // 等待启动命令
        RUNNING, // 任务进行中
    };

    TaskRunState state_ = TaskRunState::IDLE;
    ros::Time t0_;                  // 任务起点（耗时统计）
    uint32_t seq_ = 0;              // TaskState 序号
    double latest_area_m2_ = 0.0;   // 最近一次 navi 上报的已探索面积
    double max_duration_s_ = 600.0; // 看门狗：frontier 不收敛兜底

    ros::Subscriber cmd_sub_, navi_state_sub_;
    ros::Publisher navi_cmd_pub_, vel_pub_, state_pub_;
    ros::Timer watchdog_;
};

} // namespace fastbuild_task_nodelet
