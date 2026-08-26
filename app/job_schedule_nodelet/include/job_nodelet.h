#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <custom_msgs/FastBuildCmd.h>
#include <custom_msgs/JobCmd.h>
#include <custom_msgs/TaskResult.h>
#include <custom_msgs/TaskState.h>

namespace job_schedule_nodelet
{

class JobNodelet : public nodelet::Nodelet
{
public:
    JobNodelet() = default;
    ~JobNodelet() override = default;

private:
    virtual void onInit();

    void jobCmdCallback(const custom_msgs::JobCmd::ConstPtr &msg);
    void taskStateCallback(const custom_msgs::TaskState::ConstPtr &msg);

    enum class JobRunState
    {
        IDLE,    // 等待快建命令
        WAITING, // 已下发 fastbuild，等待终态
    };

    JobRunState state_ = JobRunState::IDLE;
    uint32_t seq_ = 0; // 请求序号（对账用）

    ros::Subscriber cmd_sub_, task_state_sub_;
    ros::Publisher fastbuild_cmd_pub_, task_result_pub_;
};

} // namespace job_schedule_nodelet
