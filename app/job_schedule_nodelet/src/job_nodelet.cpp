#include "job_nodelet.h"

namespace job_schedule_nodelet
{

void JobNodelet::onInit()
{
    ros::NodeHandle pnh = getPrivateNodeHandle();

    // 话题写死全局名（不再依赖 plugins.json remap）：cmd=/job_node/cmd,
    //   task_state=/fastbuild_task/state, fastbuild_cmd=/fastbuild_task/cmd,
    //   task_result=/task/TaskResult
    cmd_sub_ = pnh.subscribe("/job_node/cmd", 5, &JobNodelet::jobCmdCallback, this);
    task_state_sub_ = pnh.subscribe("/fastbuild_task/state", 5, &JobNodelet::taskStateCallback, this);

    fastbuild_cmd_pub_ = pnh.advertise<custom_msgs::FastBuildCmd>("/fastbuild_task/cmd", 5);
    task_result_pub_ = pnh.advertise<custom_msgs::TaskResult>("/task/TaskResult", 5);

    NODELET_INFO("JobNodelet initialized (fastbuild chain: JobCmd -> FastBuildCmd -> TaskState -> TaskResult)");
}

void JobNodelet::jobCmdCallback(const custom_msgs::JobCmd::ConstPtr &msg)
{
    if (msg->command == "fastbuild")
    {
        if (state_ == JobRunState::WAITING)
        {
            NODELET_WARN("[job] fastbuild requested (seq=%u) ignored: already waiting for result", msg->seq);
            return;
        }
        state_ = JobRunState::WAITING;
        custom_msgs::FastBuildCmd cmd; // 语义对齐 pb start_fastbuild_req
        cmd.cmd = custom_msgs::FastBuildCmd::CMD_START;
        cmd.seq = ++seq_;
        cmd.type = "build_map";
        cmd.map_type = 1;
        cmd.stamp = ros::Time::now();
        fastbuild_cmd_pub_.publish(cmd);
        NODELET_INFO("[job] fastbuild requested (seq=%u), forwarded to fastbuild_task", seq_);
    }
    else if (msg->command == "cancel")
    {
        if (state_ != JobRunState::WAITING)
        {
            NODELET_WARN("[job] cancel (seq=%u) ignored: no fastbuild in flight", msg->seq);
            return;
        }
        custom_msgs::FastBuildCmd cmd;
        cmd.cmd = custom_msgs::FastBuildCmd::CMD_CANCEL;
        cmd.seq = ++seq_;
        cmd.type = "build_map";
        cmd.stamp = ros::Time::now();
        fastbuild_cmd_pub_.publish(cmd);
        NODELET_INFO("[job] fastbuild cancel requested (seq=%u)", seq_);
    }
    else
    {
        NODELET_WARN("[job] unknown command '%s' (seq=%u)", msg->command.c_str(), msg->seq);
    }
}

// 感知快建结束：对齐原 onBuildMapDone → setComplete(0) → 上报 task/TaskResult
void JobNodelet::taskStateCallback(const custom_msgs::TaskState::ConstPtr &msg)
{
    if (state_ != JobRunState::WAITING)
        return; // 非本节点发起的任务（或已处理过终态，latch 重放）——忽略
    if (msg->state != custom_msgs::TaskState::STATE_DONE)
        return;

    state_ = JobRunState::IDLE;
    const bool ok = (msg->finish_reason == custom_msgs::TaskState::REASON_IS_FINISH);

    NODELET_INFO("[job] fastbuild done: ret=%d area=%.1f m2 time=%.0f s", ok ? 0 : msg->finish_reason,
                 msg->area_m2, msg->cost_time_s);

    custom_msgs::TaskResult result; // 语义对齐 TaskResultReporter（event 1127 成功 / 1115 失败）
    result.task_id = custom_msgs::TaskResult::TASK_FASTBUILD;
    result.result = ok ? 0 : msg->finish_reason;
    result.event = ok ? custom_msgs::TaskResult::EVT_FAST_BUILDING_FINISHED
                      : custom_msgs::TaskResult::EVT_FAST_BUILDING_FAILED;
    result.area_m2 = msg->area_m2;
    result.cost_time_s = msg->cost_time_s;
    char summary[128];
    snprintf(summary, sizeof(summary), "fastbuild %s: area=%.1fm2 time=%.0fs",
             ok ? "finished" : "failed", msg->area_m2, msg->cost_time_s);
    result.message = summary;
    result.stamp = ros::Time::now();
    task_result_pub_.publish(result);
}

} // namespace job_schedule_nodelet

PLUGINLIB_EXPORT_CLASS(job_schedule_nodelet::JobNodelet, nodelet::Nodelet)
