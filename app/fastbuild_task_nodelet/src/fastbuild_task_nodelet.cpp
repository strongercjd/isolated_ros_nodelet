#include "fastbuild_task_nodelet.h"

namespace fastbuild_task_nodelet
{
void FastbuildTaskNodelet::onInit()
{
    ros::NodeHandle pnh = getPrivateNodeHandle();

    pnh.param("max_duration_s", max_duration_s_, 600.0);//600s 超时时间 ros参数

    cmd_sub_ = pnh.subscribe("/fastbuild_task/cmd", 5, &FastbuildTaskNodelet::taskCmdCallback, this);
    navi_state_sub_ = pnh.subscribe("/base_navi/state", 5, &FastbuildTaskNodelet::naviStateCallback, this);

    navi_cmd_pub_ = pnh.advertise<custom_msgs::NaviCmd>("/base_navi/cmd", 5);
    vel_pub_ = pnh.advertise<geometry_msgs::Twist>("/mycar/cmd_vel", 5);
    state_pub_ = pnh.advertise<custom_msgs::TaskState>("/fastbuild_task/state", 5, true /*latch：迟订阅者也能拿到终态*/);

    watchdog_ = pnh.createTimer(ros::Duration(5.0), &FastbuildTaskNodelet::watchdogTimer, this);//5s看门狗
    NODELET_INFO("FastbuildTaskNodelet initialized: max_duration_s=%.0f", max_duration_s_);
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
        NODELET_INFO("task START (seq=%u type=%s map_type=%d from_charger=%d)", msg->seq,
                     msg->type.c_str(), msg->map_type, msg->start_from_charger ? 1 : 0);
        publishTaskState(custom_msgs::TaskState::STATE_RUNNING, 255, 0.0, 0.0);//发布快建任务状态
        sendNaviCmd(custom_msgs::NaviCmd::CMD_START);//启动导航
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
 * @brief 处理导航状态消息的回调函数
 * @param msg 接收到的导航状态消息指针
 */
void FastbuildTaskNodelet::naviStateCallback(const custom_msgs::NaviState::ConstPtr &msg)
{
    latest_area_m2_ = msg->explored_area_m2; // 缓存最新面积，收尾上报用
    if (state_ != TaskRunState::RUNNING)
        return;
    if (msg->state == custom_msgs::NaviState::FINISH)
        finishTask(custom_msgs::TaskState::REASON_IS_FINISH, "navi reported FINISH");
    else if (msg->state == custom_msgs::NaviState::ABORT)
        finishTask(custom_msgs::TaskState::REASON_FAIL, "navi reported ABORT");//
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

} // namespace fastbuild_task_nodelet

PLUGINLIB_EXPORT_CLASS(fastbuild_task_nodelet::FastbuildTaskNodelet, nodelet::Nodelet)
