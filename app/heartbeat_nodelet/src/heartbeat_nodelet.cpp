#include "heartbeat_nodelet.h"

namespace heartbeat_nodelet
{

HeartbeatNodelet::HeartbeatNodelet() : count_(0) {}

HeartbeatNodelet::~HeartbeatNodelet() {}

void HeartbeatNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  // 绝对话题，便于 flat_sim 等与 nodelet 命名空间无关的节点订阅
  heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/heartbeat", 1);
  timer_ = nh.createTimer(ros::Duration(0.1), &HeartbeatNodelet::timerCallback, this);
  NODELET_INFO("HeartbeatNodelet: /heartbeat every 100ms");
}

void HeartbeatNodelet::timerCallback(const ros::TimerEvent&)
{
  std_msgs::Empty msg;
  heartbeat_pub_.publish(msg);
  ++count_;
}

}  // namespace heartbeat_nodelet

PLUGINLIB_EXPORT_CLASS(heartbeat_nodelet::HeartbeatNodelet, nodelet::Nodelet)
