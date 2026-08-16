/**
 * TalkerNodelet 实现。
 *
 * PLUGINLIB_EXPORT_CLASS 把本类登记为 nodelet::Nodelet 的可加载插件，
 * 名称必须与 nodelet_plugins.xml 中的 type 字段一致。
 */
#include "my_nodelet_plugin/talker_nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <sstream>

namespace my_nodelet_plugin
{

TalkerNodelet::TalkerNodelet() : count_(0) {}

TalkerNodelet::~TalkerNodelet() {}

void TalkerNodelet::onInit()
{
  // getNodeHandle() 使用 nodelet 的私有/公共名称空间，与独立节点行为一致。
  ros::NodeHandle& nh = getNodeHandle();
  pub_ = nh.advertise<std_msgs::String>("chatter", 10);
  timer_ = nh.createTimer(ros::Duration(1.0), &TalkerNodelet::timerCallback, this);
  NODELET_INFO("TalkerNodelet initialized, publishing on /chatter");
}

void TalkerNodelet::timerCallback(const ros::TimerEvent&)
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "Hello from Nodelet " << count_;
  msg.data = ss.str();
  ++count_;
  NODELET_INFO("Publishing: %s", msg.data.c_str());
  pub_.publish(msg);
}

}  // namespace my_nodelet_plugin

PLUGINLIB_EXPORT_CLASS(my_nodelet_plugin::TalkerNodelet, nodelet::Nodelet)
