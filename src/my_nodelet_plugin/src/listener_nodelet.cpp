/**
 * ListenerNodelet 实现。
 *
 * 订阅相对话题名 chatter（在 manager 的名称空间下通常解析为 /chatter）。
 */
#include "my_nodelet_plugin/listener_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace my_nodelet_plugin
{

ListenerNodelet::ListenerNodelet() {}

ListenerNodelet::~ListenerNodelet() {}

void ListenerNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  sub_ = nh.subscribe("chatter", 10, &ListenerNodelet::callback, this);
  NODELET_INFO("ListenerNodelet initialized, subscribed to /chatter");
}

void ListenerNodelet::callback(const std_msgs::String::ConstPtr& msg)
{
  NODELET_INFO("Received: %s", msg->data.c_str());
}

}  // namespace my_nodelet_plugin

PLUGINLIB_EXPORT_CLASS(my_nodelet_plugin::ListenerNodelet, nodelet::Nodelet)
