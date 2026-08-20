#include "listener_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace listener_nodelet
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

}  // namespace listener_nodelet

PLUGINLIB_EXPORT_CLASS(listener_nodelet::ListenerNodelet, nodelet::Nodelet)
