#include "listener_nodelet.h"

namespace listener_nodelet
{

ListenerNodelet::ListenerNodelet() {}

ListenerNodelet::~ListenerNodelet() {}

void ListenerNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  sub_ = nh.subscribe("chatter", 10, &ListenerNodelet::callback, this);
  custom_sub_ = nh.subscribe("custom_chatter", 10, &ListenerNodelet::customCallback, this);
  NODELET_INFO("ListenerNodelet initialized: std_msgs/String on /chatter, "
               "custom_msgs/CustomData on /custom_chatter");
}

void ListenerNodelet::callback(const std_msgs::String::ConstPtr& msg)
{
  NODELET_INFO("Received String: %s", msg->data.c_str());
}

void ListenerNodelet::customCallback(const custom_msgs::CustomData::ConstPtr& msg)
{
  NODELET_INFO("Received CustomData: text=%s seq=%u value=%.2f",
               msg->text.c_str(), msg->seq, msg->value);
}

}  // namespace listener_nodelet

PLUGINLIB_EXPORT_CLASS(listener_nodelet::ListenerNodelet, nodelet::Nodelet)
