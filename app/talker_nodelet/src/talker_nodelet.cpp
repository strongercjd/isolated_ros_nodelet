#include "talker_nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <sstream>

namespace talker_nodelet
{

TalkerNodelet::TalkerNodelet() : count_(0) {}

TalkerNodelet::~TalkerNodelet() {}

void TalkerNodelet::onInit()
{
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

}  // namespace talker_nodelet

PLUGINLIB_EXPORT_CLASS(talker_nodelet::TalkerNodelet, nodelet::Nodelet)
