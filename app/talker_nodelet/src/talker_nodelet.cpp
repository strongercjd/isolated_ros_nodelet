#include "talker_nodelet.h"

#include <sstream>

namespace talker_nodelet
{

TalkerNodelet::TalkerNodelet() : count_(0) {}

TalkerNodelet::~TalkerNodelet() {}

void TalkerNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  pub_ = nh.advertise<std_msgs::String>("chatter", 10);
  custom_pub_ = nh.advertise<custom_msgs::CustomData>("custom_chatter", 10);
  timer_ = nh.createTimer(ros::Duration(1.0), &TalkerNodelet::timerCallback, this);
  NODELET_INFO("TalkerNodelet initialized: std_msgs/String on /chatter, "
               "custom_msgs/CustomData on /custom_chatter");
}

void TalkerNodelet::timerCallback(const ros::TimerEvent&)
{
  std::stringstream ss;
  ss << "Hello from Nodelet " << count_;
  const std::string text = ss.str();

  std_msgs::String str_msg;
  str_msg.data = text;
  // NODELET_INFO("Publishing String: %s", str_msg.data.c_str());
  pub_.publish(str_msg);

  custom_msgs::CustomData custom_msg;
  custom_msg.text = text;
  custom_msg.seq = static_cast<uint16_t>(count_);
  custom_msg.value = static_cast<float>(count_) * 0.1f;
  // NODELET_INFO("Publishing CustomData: text=%s seq=%u value=%.2f",
  //              custom_msg.text.c_str(), custom_msg.seq, custom_msg.value);
  custom_pub_.publish(custom_msg);

  ++count_;
}

}  // namespace talker_nodelet

PLUGINLIB_EXPORT_CLASS(talker_nodelet::TalkerNodelet, nodelet::Nodelet)
