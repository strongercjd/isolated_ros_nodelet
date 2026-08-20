#ifndef LISTENER_NODELET_LISTENER_NODELET_H
#define LISTENER_NODELET_LISTENER_NODELET_H

#include <custom_ros_nodelet/custom_ros_nodelet.h>
#include <custom_msgs/CustomData.h>
#include <std_msgs/String.h>

namespace listener_nodelet
{

class ListenerNodelet : public nodelet::Nodelet
{
public:
  ListenerNodelet();
  virtual ~ListenerNodelet();

private:
  virtual void onInit();
  void callback(const std_msgs::String::ConstPtr& msg);
  void customCallback(const custom_msgs::CustomData::ConstPtr& msg);

  ros::Subscriber sub_;
  ros::Subscriber custom_sub_;
};

}  // namespace listener_nodelet

#endif
