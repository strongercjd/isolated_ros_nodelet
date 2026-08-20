#ifndef LISTENER_NODELET_LISTENER_NODELET_H
#define LISTENER_NODELET_LISTENER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
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

  ros::Subscriber sub_;
};

}  // namespace listener_nodelet

#endif
