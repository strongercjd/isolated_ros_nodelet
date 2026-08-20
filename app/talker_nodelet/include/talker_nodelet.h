#ifndef TALKER_NODELET_TALKER_NODELET_H
#define TALKER_NODELET_TALKER_NODELET_H

#include <custom_ros_nodelet/custom_ros_nodelet.h>
#include <custom_msgs/CustomData.h>
#include <std_msgs/String.h>

namespace talker_nodelet
{

class TalkerNodelet : public nodelet::Nodelet
{
public:
  TalkerNodelet();
  virtual ~TalkerNodelet();

private:
  virtual void onInit();
  void timerCallback(const ros::TimerEvent& event);

  ros::Publisher pub_;
  ros::Publisher custom_pub_;
  ros::Timer timer_;
  int count_;
};

}  // namespace talker_nodelet

#endif
