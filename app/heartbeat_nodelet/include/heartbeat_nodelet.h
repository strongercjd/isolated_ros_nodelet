#ifndef HEARTBEAT_NODELET_HEARTBEAT_NODELET_H
#define HEARTBEAT_NODELET_HEARTBEAT_NODELET_H

#include <custom_ros_nodelet/custom_ros_nodelet.h>
#include <std_msgs/Empty.h>

namespace heartbeat_nodelet
{

// 控制侧心跳：每 100ms 发一次 /heartbeat。
// flat_sim 若连续 500ms 收不到则强制 0 速停机。
class HeartbeatNodelet : public nodelet::Nodelet
{
public:
  HeartbeatNodelet();
  virtual ~HeartbeatNodelet();

private:
  virtual void onInit();
  void timerCallback(const ros::TimerEvent& event);

  ros::Publisher heartbeat_pub_;
  ros::Timer timer_;
  uint32_t count_;
};

}  // namespace heartbeat_nodelet

#endif
