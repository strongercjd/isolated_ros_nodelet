#ifndef MY_NODELET_PLUGIN_LISTENER_NODELET_H
#define MY_NODELET_PLUGIN_LISTENER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace my_nodelet_plugin
{

/**
 * ListenerNodelet：订阅 chatter，把收到的字符串打到控制台。
 *
 * 与 TalkerNodelet 加载到同一个 manager 时，二者在同一进程内通信，
 * 不经过网络序列化（仍走 ROS 话题接口）。
 */
class ListenerNodelet : public nodelet::Nodelet
{
public:
  ListenerNodelet();
  virtual ~ListenerNodelet();

private:
  /** 管理器完成加载后回调：订阅 chatter。 */
  virtual void onInit();
  /** 收到消息后用 NODELET_INFO 打印内容。 */
  void callback(const std_msgs::String::ConstPtr& msg);

  ros::Subscriber sub_;
};

}  // namespace my_nodelet_plugin

#endif
