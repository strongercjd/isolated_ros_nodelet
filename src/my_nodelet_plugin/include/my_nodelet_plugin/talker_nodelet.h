#ifndef MY_NODELET_PLUGIN_TALKER_NODELET_H
#define MY_NODELET_PLUGIN_TALKER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace my_nodelet_plugin
{

/**
 * TalkerNodelet：在话题 chatter 上周期性发布 std_msgs/String。
 *
 * 由 nodelet 管理器 dlopen 本插件 .so 后实例化。真正的初始化必须放在
 * onInit() 里（此时 NodeHandle、名称空间才可用），不要放在构造函数中。
 */
class TalkerNodelet : public nodelet::Nodelet
{
public:
  TalkerNodelet();
  virtual ~TalkerNodelet();

private:
  /** 管理器完成加载后回调：创建 Publisher 和 1Hz 定时器。 */
  virtual void onInit();
  /** 定时器回调：组装 “Hello from Nodelet N” 并 publish。 */
  void timerCallback(const ros::TimerEvent& event);

  ros::Publisher pub_;  /**< 发布到 chatter */
  ros::Timer timer_;    /**< 1 秒周期 */
  int count_;           /**< 消息序号，从 0 递增 */
};

}  // namespace my_nodelet_plugin

#endif
