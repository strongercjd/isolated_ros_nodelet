// flat_sim —— SLAM 话题订阅端（与 RosBridge 的发布端职责分开）
//
//   订阅：/slam2d/map           nav_msgs/OccupancyGrid   （latch，1Hz 节流）
//         /slam2d/input_cloud   sensor_msgs/PointCloud2   （红）
//         /slam2d/mapping_cloud sensor_msgs/PointCloud2   （蓝）
//         /slam2d/pose          geometry_msgs/PoseStamped（蓝箭头）
//   发布：/slam2d/reset         std_msgs/Empty            （GUI 按 r 复位时联动）
//
// 回调经全局队列派发（RosBridge::spinOnce 同队列），与主循环同线程；
// snapshot() 仍加锁交换，防未来把订阅挪到独立 spin 线程。
#pragma once

#include <mutex>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include "ros/SlamSnapshot.h"

namespace flat_sim {

class SlamListener {
 public:
  SlamListener();
  ~SlamListener();

  // 取一致快照（点云拷贝 ~几 KB；地图 shared_ptr 共享不拷贝）
  SlamSnapshot snapshot() const;

  // 发布 /slam2d/reset（GUI 复位机器人时联动，避免 odom 跳变打爆 SLAM）
  void publishReset();

 private:
  static bool parseCloud(const sensor_msgs::PointCloud2::ConstPtr& msg,
                         std::vector<float>& xy);

  void cbMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void cbInput(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cbMapping(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  mutable std::mutex mtx_;
  SlamSnapshot snap_;

  ros::NodeHandle nh_;
  ros::Subscriber mapSub_, inputSub_, mappingSub_, poseSub_;
  ros::Publisher resetPub_;
};

}  // namespace flat_sim
