// flat_sim_viewer —— SLAM 话题订阅端（纯被动，不发任何话题）
//
// 拷贝自 tool/flat_sim/src/ros/SlamListener.h（2026-08 版，拆分时迁出），
// 改动：命名空间、删去 /slam2d/reset 发布（复位联动留在 flat_sim 侧）。
//
//   订阅：/slam2d/map           nav_msgs/OccupancyGrid   （latch，1Hz 节流）
//         /slam2d/input_cloud   sensor_msgs/PointCloud2   （红）
//         /slam2d/mapping_cloud sensor_msgs/PointCloud2   （蓝）
//         /slam2d/pose          geometry_msgs/PoseStamped（蓝箭头）
//
// 回调经全局队列派发（ViewerWindow 的 tickTimer 里 spinOnce），与主循环同线程；
// snapshot() 仍加锁交换，防未来把订阅挪到独立 spin 线程。
#pragma once

#include <mutex>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "ros/SlamSnapshot.h"

namespace flat_sim_viewer {

class SlamListener {
 public:
  SlamListener();
  ~SlamListener();

  // 取一致快照（点云拷贝 ~几 KB；地图 shared_ptr 共享不拷贝）
  SlamSnapshot snapshot() const;

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
};

}  // namespace flat_sim_viewer
