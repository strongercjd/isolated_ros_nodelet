// 拷贝自 tool/flat_sim/src/ros/SlamListener.cpp（2026-08 版），改动：命名空间、
// 填充逻辑移至 ros/SnapshotBuilder（与 BagPlayer 共用，回放端复用同一套解析）。
#include "ros/SlamListener.h"

#include "ros/SnapshotBuilder.h"

namespace flat_sim_viewer {

SlamListener::SlamListener() {
  // queue=1：可视化只要最新一帧；map 是 latch 话题，订阅建立时即收到最后一版
  mapSub_ = nh_.subscribe("/slam2d/map", 1, &SlamListener::cbMap, this);
  inputSub_ = nh_.subscribe("/slam2d/input_cloud", 1, &SlamListener::cbInput, this);
  mappingSub_ = nh_.subscribe("/slam2d/mapping_cloud", 1, &SlamListener::cbMapping, this);
  poseSub_ = nh_.subscribe("/slam2d/pose", 1, &SlamListener::cbPose, this);
  decisionSub_ = nh_.subscribe("/fastbuild_task/decision", 1, &SlamListener::cbDecision, this);
}

SlamListener::~SlamListener() = default;

SlamSnapshot SlamListener::snapshot() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return snap_;  // 地图 data 是 shared_ptr，深拷贝只有点云（几 KB）
}

void SlamListener::cbMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  snapshot_builder::applyMap(snap_, *msg, ++mapSeq_);  // seq 递增，SlamView 据此重填图像
}

void SlamListener::cbInput(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  snapshot_builder::applyInputCloud(snap_, *msg);
}

void SlamListener::cbMapping(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  snapshot_builder::applyMappingCloud(snap_, *msg);
}

void SlamListener::cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  snapshot_builder::applyPose(snap_, *msg);
}

void SlamListener::cbDecision(const custom_msgs::FastBuildDecision::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  snapshot_builder::applyDecision(snap_, *msg);
}

}  // namespace flat_sim_viewer
