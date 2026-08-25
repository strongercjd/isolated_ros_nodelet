#include "ros/SlamListener.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <utility>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

namespace flat_sim {

SlamListener::SlamListener() {
  // queue=1：可视化只要最新一帧；map 是 latch 话题，订阅建立时即收到最后一版
  mapSub_ = nh_.subscribe("/slam2d/map", 1, &SlamListener::cbMap, this);
  inputSub_ = nh_.subscribe("/slam2d/input_cloud", 1, &SlamListener::cbInput, this);
  mappingSub_ = nh_.subscribe("/slam2d/mapping_cloud", 1, &SlamListener::cbMapping, this);
  poseSub_ = nh_.subscribe("/slam2d/pose", 1, &SlamListener::cbPose, this);
  resetPub_ = nh_.advertise<std_msgs::Empty>("/slam2d/reset", 1);
}

SlamListener::~SlamListener() = default;

SlamSnapshot SlamListener::snapshot() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return snap_;  // 地图 data 是 shared_ptr，深拷贝只有点云（几 KB）
}

void SlamListener::publishReset() {
  std_msgs::Empty msg;
  resetPub_.publish(msg);
}

// PointCloud2 → 平铺 x,y。按 fields 查 offset（发送端是本项目 slam2d，恒为 FLOAT32）。
bool SlamListener::parseCloud(const sensor_msgs::PointCloud2::ConstPtr& msg,
                              std::vector<float>& xy) {
  xy.clear();
  if (!msg || msg->data.empty()) return false;
  uint32_t offX = 0, offY = 4;
  bool hasX = false, hasY = false;
  for (const sensor_msgs::PointField& f : msg->fields) {
    if (f.name == "x") { offX = f.offset; hasX = true; }
    if (f.name == "y") { offY = f.offset; hasY = true; }
  }
  if (!hasX || !hasY) return false;

  const uint32_t n = msg->width * msg->height;
  xy.reserve(n * 2);
  const uint8_t* base = msg->data.data();
  for (uint32_t i = 0; i < n; ++i) {
    const uint8_t* p = base + (size_t)i * msg->point_step;
    float x, y;
    std::memcpy(&x, p + offX, sizeof(float));
    std::memcpy(&y, p + offY, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y)) continue;
    xy.push_back(x);
    xy.push_back(y);
  }
  return !xy.empty();
}

void SlamListener::cbMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // 4MB 拷贝在锁外完成，锁内只交换指针
  auto data = std::make_shared<std::vector<int8_t>>(msg->data);
  std::lock_guard<std::mutex> lk(mtx_);
  snap_.map.data = std::move(data);
  snap_.map.width = (int)msg->info.width;
  snap_.map.height = (int)msg->info.height;
  snap_.map.resolution = msg->info.resolution;
  snap_.map.originX = msg->info.origin.position.x;
  snap_.map.originY = msg->info.origin.position.y;
  ++snap_.map.seq;  // SlamView 据此判断是否重填纹理
}

void SlamListener::cbInput(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::vector<float> xy;
  const bool ok = parseCloud(msg, xy);
  std::lock_guard<std::mutex> lk(mtx_);
  snap_.inputXY = std::move(xy);
  snap_.hasInput = ok;
}

void SlamListener::cbMapping(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::vector<float> xy;
  const bool ok = parseCloud(msg, xy);
  std::lock_guard<std::mutex> lk(mtx_);
  snap_.mappingXY = std::move(xy);
  snap_.hasMapping = ok;
}

void SlamListener::cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // slam2d 只发绕 z 的四元数（x=y=0）→ yaw = 2*atan2(z, w)
  const double yaw = 2.0 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  std::lock_guard<std::mutex> lk(mtx_);
  snap_.hasPose = true;
  snap_.poseX = msg->pose.position.x;
  snap_.poseY = msg->pose.position.y;
  snap_.poseYaw = yaw;
}

}  // namespace flat_sim
