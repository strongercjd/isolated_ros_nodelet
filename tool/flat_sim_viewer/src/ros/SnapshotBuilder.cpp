// flat_sim_viewer —— SnapshotBuilder 实现（逻辑迁自 SlamListener.cpp）
#include "ros/SnapshotBuilder.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <utility>

namespace flat_sim_viewer {
namespace snapshot_builder {

bool parseCloudXY(const sensor_msgs::PointCloud2& msg, std::vector<float>& xy) {
  xy.clear();
  if (msg.data.empty()) return false;
  uint32_t offX = 0, offY = 4;
  bool hasX = false, hasY = false;
  for (const sensor_msgs::PointField& f : msg.fields) {
    if (f.name == "x") { offX = f.offset; hasX = true; }
    if (f.name == "y") { offY = f.offset; hasY = true; }
  }
  if (!hasX || !hasY) return false;

  const uint32_t n = msg.width * msg.height;
  xy.reserve(n * 2);
  const uint8_t* base = msg.data.data();
  for (uint32_t i = 0; i < n; ++i) {
    const uint8_t* p = base + (size_t)i * msg.point_step;
    float x, y;
    std::memcpy(&x, p + offX, sizeof(float));
    std::memcpy(&y, p + offY, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y)) continue;
    xy.push_back(x);
    xy.push_back(y);
  }
  return !xy.empty();
}

void applyMap(SlamSnapshot& snap, const nav_msgs::OccupancyGrid& msg, uint32_t seq) {
  snap.map.data = std::make_shared<std::vector<int8_t>>(msg.data);
  snap.map.width = (int)msg.info.width;
  snap.map.height = (int)msg.info.height;
  snap.map.resolution = msg.info.resolution;
  snap.map.originX = msg.info.origin.position.x;
  snap.map.originY = msg.info.origin.position.y;
  snap.map.seq = seq;
}

void applyPose(SlamSnapshot& snap, const geometry_msgs::PoseStamped& msg) {
  snap.hasPose = true;
  snap.poseX = msg.pose.position.x;
  snap.poseY = msg.pose.position.y;
  snap.poseYaw = 2.0 * std::atan2(msg.pose.orientation.z, msg.pose.orientation.w);
}

void applyInputCloud(SlamSnapshot& snap, const sensor_msgs::PointCloud2& msg) {
  snap.hasInput = parseCloudXY(msg, snap.inputXY);
  if (!snap.hasInput) snap.inputXY.clear();
}

void applyMappingCloud(SlamSnapshot& snap, const sensor_msgs::PointCloud2& msg) {
  snap.hasMapping = parseCloudXY(msg, snap.mappingXY);
  if (!snap.hasMapping) snap.mappingXY.clear();
}

void applyDecision(SlamSnapshot& snap, const custom_msgs::FastBuildDecision& msg) {
  auto& d = snap.decision;
  d.has = true;
  d.candidates.clear();
  d.candidates.reserve(msg.candidates.size());
  for (size_t i = 0; i < msg.candidates.size(); ++i) {
    SlamSnapshot::DecisionOverlay::Cand c;
    c.x = msg.candidates[i].x;
    c.y = msg.candidates[i].y;
    c.size = i < msg.candidate_size.size() ? msg.candidate_size[i] : 0;
    c.via = i < msg.candidate_via.size() ? msg.candidate_via[i] : 0;
    d.candidates.push_back(c);
  }
  d.hasSelected = msg.has_selected != 0;
  d.selX = msg.selected_x;
  d.selY = msg.selected_y;
  d.blacklist.clear();
  d.blacklist.reserve(msg.blacklist.size());
  for (const auto& b : msg.blacklist) d.blacklist.emplace_back(b.x, b.y);
  d.areaM2 = msg.area_m2;
  d.taskState = msg.task_state;
}

}  // namespace snapshot_builder
}  // namespace flat_sim_viewer
