// flat_sim_viewer —— 消息 → SlamSnapshot 填充（无状态、无锁）
//
// 从 SlamListener 的回调填充体抽出，供两个数据源共用：
//   实时：SlamListener 回调（ros 层）
//   回放：replay/BagPlayer 的事件应用（replay 层）
// 调用方自行决定加锁时机；所有函数均为"最新值"语义，重复应用无害。
#pragma once

#include <vector>

#include <custom_msgs/FastBuildDecision.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include "ros/SlamSnapshot.h"

namespace flat_sim_viewer {
namespace snapshot_builder {

// PointCloud2 → 平铺 x,y（按 fields 查 offset；发送端是本项目 slam2d，恒为 FLOAT32）。
// 空 / 缺 x,y 字段 / 无有限点 → false。
bool parseCloudXY(const sensor_msgs::PointCloud2& msg, std::vector<float>& xy);

// 地图：拷贝 data（4MB 级）后覆盖 snap.map 各字段。
// seq 由调用方传入并保证递增 —— SlamView 靠 snap.map.seq 变化重建 QImage，
// 回放回退时必须传更大的高水位，否则出现残影。
void applyMap(SlamSnapshot& snap, const nav_msgs::OccupancyGrid& msg, uint32_t seq);

// slam2d 只发绕 z 的四元数（x=y=0）→ yaw = 2*atan2(z, w)
void applyPose(SlamSnapshot& snap, const geometry_msgs::PoseStamped& msg);

void applyInputCloud(SlamSnapshot& snap, const sensor_msgs::PointCloud2& msg);
void applyMappingCloud(SlamSnapshot& snap, const sensor_msgs::PointCloud2& msg);

// fastbuild 决策覆盖层：从决策消息填充 candidates / selected / blacklist / area / task_state
void applyDecision(SlamSnapshot& snap, const custom_msgs::FastBuildDecision& msg);

}  // namespace snapshot_builder
}  // namespace flat_sim_viewer
