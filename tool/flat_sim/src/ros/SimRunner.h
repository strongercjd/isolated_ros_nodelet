// flat_sim —— 仿真运行时：单步推进 + ROS 延迟初始化
//
// 从 main.cpp 抽出，保证两条运行路径（headless 纯循环 / GUI 定时器步进）
// 的单步顺序完全一致，唯一来源在 stepOnce()：
//   spinOnce（收心跳/cmd_vel）→ 心跳看门狗 → sim.step
//   →（未连 ROS 时每 100 步重探 master）→ publish
//
// ROS 延迟初始化：master 不可达时不碰 ros::init（否则 roscpp 后台线程会
// 无限重试注册刷屏）；tryStartRos() 用裸 socket 探测，可达后才初始化。
#pragma once

#include <memory>

#include "core/Simulator.h"
#include "core/World.h"
#include "ros/RosBridge.h"

namespace flat_sim {

class SimRunner {
 public:
  // argv 由 main 持有；ros::init 需要原始 argc/argv（解析 ~sim_time 等）
  SimRunner(WorldDesc world, int argc, char** argv);

  // master 可达才 ros::init + 建 RosBridge；幂等，返回当前是否已连
  bool tryStartRos();
  // 单步；返回 false 表示请求退出（Ctrl+C 后 ros::ok() 变 false）
  bool stepOnce();
  // 复位机器人（r 键）；联动发布 /slam2d/reset
  void reset();

  bool rosRunning() const { return rosRunning_; }
  const WorldDesc& world() const { return world_; }
  const Simulator& sim() const { return sim_; }
  Simulator& simMutable() { return sim_; }  // 仅供 GUI poll（r 键复位）
  double dt() const { return dt_; }

  // 编辑墙格层（Simulator 与 GUI 共享同一实例；改动即生效）
  std::shared_ptr<GridLayer> editGrid() const { return editGrid_; }
  // 整体替换格层实例（改格子边长/重划分时用；旧墙按格心迁移由调用方完成）
  void replaceEditGrid(std::shared_ptr<GridLayer> g) {
    editGrid_ = std::move(g);
    sim_.setEditGrid(editGrid_);
  }

  // 热切换世界（GUI「文件 → 打开…」）：以另一 .fworld 重建 sim_ 并替换编辑墙层。
  // ROS 已连时重建话题桥（不重复 ros::init）；返回新格层实例供 GUI 绑定到视图。
  // 调用前 GUI 应停止步进定时器；调用后需把返回格层 setEditGrid 到视图。
  std::shared_ptr<GridLayer> loadWorld(WorldDesc newWorld);

 private:
  // 裸 socket 探测 ROS_MASTER_URI 端口（ros::master::check 要求已 ros::init）
  static bool masterReachable();

  WorldDesc world_;              // 仅供构造 Simulator / 查询步长
  double dt_;
  Simulator sim_;                // bridge_ 持其引用，须声明在 bridge_ 之前
  std::shared_ptr<GridLayer> editGrid_;  // 编辑墙层（默认取自 world_.editGrid）
  std::unique_ptr<RosBridge> bridge_;
  bool rosRunning_ = false;
  bool useSimTime_ = false;      // ~sim_time 解析值；重建桥时复用（不可重复 ros::init）
  uint64_t spinCount_ = 0;       // 未连 ROS 时每 100 步重探一次 master
  int argc_;
  char** argv_;
};

}  // namespace flat_sim
