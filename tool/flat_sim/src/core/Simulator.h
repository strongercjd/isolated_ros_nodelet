// flat_sim —— 仿真核心：差速运动积分 + 静态障碍碰撞 + 2D 激光射线
// 不依赖 ROS / GUI：可独立 headless 运行。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/World.h"

namespace flat_sim {

class Simulator {
 public:
  // 一帧激光扫描结果（世界系，GUI / ROS 桥接直接消费）
  struct Scan {
    Pose2 origin;               // 激光原点位姿（世界系；yaw = 光束基准角）
    double angleStart = 0.0;    // 第 1 束光线世界角（弧度）
    double angleInc = 0.0;      // 相邻光束角增量（弧度）
    double fov = 0.0;           // 视场角（弧度）
    int samples = 0;
    double rangeMin = 0.0;
    double rangeMax = 10.0;
    std::vector<float> ranges;  // 米；无回波 = inf
  };

  struct RobotState {
    std::string name;
    std::string color;
    double radius = 0.1;
    Pose2 pose;         // 当前位姿（世界系）
    Pose2 initialPose;  // 初始位姿（复位用）
    double cmdV = 0.0;  // 最近一次 cmd_vel 线速度
    double cmdW = 0.0;  // 最近一次 cmd_vel 角速度
    double curV = 0.0;  // 本步实际执行的线速度（撞墙被挡时为 0）
    double curW = 0.0;
    std::vector<LaserDesc> laserDescs;  // 静态配置（与 scans 下标对齐）
    std::vector<Scan> scans;
  };

  explicit Simulator(const WorldDesc& world);

  const WorldDesc& world() const { return world_; }
  const std::vector<RobotState>& robots() const { return robots_; }
  RobotState* robot(const std::string& name);

  // 设置速度指令（m/s、rad/s）；未知机器人名忽略并告警
  void setCmd(const std::string& name, double v, double w);
  // 推进一个仿真步：运动积分 → 碰撞检测（分轴回退，允许贴墙滑动）→ 激光
  void step(double dt);
  // 所有机器人回到初始位姿并清零速度
  void reset();

  // 编辑墙占据层：与 GUI 共用同一 shared_ptr 实例，编辑即生效（挡激光/挡运动）。
  void setEditGrid(std::shared_ptr<GridLayer> g) { editGrid_ = std::move(g); }
  const GridLayer* editGrid() const { return editGrid_ ? editGrid_.get() : nullptr; }
  // 网格被改后重算各机器人当前一帧激光（GUI 暂停期间编辑完调用）
  void recomputeScans() {
    for (RobotState& r : robots_) updateLasers(r);
  }

  uint64_t steps() const { return steps_; }
  double simTime() const { return simTime_; }

 private:
  bool blocked(const RobotState& self, Vec2 center) const;
  double castRay(Vec2 o, Vec2 d, const RobotState* self) const;
  // 编辑格层 DDA 求交：射线最先进入"占用格"的 t；无命中 / 出界返回 -1
  double gridRay(Vec2 o, Vec2 d) const;
  void updateLasers(RobotState& r);

  WorldDesc world_;
  std::shared_ptr<GridLayer> editGrid_;  // 可选：用户编辑墙（与 GUI 共享）
  std::vector<RobotState> robots_;
  uint64_t steps_ = 0;
  double simTime_ = 0.0;
};

}  // namespace flat_sim
