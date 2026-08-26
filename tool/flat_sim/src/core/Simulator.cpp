// flat_sim —— 仿真核心实现
#include "core/Simulator.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace flat_sim {

Simulator::Simulator(const WorldDesc& world) : world_(world) {
  robots_.reserve(world_.robots.size());
  for (const RobotDesc& d : world_.robots) {
    RobotState r;
    r.name = d.name;
    r.color = d.color;
    r.radius = d.radius;
    r.pose = d.pose;
    r.initialPose = d.pose;
    r.laserDescs = d.lasers;
    r.scans.resize(d.lasers.size());
    robots_.push_back(std::move(r));
  }
  // 构造完成先算一帧激光，保证订阅者 / GUI 第一眼就有数据
  for (RobotState& r : robots_) updateLasers(r);
  // 初始位姿与障碍重叠的机器人会被碰撞检测永远挡住，加载时提示（不阻断）
  for (const RobotState& r : robots_) {
    if (blocked(r, r.pose.p))
      std::fprintf(stderr,
                   "[flat_sim][警告] 机器人 \"%s\" 初始位姿 (%.2f, %.2f) 与障碍重叠，将无法移动；"
                   "请调整 world 里该机器人的 pose\n",
                   r.name.c_str(), r.pose.p.x, r.pose.p.y);
  }
}

Simulator::RobotState* Simulator::robot(const std::string& name) {
  for (RobotState& r : robots_)
    if (r.name == name) return &r;
  return nullptr;
}

void Simulator::setCmd(const std::string& name, double v, double w) {
  RobotState* r = robot(name);
  if (!r) {
    std::fprintf(stderr, "[flat_sim][警告] cmd_vel 指向未知机器人 \"%s\"，忽略\n", name.c_str());
    return;
  }
  r->cmdV = v;
  r->cmdW = w;
}

void Simulator::reset() {
  for (RobotState& r : robots_) {
    r.pose = r.initialPose;
    r.cmdV = r.cmdW = r.curV = r.curW = 0.0;
    updateLasers(r);
  }
}

bool Simulator::blocked(const RobotState& self, Vec2 c) const {
  for (const BoxObstacle& b : world_.boxes) {
    if (circleBoxOverlap(c, self.radius, b.pose.p, b.pose.yaw, b.len / 2, b.wid / 2)) return true;
  }
  for (const CircleObstacle& o : world_.circles) {
    const double rr = o.radius + self.radius;
    const Vec2 d = c - o.center;
    if (dot(d, d) <= rr * rr) return true;
  }
  for (const RobotState& other : robots_) {
    if (&other == &self) continue;
    const double rr = other.radius + self.radius;
    const Vec2 d = c - other.pose.p;
    if (dot(d, d) <= rr * rr) return true;
  }
  return false;
}

double Simulator::castRay(Vec2 o, Vec2 d, const RobotState* self) const {
  double best = -1.0;
  auto keep = [&best](double t) {
    if (t >= 0.0 && (best < 0.0 || t < best)) best = t;
  };
  for (const BoxObstacle& b : world_.boxes)
    keep(rayBox(o, d, b.pose.p, b.pose.yaw, b.len / 2, b.wid / 2));
  for (const CircleObstacle& c : world_.circles)
    keep(rayCircle(o, d, c.center, c.radius));
  // 激光也能照到其他机器人（多机场景）；self 除外
  for (const RobotState& other : robots_)
    if (&other != self) keep(rayCircle(o, d, other.pose.p, other.radius));
  return best;
}

void Simulator::updateLasers(RobotState& r) {
  r.scans.resize(r.laserDescs.size());
  for (size_t i = 0; i < r.laserDescs.size(); ++i) {
    const LaserDesc& ld = r.laserDescs[i];
    Scan& s = r.scans[i];
    s.origin = compose(r.pose, ld.mount);      // 世界系激光位姿
    s.fov = ld.fov;
    s.samples = ld.samples;
    s.rangeMin = ld.rangeMin;
    s.rangeMax = ld.rangeMax;
    s.angleInc = ld.samples > 1 ? ld.fov / (ld.samples - 1) : 0.0;
    // 单线时取视场中心；多线从 -fov/2 起
    s.angleStart = s.origin.yaw - ld.fov / 2.0 + (ld.samples == 1 ? ld.fov / 2.0 : 0.0);
    // 无回波 = inf（ROS 惯例，REP 117）：消费方按 !isfinite / >range_max 剔除；
    // 填 rangeMax 会被下游当成量程边界处的真实障碍物。
    s.ranges.assign((size_t)std::max(1, ld.samples), std::numeric_limits<float>::infinity());
    for (int k = 0; k < ld.samples; ++k) {
      const double ang = s.angleStart + s.angleInc * (double)k;
      const double t = castRay(s.origin.p, Vec2{std::cos(ang), std::sin(ang)}, &r);
      if (t >= ld.rangeMin && t <= ld.rangeMax) s.ranges[(size_t)k] = (float)t;
    }
  }
}

void Simulator::step(double dt) {
  for (RobotState& r : robots_) {
    const double yaw = r.pose.yaw;
    const Vec2 cur = r.pose.p;
    const Vec2 moved = cur + Vec2{r.cmdV * std::cos(yaw) * dt, r.cmdV * std::sin(yaw) * dt};
    r.pose.yaw += r.cmdW * dt;  // 原地旋转不改变外形，不受碰撞影响
    Vec2 accepted = cur;
    if (!blocked(r, moved)) {
      accepted = moved;                                   // 正常前进
    } else if (!blocked(r, Vec2{moved.x, cur.y})) {
      accepted = Vec2{moved.x, cur.y};                    // 卡墙：保留 x 分量（贴墙滑动）
    } else if (!blocked(r, Vec2{cur.x, moved.y})) {
      accepted = Vec2{cur.x, moved.y};                    // 卡墙：保留 y 分量
    }                                                     // 否则完全被挡，留在原位
    const Vec2 d = accepted - cur;
    r.curV = (d.x == 0.0 && d.y == 0.0) ? 0.0 : length(d) / dt * (r.cmdV < 0.0 ? -1.0 : 1.0);
    r.curW = r.cmdW;
    r.pose.p = accepted;
    updateLasers(r);
  }
  ++steps_;
  simTime_ += dt;
}

}  // namespace flat_sim
