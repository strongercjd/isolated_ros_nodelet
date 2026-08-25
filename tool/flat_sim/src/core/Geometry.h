// flat_sim —— 2D 几何工具：向量 / 位姿 / 射线求交 / 碰撞检测
//
// 坐标系约定（全文一致，README 同步说明）：
//   世界系 +x 东、+y 北；yaw 为弧度，绕 +z 逆时针为正（对齐 REP-103 平面约定）。
//   数据文件（.fworld / .frobot）中的角度一律写「度」，加载时由解析层转为弧度。
#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace flat_sim {

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double deg) { return deg * (kPi / 180.0); }

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2 {
  Vec2 p;
  double yaw = 0.0;  // 弧度
  Pose2() = default;
  Pose2(double x, double y, double yaw_rad) : p{x, y}, yaw(yaw_rad) {}
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::hypot(a.x, a.y); }

// 向量绕原点旋转 ang（弧度）
inline Vec2 rotate(Vec2 v, double ang) {
  const double c = std::cos(ang), s = std::sin(ang);
  return {v.x * c - v.y * s, v.x * s + v.y * c};
}

// local（pose 坐标系下的点）变换到世界系
inline Vec2 toWorld(const Pose2& pose, Vec2 local) { return pose.p + rotate(local, pose.yaw); }

// 位姿复合：outer ⊕ inner —— inner 先按 outer 摆放（用于「机器人位姿 ⊕ 激光挂载位姿」）
inline Pose2 compose(const Pose2& outer, const Pose2& inner) {
  const Vec2 p = toWorld(outer, inner.p);
  return Pose2(p.x, p.y, outer.yaw + inner.yaw);
}

// 射线 o + t*d（d 需为单位向量）与圆 (c, r) 求交；返回最近 t ≥ 0，无交返回 -1。
// 起点在圆内时返回出射点参数。
double rayCircle(Vec2 o, Vec2 d, Vec2 c, double r);

// 射线与线段 a-b 求交；返回 t ≥ 0，无交返回 -1
double raySegment(Vec2 o, Vec2 d, Vec2 a, Vec2 b);

// 射线与带位姿矩形求交（中心 bc、转角 yaw、半长 hl、半宽 hw）；返回 t ≥ 0，无交返回 -1
double rayBox(Vec2 o, Vec2 d, Vec2 bc, double yaw, double hl, double hw);

// 圆 (c, r) 与带位姿矩形是否重叠（碰撞检测用）
bool circleBoxOverlap(Vec2 c, double r, Vec2 bc, double yaw, double hl, double hw);

}  // namespace flat_sim
