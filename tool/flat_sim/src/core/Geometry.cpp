// flat_sim —— 几何求交实现（解析法，不做逐步步进，world 的 resolution 字段不参与计算）
#include "core/Geometry.h"

#include <algorithm>

namespace flat_sim {

double rayCircle(Vec2 o, Vec2 d, Vec2 c, double r) {
  const Vec2 oc = o - c;
  const double b = dot(oc, d);
  const double cc = dot(oc, oc) - r * r;
  const double disc = b * b - cc;
  if (disc < 0.0) return -1.0;
  const double s = std::sqrt(disc);
  const double t1 = -b - s;
  if (t1 >= 0.0) return t1;
  const double t2 = -b + s;
  return t2 >= 0.0 ? t2 : -1.0;
}

double raySegment(Vec2 o, Vec2 d, Vec2 a, Vec2 b) {
  const Vec2 seg = b - a;
  const Vec2 ao = a - o;
  const double denom = cross(d, seg);
  if (std::fabs(denom) < 1e-12) return -1.0;  // 平行（含共线，忽略重合情形）
  const double t = cross(ao, seg) / denom;    // 沿射线参数
  const double u = cross(ao, d) / denom;      // 沿线段参数
  if (t >= 0.0 && u >= 0.0 && u <= 1.0) return t;
  return -1.0;
}

double rayBox(Vec2 o, Vec2 d, Vec2 bc, double yaw, double hl, double hw) {
  // 把射线变换到矩形局部系（旋转平移不改变参数 t）
  const Vec2 lo = rotate(o - bc, -yaw);
  const Vec2 ld = rotate(d, -yaw);
  double tmin = -1e18, tmax = 1e18;
  if (std::fabs(ld.x) < 1e-12) {
    if (std::fabs(lo.x) > hl) return -1.0;
  } else {
    double t1 = (-hl - lo.x) / ld.x, t2 = (hl - lo.x) / ld.x;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) return -1.0;
  }
  if (std::fabs(ld.y) < 1e-12) {
    if (std::fabs(lo.y) > hw) return -1.0;
  } else {
    double t1 = (-hw - lo.y) / ld.y, t2 = (hw - lo.y) / ld.y;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) return -1.0;
  }
  if (tmax < 0.0) return -1.0;      // 整个矩形在射线反方向
  return tmin > 0.0 ? tmin : 0.0;   // 起点在矩形内部按 0 处理
}

bool circleBoxOverlap(Vec2 c, double r, Vec2 bc, double yaw, double hl, double hw) {
  const Vec2 l = rotate(c - bc, -yaw);
  const double cx = std::max(-hl, std::min(hl, l.x));
  const double cy = std::max(-hw, std::min(hw, l.y));
  const Vec2 diff = l - Vec2{cx, cy};
  return dot(diff, diff) <= r * r;  // 圆心在矩形内时 diff=0，同样判定重叠
}

}  // namespace flat_sim
