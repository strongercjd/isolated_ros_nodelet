// flat_sim —— 编辑墙格层工具实现（栅格化 / 相交判定）
#include "core/GridLayerTools.h"

#include <algorithm>
#include <cmath>

#include "core/Geometry.h"

namespace flat_sim {
namespace {

// 两区间重叠
inline bool overlap1D(double a0, double a1, double b0, double b1) {
  return a1 >= b0 && b1 >= a0;
}

// 旋转矩形（中心 pose，半长 hl/hw）与轴对齐格子（左下 (cx0,cy0)、边长 cs）是否相交
// 分离轴：矩形的两条局部轴 + 格子的 x/y 轴，逐一投影判重叠。
bool rectCellOverlap(const Pose2& p, double hl, double hw, double cx0, double cy0, double cs) {
  const double ca = std::cos(p.yaw), sa = std::sin(p.yaw);
  // 矩形局部轴在世界系的方向
  const Vec2 axA{ca, sa};   // 矩形 +x 方向
  const Vec2 axB{-sa, ca};  // 矩形 +y 方向
  // 矩形四角（世界系）
  const Vec2 R[4] = {
      flat_sim::toWorld(p, Vec2{-hl, -hw}),
      flat_sim::toWorld(p, Vec2{hl, -hw}),
      flat_sim::toWorld(p, Vec2{hl, hw}),
      flat_sim::toWorld(p, Vec2{-hl, hw}),
  };
  // 格子四角（世界系，轴对齐）
  const Vec2 S[4] = {
      Vec2{cx0, cy0}, Vec2{cx0 + cs, cy0}, Vec2{cx0 + cs, cy0 + cs}, Vec2{cx0, cy0 + cs},
  };
  auto axisOk = [&](const Vec2& ax) {
    double rm = 1e300, rx = -1e300;
    for (const Vec2& v : R) { const double d = dot(ax, v); rm = std::min(rm, d); rx = std::max(rx, d); }
    double sm = 1e300, sx = -1e300;
    for (const Vec2& v : S) { const double d = dot(ax, v); sm = std::min(sm, d); sx = std::max(sx, d); }
    return overlap1D(rm, rx, sm, sx);
  };
  if (!axisOk(axA)) return false;
  if (!axisOk(axB)) return false;
  if (!axisOk(Vec2{1.0, 0.0})) return false;
  if (!axisOk(Vec2{0.0, 1.0})) return false;
  return true;  // 四个轴都重叠 => 相交
}

// 圆与格子相交：圆心到格子（AABB）最近距离 ≤ 半径
bool circleCellOverlap(Vec2 c, double r, double cx0, double cy0, double cs) {
  const double nx = std::max(cx0, std::min(c.x, cx0 + cs));
  const double ny = std::max(cy0, std::min(c.y, cy0 + cs));
  const double dx = c.x - nx, dy = c.y - ny;
  return dx * dx + dy * dy <= r * r;
}

// 把障碍物占据的格下标区间限定在格层内
int clampIndex(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

}  // namespace

void bakeWorldObstacles(const WorldDesc& w, GridLayer& g) {
  if (!g.inited()) return;
  const double cs = g.cell;
  const double ox = g.ox, oy = g.oy;

  for (const BoxObstacle& b : w.boxes) {
    const double hl = b.len / 2.0, hw = b.wid / 2.0;
    // 包围盒（世界系）→ 待查格区间
    double bx0 = 1e300, bx1 = -1e300, by0 = 1e300, by1 = -1e300;
    const Vec2 corners[4] = {
        flat_sim::toWorld(b.pose, Vec2{-hl, -hw}), flat_sim::toWorld(b.pose, Vec2{hl, -hw}),
        flat_sim::toWorld(b.pose, Vec2{hl, hw}),   flat_sim::toWorld(b.pose, Vec2{-hl, hw}),
    };
    for (const Vec2& c : corners) {
      bx0 = std::min(bx0, c.x); bx1 = std::max(bx1, c.x);
      by0 = std::min(by0, c.y); by1 = std::max(by1, c.y);
    }
    const int i0 = clampIndex((int)std::floor((bx0 - ox) / cs), 0, g.cols - 1);
    const int i1 = clampIndex((int)std::floor((bx1 - ox) / cs), 0, g.cols - 1);
    const int j0 = clampIndex((int)std::floor((by0 - oy) / cs), 0, g.rows - 1);
    const int j1 = clampIndex((int)std::floor((by1 - oy) / cs), 0, g.rows - 1);
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i)
        if (rectCellOverlap(b.pose, hl, hw, ox + (double)i * cs, oy + (double)j * cs, cs))
          g.set(i, j, true);
  }

  for (const CircleObstacle& o : w.circles) {
    const int i0 = clampIndex((int)std::floor((o.center.x - o.radius - ox) / cs), 0, g.cols - 1);
    const int i1 = clampIndex((int)std::floor((o.center.x + o.radius - ox) / cs), 0, g.cols - 1);
    const int j0 = clampIndex((int)std::floor((o.center.y - o.radius - oy) / cs), 0, g.rows - 1);
    const int j1 = clampIndex((int)std::floor((o.center.y + o.radius - oy) / cs), 0, g.rows - 1);
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i)
        if (circleCellOverlap(o.center, o.radius, ox + (double)i * cs, oy + (double)j * cs, cs))
          g.set(i, j, true);
  }
}

}  // namespace flat_sim
