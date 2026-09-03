// flat_sim —— .fworld 序列化实现
#include "format/WorldWriter.h"

#include <cstdio>
#include <fstream>
#include <ostream>
#include <sstream>

namespace flat_sim {
namespace {

// 数值输出：最多 6 位有效数字，去掉无谓尾零（够往返且可读）
std::string fmt(double v) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

void poseBlock(std::ostream& os, const Pose2& p) {
  os << "  pose: [" << fmt(p.p.x) << ", " << fmt(p.p.y) << ", "
     << fmt(p.yaw * 180.0 / kPi) << "]\n";
}

void writeObstacles(std::ostream& os, const WorldDesc& w) {
  for (const BoxObstacle& b : w.boxes) {
    os << (b.isWall ? "wall:\n" : "box:\n");
    os << "  name: " << b.name << "\n";
    poseBlock(os, b.pose);
    os << "  size: [" << fmt(b.len) << ", " << fmt(b.wid) << "]\n";
  }
  for (const CircleObstacle& o : w.circles) {
    os << "circle:\n";
    os << "  name: " << o.name << "\n";
    os << "  pose: [" << fmt(o.center.x) << ", " << fmt(o.center.y) << "]\n";
    os << "  radius: " << fmt(o.radius) << "\n";
  }
}

void writeRobots(std::ostream& os, const WorldDesc& w) {
  for (const RobotDesc& r : w.robots) {
    os << "robot:\n";
    os << "  name: " << r.name << "\n";
    os << "  shape: " << r.shape << "\n";
    os << "  radius: " << fmt(r.radius) << "\n";
    poseBlock(os, r.pose);
    os << "  drive: " << r.drive << "\n";
    os << "  color: " << r.color << "\n";
    for (const LaserDesc& ld : r.lasers) {
      os << "  laser:\n";
      os << "    name: " << ld.name << "\n";
      os << "    pose: [" << fmt(ld.mount.p.x) << ", " << fmt(ld.mount.p.y) << ", "
         << fmt(ld.mount.yaw * 180.0 / kPi) << "]\n";
      os << "    range: [" << fmt(ld.rangeMin) << ", " << fmt(ld.rangeMax) << "]\n";
      os << "    fov_deg: " << fmt(ld.fov * 180.0 / kPi) << "\n";
      os << "    samples: " << ld.samples << "\n";
    }
  }
}

// 逐行 run 编码（同行的连续占用合并成 起-止）
void writeEditWalls(std::ostream& os, const GridLayer& g) {
  if (!g.inited()) return;                     // 从未初始化：不写
  if (!g.active && g.wallCount() == 0) return; // 未权威且无用户墙：无意义
  os << "editwall:\n";
  os << "  cell_size: " << fmt(g.cell) << "\n";
  os << "  origin: [" << fmt(g.ox) << ", " << fmt(g.oy) << "]\n";
  os << "  size: [" << g.cols << ", " << g.rows << "]\n";
  if (g.active)
    os << "  active: 1\n";  // 权威占用层（含栅格化的原障碍；即使全部清空也须保真）
  for (int j = 0; j < g.rows; ++j) {
    // 收集该行占用列
    std::string line;
    int i = 0;
    while (i < g.cols) {
      if (!g.at(i, j)) { ++i; continue; }
      const int start = i;
      while (i < g.cols && g.at(i, j)) ++i;
      const int end = i - 1;
      if (!line.empty()) line += ", ";
      line += std::to_string(start) + "-" + std::to_string(end);
    }
    if (!line.empty()) os << "  row: " << j << " " << line << "\n";
  }
}

void writeAnnotations(std::ostream& os, const std::vector<Annotation>& anns) {
  for (const Annotation& a : anns) {
    os << "annotation:\n";
    os << "  from: [" << fmt(a.from.x) << ", " << fmt(a.from.y) << "]\n";
    os << "  to: [" << fmt(a.to.x) << ", " << fmt(a.to.y) << "]\n";
  }
}

}  // namespace

std::string worldToText(const WorldDesc& w) {
  std::ostringstream os;
  os << "# " << w.name << ".fworld —— flat_sim 世界（另存于 flat_sim 编辑器，含编辑墙/测量标注）\n";
  os << "world:\n";
  os << "  name: " << w.name << "\n";
  if (w.hasSize) os << "  size: [" << fmt(w.width) << ", " << fmt(w.height) << "]\n";
  if (w.resolution > 0) os << "  resolution: " << fmt(w.resolution) << "\n";
  os << "  timestep_ms: " << w.timestepMs << "\n";
  writeObstacles(os, w);
  writeRobots(os, w);
  writeEditWalls(os, w.editGrid);
  writeAnnotations(os, w.annotations);
  return os.str();
}

std::string saveWorldFile(const WorldDesc& w, const std::string& path) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) return "无法打开文件写入: " + path;
  out << worldToText(w);
  if (!out.good()) return "写入失败: " + path;
  return "";
}

}  // namespace flat_sim
