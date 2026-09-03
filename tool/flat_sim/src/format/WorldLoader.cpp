// flat_sim —— .fworld / .frobot 加载实现
#include "format/WorldLoader.h"

#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <vector>

#include "format/TextFormat.h"

namespace flat_sim {
namespace {

using format::FormatError;
using format::Node;

// 可恢复问题只告警，不中断（需求 4.1：未知字段忽略或警告，不崩溃）
void warn(const std::string& file, int line, const std::string& msg) {
  std::fprintf(stderr, "[flat_sim][警告] %s:%d: %s\n", file.c_str(), line, msg.c_str());
}

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw FormatError(path, 0, "无法打开文件");
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string dirOf(const std::string& path) {
  const size_t p = path.find_last_of('/');
  return p == std::string::npos ? "." : path.substr(0, p);
}

std::string joinPath(const std::string& baseDir, const std::string& rel) {
  if (!rel.empty() && rel[0] == '/') return rel;  // 绝对路径原样使用
  return baseDir + "/" + rel;
}

// pose: [x, y] 或 [x, y, yaw_deg]（度 → 弧度）
Pose2 parsePose(const Node& n, const std::string& file) {
  const std::vector<double> v = format::toDoubleList(n, 0, file);
  if (v.size() != 2 && v.size() != 3)
    throw FormatError(file, n.line, "`" + n.key + "` 应为 [x, y] 或 [x, y, yaw_deg]");
  return Pose2(v[0], v[1], v.size() == 3 ? deg2rad(v[2]) : 0.0);
}

std::string parseName(const Node& blk, const std::string& file, const std::string& fallback) {
  const Node* n = blk.find("name");
  if (!n) {
    warn(file, blk.line, "缺少 name，使用缺省 \"" + fallback + "\"");
    return fallback;
  }
  return format::scalarValue(*n, file);
}

RobotDesc parseRobot(const Node& blk, const std::string& file) {
  static const std::set<std::string> kRobotFields{"name", "shape", "radius", "pose",
                                                  "drive", "color", "laser"};
  static const std::set<std::string> kLaserFields{"name", "pose", "range", "fov_deg", "fov",
                                                  "samples"};
  RobotDesc r;
  const Node* n = nullptr;

  if ((n = blk.find("name")) != nullptr) {
    r.name = format::scalarValue(*n, file);
  } else {
    throw FormatError(file, blk.line, "robot 缺少 name（name 亦用作话题前缀 /<name>/...）");
  }
  if ((n = blk.find("shape")) != nullptr) {
    r.shape = format::scalarValue(*n, file);
    if (r.shape != "circle")
      throw FormatError(file, n->line, "首期仅支持 shape: circle（当前: " + r.shape + "）");
  }
  if ((n = blk.find("radius")) != nullptr) {
    r.radius = format::toDouble(*n, file);
    if (r.radius <= 0) throw FormatError(file, n->line, "radius 必须为正数（米）");
  }
  if ((n = blk.find("pose")) != nullptr) r.pose = parsePose(*n, file);
  if ((n = blk.find("drive")) != nullptr) {
    r.drive = format::scalarValue(*n, file);
    if (r.drive != "diff")
      warn(file, n->line, "首期仅实现 drive: diff，\"" + r.drive + "\" 按 diff 处理");
  }
  if ((n = blk.find("color")) != nullptr) r.color = format::scalarValue(*n, file);

  // laser 块可写多个；第 1 个 → /<name>/base_scan，其后 → base_scan_2、base_scan_3 ...
  const std::vector<const Node*> lasers = blk.findAll("laser");
  int idx = 0;
  for (const Node* l : lasers) {
    LaserDesc ld;
    ld.name = "laser_" + std::to_string(++idx);
    if ((n = l->find("name")) != nullptr) ld.name = format::scalarValue(*n, file);
    if ((n = l->find("pose")) != nullptr) ld.mount = parsePose(*n, file);
    if ((n = l->find("range")) != nullptr) {
      const std::vector<double> v = format::toDoubleList(*n, 2, file);
      if (v[0] < 0 || v[1] <= v[0])
        throw FormatError(file, n->line, "range 应为 [min, max] 且 0 ≤ min < max");
      ld.rangeMin = v[0];
      ld.rangeMax = v[1];
    } else {
      warn(file, l->line, "laser 缺 range，默认 [0.0, 10.0]");
    }
    double fovDeg = 180.0;
    if ((n = l->find("fov_deg")) != nullptr || (n = l->find("fov")) != nullptr) {
      fovDeg = format::toDouble(*n, file);
      if (fovDeg <= 0 || fovDeg > 360)
        throw FormatError(file, n->line, "fov_deg 应在 (0, 360]（度）");
    } else {
      warn(file, l->line, "laser 缺 fov_deg，默认 180");
    }
    ld.fov = deg2rad(fovDeg);
    if ((n = l->find("samples")) != nullptr) {
      ld.samples = format::toInt(*n, file);
      if (ld.samples < 1) throw FormatError(file, n->line, "samples 应 ≥ 1");
    } else {
      warn(file, l->line, "laser 缺 samples，默认 181");
    }
    for (const Node& c : l->children)
      if (!kLaserFields.count(c.key)) warn(file, c.line, "laser 忽略未知字段: " + c.key);
    r.lasers.push_back(ld);
  }
  for (const Node& c : blk.children)
    if (!kRobotFields.count(c.key)) warn(file, c.line, "robot 忽略未知字段: " + c.key);
  return r;
}

}  // namespace

RobotDesc loadRobotFile(const std::string& path) {
  const Node root = format::parseText(readFile(path), path);
  const Node* blk = root.find("robot");
  if (!blk) throw FormatError(path, 0, "缺少 robot: 块");
  for (const Node& c : root.children)
    if (c.key != "robot") warn(path, c.line, "忽略 .frobot 顶层字段: " + c.key);
  return parseRobot(*blk, path);
}

WorldDesc loadWorld(const std::string& path) {
  static const std::set<std::string> kWorldFields{"name", "size", "resolution", "timestep_ms"};
  static const std::set<std::string> kBoxFields{"name", "pose", "size"};
  static const std::set<std::string> kCircleFields{"name", "pose", "radius"};

  const Node root = format::parseText(readFile(path), path);
  WorldDesc w;
  w.sourceFile = path;

  for (const Node& c : root.children) {
    const Node* n = nullptr;
    if (c.key == "world") {
      if ((n = c.find("name")) != nullptr) w.name = format::scalarValue(*n, path);
      if ((n = c.find("size")) != nullptr) {
        const std::vector<double> v = format::toDoubleList(*n, 2, path);
        if (v[0] <= 0 || v[1] <= 0) throw FormatError(path, n->line, "size 应为 [宽, 高] 且均为正");
        w.width = v[0];
        w.height = v[1];
        w.hasSize = true;
      }
      if ((n = c.find("resolution")) != nullptr) {
        w.resolution = format::toDouble(*n, path);
        if (w.resolution <= 0) throw FormatError(path, n->line, "resolution 必须为正数");
      }
      if ((n = c.find("timestep_ms")) != nullptr) {
        w.timestepMs = format::toInt(*n, path);
        if (w.timestepMs <= 0) throw FormatError(path, n->line, "timestep_ms 必须为正整数");
      }
      for (const Node& f : c.children)
        if (!kWorldFields.count(f.key)) warn(path, f.line, "world 忽略未知字段: " + f.key);
    } else if (c.key == "wall" || c.key == "box") {
      BoxObstacle b;
      b.isWall = (c.key == "wall");  // 保存时还原 wall/box 标签
      b.name = parseName(c, path, c.key + "_" + std::to_string(w.boxes.size() + 1));
      if ((n = c.find("pose")) != nullptr) {
        b.pose = parsePose(*n, path);
      } else {
        throw FormatError(path, c.line, c.key + " 缺少 pose: [x, y, yaw_deg]");
      }
      if ((n = c.find("size")) != nullptr) {
        const std::vector<double> v = format::toDoubleList(*n, 2, path);
        if (v[0] <= 0 || v[1] <= 0)
          throw FormatError(path, n->line, c.key + " size 应为 [长, 宽] 且均为正");
        b.len = v[0];
        b.wid = v[1];
      } else {
        throw FormatError(path, c.line, c.key + " 缺少 size: [长, 宽]");
      }
      for (const Node& f : c.children)
        if (!kBoxFields.count(f.key)) warn(path, f.line, c.key + " 忽略未知字段: " + f.key);
      w.boxes.push_back(b);
    } else if (c.key == "circle") {
      CircleObstacle o;
      o.name = parseName(c, path, "circle_" + std::to_string(w.circles.size() + 1));
      if ((n = c.find("pose")) != nullptr) {
        const Pose2 p = parsePose(*n, path);
        o.center = p.p;  // 圆无朝向，yaw 忽略
      } else {
        throw FormatError(path, c.line, "circle 缺少 pose: [x, y]");
      }
      if ((n = c.find("radius")) != nullptr) {
        o.radius = format::toDouble(*n, path);
        if (o.radius <= 0) throw FormatError(path, n->line, "radius 必须为正数（米）");
      } else {
        throw FormatError(path, c.line, "circle 缺少 radius");
      }
      for (const Node& f : c.children)
        if (!kCircleFields.count(f.key)) warn(path, f.line, "circle 忽略未知字段: " + f.key);
      w.circles.push_back(o);
    } else if (c.key == "robot") {
      w.robots.push_back(parseRobot(c, path));
    } else if (c.key == "robot_file") {
      const std::string rel = format::scalarValue(c, path);
      const std::string full = joinPath(dirOf(path), rel);
      RobotDesc rd = loadRobotFile(full);
      if ((n = c.find("pose")) != nullptr) rd.pose = parsePose(*n, path);  // 世界内覆盖位姿
      w.robots.push_back(rd);
    } else if (c.key == "annotation") {
      // 测量标注：辅助线，不参与仿真
      Annotation an;
      if ((n = c.find("from")) != nullptr) {
        const std::vector<double> v = format::toDoubleList(*n, 2, path);
        an.from = Vec2{v[0], v[1]};
      } else {
        throw FormatError(path, c.line, "annotation 缺少 from: [x, y]");
      }
      if ((n = c.find("to")) != nullptr) {
        const std::vector<double> v = format::toDoubleList(*n, 2, path);
        an.to = Vec2{v[0], v[1]};
      } else {
        throw FormatError(path, c.line, "annotation 缺少 to: [x, y]");
      }
      w.annotations.push_back(an);
    } else if (c.key == "editwall") {
      // 用户编辑墙格层：占用的行按 run 编码
      GridLayer gl;
      if ((n = c.find("cell_size")) != nullptr) {
        gl.cell = format::toDouble(*n, path);
        if (gl.cell <= 0) throw FormatError(path, n->line, "editwall cell_size 必须为正数");
      } else {
        gl.cell = 0.1;
      }
      if ((n = c.find("origin")) != nullptr) {
        const std::vector<double> v = format::toDoubleList(*n, 2, path);
        gl.ox = v[0];
        gl.oy = v[1];
      }
      if ((n = c.find("size")) != nullptr) {
        const std::vector<double> v = format::toDoubleList(*n, 2, path);
        gl.cols = (int)v[0];
        gl.rows = (int)v[1];
        if (gl.cols <= 0 || gl.rows <= 0)
          throw FormatError(path, n->line, "editwall size 应为 [列数, 行数] 且均为正整数");
      }
      if (gl.cols <= 0 || gl.rows <= 0)
        throw FormatError(path, c.line, "editwall 缺少 size: [列数, 行数]");
      // active: 1 表示权威占用层（含栅格化的原障碍），缺省为旧的"仅用户墙"叠加层
      if ((n = c.find("active")) != nullptr) gl.active = format::toInt(*n, path) != 0;
      gl.occ.assign((size_t)gl.cols * gl.rows, 0);
      // row: <行号> <列起>-<列止>[, <列起>-<列止>...]
      static const std::set<std::string> kEditWallFields{"cell_size", "origin", "size", "active", "row"};
      for (const Node& rr : c.children) {
        if (rr.key != "row") {
          if (!kEditWallFields.count(rr.key))
            warn(path, rr.line, "editwall 忽略未知字段: " + rr.key);
          continue;
        }
        std::string v = format::scalarValue(rr, path);
        for (char& ch : v)  // 分隔符统一按空白处理（容忍手写逗号）
          if (ch == ',') ch = ' ';
        std::istringstream is(v);
        int j = 0;
        is >> j;
        std::string seg;
        while (is >> seg) {
          const size_t dash = seg.find('-');
          if (dash == std::string::npos)
            throw FormatError(path, rr.line, "editwall row 段应为 列起-列止: \"" + seg + "\"");
          const int i0 = std::stoi(seg.substr(0, dash));
          const int i1 = std::stoi(seg.substr(dash + 1));
          for (int i = i0; i <= i1; ++i) gl.set(i, j, true);
        }
      }
      w.editGrid = gl;
    } else {
      warn(path, c.line, "忽略未知顶层块: " + c.key);
    }
  }

  if (w.robots.empty())
    throw FormatError(path, 0, "世界中没有任何机器人（需要 robot: 块或 robot_file: 引用）");
  for (size_t i = 0; i < w.robots.size(); ++i)
    for (size_t j = i + 1; j < w.robots.size(); ++j)
      if (w.robots[i].name == w.robots[j].name)
        throw FormatError(path, 0, "机器人重名: \"" + w.robots[i].name + "\"");
  return w;
}

}  // namespace flat_sim
