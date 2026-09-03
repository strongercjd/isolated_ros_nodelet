// flat_sim —— 世界与机器人的运行时模型（数据结构只在此定义）
// 本层不含任何 I/O：由 format/ 解析层填充，由 Simulator / Gui / RosBridge 消费。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "core/Geometry.h"

namespace flat_sim {

// 矩形障碍（wall 与 box 同构，仅语义不同；yaw 为 0 时 len 沿 x、wid 沿 y）
struct BoxObstacle {
  std::string name;
  Pose2 pose;         // 中心位姿
  double len = 1.0;   // 长（米，局部 x 方向）
  double wid = 0.15;  // 宽（米，局部 y 方向）
  bool isWall = false;  // 来源是 wall: 块（保存时还原标签用；不影响仿真/显示）
};

struct CircleObstacle {
  std::string name;
  Vec2 center;
  double radius = 0.3;
};

// 激光（挂在机器人上，位姿相对机体；文件里写度，这里已转弧度）
struct LaserDesc {
  std::string name;              // 缺省 laser_1、laser_2 ...
  Pose2 mount;                   // 相对机器人中心
  double rangeMin = 0.0;         // 米
  double rangeMax = 10.0;        // 米
  double fov = deg2rad(180.0);   // 视场角（弧度）
  int samples = 181;             // 线数
};

// CAD 式测量标注（两点直线距离）：纯辅助线，不参与仿真/碰撞；随 .fworld 保存。
struct Annotation {
  Vec2 from;  // 世界坐标（米）
  Vec2 to;
};

// 编辑墙占据层：覆盖某矩形区域的规则正交网格（格子与世界轴对齐）。
// 分两种状态：
//   active == false（默认）：仅记录"用户新增的墙"，原障碍仍按几何解析表达。
//   active == true ：权威占用层——进入编辑时把原障碍(墙/箱/圆)栅格化进本层，
//                    左键补占用、右键清占用（可在原墙上"抠洞"）；此后碰撞与显示
//                    都以本层为准（激光/小车可穿过被清空的洞）。active 随 .fworld 保存。
// GUI 编辑与 Simulator 遮挡共用同一实例（SimRunner 持 shared_ptr，见 Simulator）。
struct GridLayer {
  double cell = 0.1;      // 格子边长（米）
  int cols = 0;           // 列数（沿 +x）
  int rows = 0;           // 行数（沿 +y）
  double ox = 0.0;        // 网格覆盖区最小角点的世界坐标
  double oy = 0.0;
  bool active = false;    // 权威占用层标记（含栅格化的原障碍）
  std::vector<uint8_t> occ;  // rows*cols 行主序（occ[rows*i? 见 at]，1=墙）

  bool inited() const { return cols > 0 && rows > 0 && occ.size() == (size_t)cols * rows; }
  bool inRange(int i, int j) const {
    return inited() && i >= 0 && i < cols && j >= 0 && j < rows;
  }
  bool at(int i, int j) const { return inRange(i, j) && occ[(size_t)j * cols + i] != 0; }
  void set(int i, int j, bool v) {
    if (inRange(i, j)) occ[(size_t)j * cols + i] = v ? 1 : 0;
  }
  int wallCount() const {
    int n = 0;
    for (uint8_t o : occ) n += o != 0;
    return n;
  }

  // 网格覆盖范围（世界坐标，米）
  double minX() const { return ox; }
  double minY() const { return oy; }
  double maxX() const { return ox + cols * cell; }
  double maxY() const { return oy + rows * cell; }

  // 覆盖矩形区域 x0..x1/y0..y1、格边长 cellSize 重排并清空
  void configure(double x0, double x1, double y0, double y1, double cellSize) {
    cell = cellSize > 0.0 ? cellSize : 0.1;
    ox = x0;
    oy = y0;
    cols = (int)std::ceil((x1 - x0) / cell);
    rows = (int)std::ceil((y1 - y0) / cell);
    if (cols < 1) cols = 1;
    if (rows < 1) rows = 1;
    occ.assign((size_t)cols * rows, 0);
  }
  // 世界坐标 -> 格下标；在网格外返回 false
  bool cellOf(Vec2 w, int& i, int& j) const {
    if (cols <= 0 || rows <= 0) return false;
    i = (int)std::floor((w.x - ox) / cell);
    j = (int)std::floor((w.y - oy) / cell);
    return inRange(i, j);
  }
  // 格中心世界坐标
  Vec2 center(int i, int j) const { return {ox + ((double)i + 0.5) * cell,
                                            oy + ((double)j + 0.5) * cell}; }
  // 把矩形区域 x0..x1/y0..y1 内被覆盖的格子置为墙（迁移/栅格化用）
  void fillWorldRect(double x0, double y0, double x1, double y1) {
    if (x1 < x0) std::swap(x0, x1);
    if (y1 < y0) std::swap(y0, y1);
    int i0 = (int)std::floor((x0 - ox) / cell);
    int i1 = (int)std::floor((x1 - ox) / cell);
    int j0 = (int)std::floor((y0 - oy) / cell);
    int j1 = (int)std::floor((y1 - oy) / cell);
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) set(i, j, true);
  }
  void clearWalls() { std::fill(occ.begin(), occ.end(), 0); }
  void copyFrom(const GridLayer& o) {
    cell = o.cell; cols = o.cols; rows = o.rows; ox = o.ox; oy = o.oy;
    active = o.active; occ = o.occ;
  }
};

// 机器人 = 平面形状 + 驱动 + 传感器挂载（无 URDF / 连杆 / TF frame 树）
struct RobotDesc {
  std::string name = "mycar";      // 亦用于话题前缀 /<name>/...
  std::string shape = "circle";    // 首期仅 circle
  double radius = 0.1;             // 圆形半径（米）
  Pose2 pose;                      // 世界系初始位姿
  std::string drive = "diff";      // 首期仅 diff（差速）
  std::string color = "red";       // 仅显示用
  std::vector<LaserDesc> lasers;
};

struct WorldDesc {
  std::string name = "flat_world";
  double width = 15.0;    // 世界宽（米，GUI 视图适配用）
  double height = 10.0;   // 世界高（米）
  bool hasSize = false;   // 文件是否显式给了 size
  double resolution = 0.02;  // 兼容字段：解析法求交不使用
  int timestepMs = 50;       // 仿真步长（毫秒）
  std::vector<BoxObstacle> boxes;
  std::vector<CircleObstacle> circles;
  std::vector<RobotDesc> robots;
  GridLayer editGrid;            // 用户编辑墙（editwall 块；空 = 无编辑层）
  std::vector<Annotation> annotations;  // 测量标注（辅助，不参与仿真）
  std::string sourceFile;        // 来源文件（GUI 标题 / 报错用）
};

}  // namespace flat_sim
