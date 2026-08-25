// flat_sim —— 世界与机器人的运行时模型（数据结构只在此定义）
// 本层不含任何 I/O：由 format/ 解析层填充，由 Simulator / Gui / RosBridge 消费。
#pragma once

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
  std::string sourceFile;    // 来源文件（GUI 标题 / 报错用）
};

}  // namespace flat_sim
