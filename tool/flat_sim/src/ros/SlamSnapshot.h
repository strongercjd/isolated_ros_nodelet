// flat_sim —— SLAM 可视化数据快照（纯标准库类型，无 ROS 依赖）
//
// gui 层（SlamView/Gui）只 include 本头文件，保持"gui 不依赖 ROS"的分层；
// ros 层（SlamListener）在回调里填充，地图经 shared_ptr 共享（4MB 免拷贝）。
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace flat_sim {

struct SlamSnapshot {
  // 占用栅格地图（对应 nav_msgs/OccupancyGrid 的拷贝，值 0-100，50=未观测）
  struct Map {
    std::shared_ptr<const std::vector<int8_t>> data; // 行优先，row0 = y 最小（标准语义）
    int width = 0;
    int height = 0;
    double resolution = 0.05;  // m / 格
    double originX = 0.0;      // 左下角世界坐标
    double originY = 0.0;
    uint32_t seq = 0;  // 代数，每收到新地图 +1；SlamView 据此判断是否重填纹理
  };
  Map map;

  // 平铺的 x,y 序列（世界系 / "map" 系）
  std::vector<float> inputXY;    // 红色：配准前预测位姿下的当前帧点云
  std::vector<float> mappingXY;  // 蓝色：配准后点云
  bool hasInput = false;
  bool hasMapping = false;

  // 机器人位姿（蓝色箭头）
  bool hasPose = false;
  double poseX = 0.0, poseY = 0.0, poseYaw = 0.0;
};

}  // namespace flat_sim
