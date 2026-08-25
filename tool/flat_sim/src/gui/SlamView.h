// flat_sim —— 右侧 SLAM 建图视图（SDL2 实现，pimpl 隐藏 SDL 头）
//
// 显示内容与 lidarslam_2d 参考工程的 cv_ui 画布一致：
//   占用栅格地图（灰度）→ 5m 网格线 → 红色 input_cloud → 蓝色 mapping_cloud
//   → 蓝色位姿箭头；y 向上，0.01 m/px。视图默认跟随最新位姿（机器人初始
//   不在原点时箭头也不会跑出视口），拖拽平移即脱离，v 键恢复跟随。
// 地图经 STREAMING 纹理上传，仅在代数（seq）变化时重填。
//
// 本文件不依赖 ROS：数据来自纯标准库的 SlamSnapshot（ros/SlamSnapshot.h）。
#pragma once

#include <memory>

#include "ros/SlamSnapshot.h"

struct SDL_Renderer;

namespace flat_sim {

class SlamView {
 public:
  SlamView();
  ~SlamView();

  void setPane(int x, int y, int w, int h);  // 窗口内绘制区域
  bool inside(int mx, int my) const;         // 窗口坐标是否落在本视图

  // 交互（窗口系坐标）：滚轮缩放（锚定光标）/ 左键拖拽平移 / 复位视图
  void handleWheel(int mx, int my, int dy);
  void handleDrag(int dx, int dy);
  void resetView();

  void draw(SDL_Renderer* ren, const SlamSnapshot& snap);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flat_sim
