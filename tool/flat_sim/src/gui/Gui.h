// flat_sim —— 2D 俯视图 GUI 接口（SDL2 实现，pimpl 隐藏 SDL 头）
//
// 只有 CMake 找到 SDL2 且 FLAT_SIM_ENABLE_GUI=ON 时才有实现
// （编译期宏 FLAT_SIM_HAVE_GUI 由 CMake 注入）；headless 构建不含本文件实现，
// main.cpp 用同名宏决定是否使用。
//
// 窗口左右分屏：左半为仿真视图，右半为 SLAM 建图视图（SlamView，数据由
// ros 层 SlamListener 提供，为 nullptr 时右半仅显示网格占位）。
//
// 按键：ESC / q 退出（等同关窗）｜ l 开关激光射线显示 ｜ r 复位机器人
//       ｜ v 复位 SLAM 视图（缩放 / 平移）
// 鼠标（右半）：滚轮缩放（锚定光标）｜ 左键拖拽平移
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "core/Simulator.h"

namespace flat_sim {

struct SlamSnapshot;
class SlamView;

class Gui {
 public:
  Gui(const WorldDesc& world, int winW = 1600, int winH = 680);
  ~Gui();

  bool valid() const;  // 窗口是否创建成功（失败时给出原因并建议 --headless）

  // r 键复位机器人后联动调用（如发布 /slam2d/reset 同步复位 SLAM）
  void setResetHook(std::function<void()> hook);

  // 处理窗口事件；返回 false 表示请求退出（关窗 / ESC）
  bool poll(Simulator& sim);
  // 左半绘制仿真状态；右半绘制 SLAM 快照（可为 nullptr）
  void draw(const Simulator& sim, const SlamSnapshot* slam = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flat_sim
