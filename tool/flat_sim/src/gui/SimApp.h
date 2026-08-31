// flat_sim —— Qt GUI 控制器：QTimer 固定步长驱动 SimRunner，接线 SimView
//
// 事件模型（全单线程，主线程）：
//   stepTimer_（步长 ms，PreciseTimer）→ SimRunner::stepOnce() → view->update()
//     单步顺序与 headless 纯循环完全一致（唯一来源 SimRunner::stepOnce）；
//     事件循环被长绘制阻塞时 Qt 只补发一次而非追帧，等价旧"慢机重对齐"。
//   quitTimer_（200ms）→ 轮询 ros::ok()：roscpp 接管了 SIGINT，但 Qt 事件
//     循环对信号无感知，Ctrl+C 后靠它退出。
//
// 编入 flat_sim_node 可执行目标（依赖 ROS 层 SimRunner，不进 flat_sim_gui 库）。
#pragma once

#include <QObject>

#include "core/World.h"

class QTimer;

namespace flat_sim {

class SimRunner;
class SimView;

class SimApp : public QObject {
  Q_OBJECT
 public:
  SimApp(SimRunner& runner, const WorldDesc& world, QObject* parent = nullptr);

  // 创建视图、启动定时器并 show()；在进入 QApplication::exec() 前调用
  void start();

 private Q_SLOTS:
  void onStep();      // 仿真单步 + 重绘
  void onQuitPoll();  // Ctrl+C 检测
  void onReset();     // r 键：复位机器人并联动发布 /slam2d/reset

 private:
  SimRunner& runner_;
  SimView* view_ = nullptr;
  QTimer* stepTimer_ = nullptr;
  QTimer* quitTimer_ = nullptr;
};

}  // namespace flat_sim
