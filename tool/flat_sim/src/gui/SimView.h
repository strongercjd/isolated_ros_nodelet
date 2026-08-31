// flat_sim —— 2D 俯视仿真视图（Qt6 Widgets 自绘，paintEvent + QPainter）
//
// 只有 CMake 找到 Qt6 且 FLAT_SIM_ENABLE_GUI=ON 时才有实现（编译期宏
// FLAT_SIM_HAVE_GUI 由 CMake 注入）；headless 构建不含本文件实现。
//
// 显示内容（与原 SDL 版一致）：世界边界 → 矩形/圆障碍 → 激光射线（半透明，
// 可开关）→ 机器人圆 + 朝向线。
//
// 按键：ESC / q 退出 ｜ l 开关激光射线显示 ｜ r 复位机器人（联动逻辑在外部）
//
// 本文件不依赖 ROS：只 include core 层数据结构，保持"gui 不依赖 ROS"的分层。
#pragma once

#include <QWidget>

#include "core/Simulator.h"
#include "core/World.h"

namespace flat_sim {

class SimView : public QWidget {
  Q_OBJECT
 public:
  explicit SimView(const WorldDesc& world, QWidget* parent = nullptr);

  QSize sizeHint() const override;

  // 每个仿真步后由控制器设置并触发重绘（sim 的生命周期由外部保证）
  void setSimulator(const Simulator* sim);

 Q_SIGNALS:
  void resetRequested();  // r 键：复位机器人（外部联动发布 /slam2d/reset）
  void quitRequested();   // ESC / q：退出事件循环

 protected:
  void paintEvent(QPaintEvent* e) override;
  void keyPressEvent(QKeyEvent* e) override;
  void resizeEvent(QResizeEvent* e) override;

 private:
  // 世界 → 屏幕（y 翻转：世界 +y 朝上，屏幕 +y 朝下）
  QPointF toScreen(Vec2 w) const;
  // 视口适配：优先世界 size（原点居中），否则用全部几何体包围盒
  void fitView();

  WorldDesc world_;  // 静态快照（障碍等不随仿真变化）
  const Simulator* sim_ = nullptr;
  bool showLasers_ = true;
  double scale_ = 40.0;          // 像素 / 米
  double minX_ = 0.0, minY_ = 0.0;
  static constexpr double kPad = 24.0;  // 视图边距（像素）
};

}  // namespace flat_sim
