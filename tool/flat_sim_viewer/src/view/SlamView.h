// flat_sim_viewer —— SLAM 建图视图（Qt6 Widgets 自绘，paintEvent + QPainter）
//
// 显示内容与 lidarslam_2d 参考工程的 cv_ui 画布一致（视图数学自
// tool/flat_sim 的 SDL 版 SlamView 迁移，行为逐行保留）：
//   黑底 → 占用栅格地图（灰度）→ 5m 网格线 → 红色 input_cloud
//   → 蓝色 mapping_cloud → 蓝色位姿箭头
// 地图经 QImage 缓存，仅在代数（seq）变化时重填（对应 SDL STREAMING 纹理）。
//
// 交互：滚轮缩放（锚定光标）/ 左键拖拽平移（拖拽即脱离跟随）/ v 恢复跟随。
// 跟随模式下视图中心锁定最新位姿（机器人初始不在原点时箭头也不会跑出视口）。
//
// 本文件不依赖 ROS：数据来自纯标准库的 SlamSnapshot（ros/SlamSnapshot.h）。
#pragma once

#include <QImage>
#include <QString>
#include <QWidget>

#include "ros/SlamSnapshot.h"

namespace flat_sim_viewer {

class SlamView : public QWidget {
  Q_OBJECT
 public:
  explicit SlamView(QWidget* parent = nullptr);

  // 存快照拷贝并触发重绘（ViewerWindow 的 tickTimer 每周期调用）
  void setSnapshot(const SlamSnapshot& snap);

  // fastbuild 回放：仅隐藏点云（false 时），地图背景 + 位姿箭头 + 决策标记照常画
  // —— 位姿不隐藏是为了看机器在哪里。切回实时/普通回放需恢复 true。
  void setShowSlamDetails(bool on) { showSlamDetails_ = on; }

  // 状态栏文本："0.010 m/px · 跟随中"
  QString statusText() const;

 Q_SIGNALS:
  void quitRequested();  // ESC / q
  void viewChanged();    // 缩放 / 平移 / 跟随切换（刷新状态栏）

 protected:
  void paintEvent(QPaintEvent* e) override;
  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void keyPressEvent(QKeyEvent* e) override;

 private:
  // 地图代数变化时重建 QImage（消息行序 row0=y 最小 → 显示需行翻转）
  void rebuildMapImage(const SlamSnapshot::Map& m);

  double pxPerM() const { return 1.0 / mPerPx_; }
  // 世界 ↔ 屏幕（y 翻转：世界 +y 朝上，屏幕 +y 朝下；视图居中）
  double sx(double wx) const { return width() / 2.0 + (wx - cx_) * pxPerM(); }
  double sy(double wy) const { return height() / 2.0 - (wy - cy_) * pxPerM(); }
  double wxOf(double s) const { return cx_ + (s - width() / 2.0) * mPerPx_; }
  double wyOf(double s) const { return cy_ - (s - height() / 2.0) * mPerPx_; }

  SlamSnapshot snap_;
  double mPerPx_ = 0.01;  // 米 / 像素（越小越放大；对应 cv_ui scalar=100）
  double cx_ = 0.0, cy_ = 0.0;  // 视图中心（世界系）
  bool follow_ = true;          // 跟随最新位姿；拖拽即脱离，v 恢复
  bool dragging_ = false;
  QPointF lastMouse_;
  bool showSlamDetails_ = true;  // false=fastbuild 回放：隐藏点云与位姿箭头

  QImage mapImg_;         // 代数缓存的地图图像
  uint32_t mapSeq_ = 0;   // 已重建的地图代数；0 = 尚无
};

}  // namespace flat_sim_viewer
