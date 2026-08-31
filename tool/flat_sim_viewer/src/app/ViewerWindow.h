// flat_sim_viewer —— 主窗口：SlamView + 状态栏 + ROS 数据泵
//
// 事件模型（全单线程，主线程）：
//   connectTimer_（1s）master 可达探测 → tryStartRos()（幂等）→ 更新连接标签
//   tickTimer_（30ms）已连时：ros::spinOnce() → snapshot() 拷贝 → 视图刷新
//             ＋ 状态栏（连接 / 位姿 / 地图）
//   Ctrl+C：roscpp 置 ros::ok()=false，tick 轮询到后关窗
#pragma once

#include <memory>

#include <QMainWindow>

#include "ros/SlamSnapshot.h"

class QLabel;
class QTimer;

namespace flat_sim_viewer {

class SlamListener;
class SlamView;

class ViewerWindow : public QMainWindow {
  Q_OBJECT
 public:
  explicit ViewerWindow(QWidget* parent = nullptr);
  ~ViewerWindow() override;  // unique_ptr<SlamListener> 析构需要完整类型，定义在 cpp

 private Q_SLOTS:
  void onTick();          // 数据泵：spin → 快照 → 视图 / 状态栏
  void onConnectPoll();   // master 探测与重连
  void onQuit();          // ESC / Ctrl+C → 关窗

 private:
  // 裸 socket 探测 ROS_MASTER_URI 端口（ros::master::check 要求已 ros::init）
  static bool masterReachable();
  bool tryStartRos();  // 幂等；成功后 listener_ 就绪
  void refreshConnLabel(const SlamSnapshot* snap);

  SlamView* view_ = nullptr;
  QLabel* connLabel_ = nullptr;  // 未连接 / 已连接 · 等待数据 / 数据正常
  QLabel* poseLabel_ = nullptr;  // 位姿（无数据时占位）
  QLabel* mapLabel_ = nullptr;   // 地图尺寸 / 分辨率 / 代数
  QString viewLabelText_;

  std::unique_ptr<SlamListener> listener_;
  bool rosRunning_ = false;
  QTimer* tickTimer_ = nullptr;
  QTimer* connectTimer_ = nullptr;
};

}  // namespace flat_sim_viewer
