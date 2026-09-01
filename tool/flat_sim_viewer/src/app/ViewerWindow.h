// flat_sim_viewer —— 主窗口：SlamView + 播放条 + 状态栏 + 数据泵
//
// 数据源（source_）：
//   Live   实时订阅 /slam2d/*（SlamListener，原有行为）
//   Replay 本地 rosbag 回放（BagPlayer，完整播放器：播放/暂停/进度/倍速/单步）
// 切换规则：不触碰 ROS 生命周期（ros::init 只能一次、rosRunning_ 单向），
// 只按模式短路 onTick 的分派；切回实时时 latch 的 map 订阅即收最后一版。
//
// 事件模型（全单线程，主线程）：
//   connectTimer_（1s）master 可达探测 → tryStartRos()（幂等）→ 更新连接标签
//   tickTimer_（30ms）Live：ros::spinOnce() → snapshot() → 视图刷新
//                        Replay：player_->advance() → snapshot() → 视图刷新
//             ＋ 状态栏（连接 / 位姿 / 地图 / 回放时间）
//   Ctrl+C：roscpp 置 ros::ok()=false，Live 分支轮询到后关窗
#pragma once

#include <memory>
#include <string>

#include <QElapsedTimer>
#include <QMainWindow>

#include "ros/SlamSnapshot.h"

class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;

namespace flat_sim_viewer {

class BagPlayer;
class SlamListener;
class SlamView;

class ViewerWindow : public QMainWindow {
  Q_OBJECT
 public:
  // initialBag 非空：启动后直接进回放模式并加载该文件（--bag 参数）
  explicit ViewerWindow(QWidget* parent = nullptr,
                        const std::string& initialBag = std::string());
  ~ViewerWindow() override;  // unique_ptr<SlamListener> 析构需要完整类型，定义在 cpp

 private Q_SLOTS:
  void onTick();          // 数据泵：按 source_ 分派 tickLive / tickReplay
  void onConnectPoll();   // master 探测与重连
  void onQuit();          // ESC / Ctrl+C → 关窗

  // 播放条
  void onOpenBag();       // QFileDialog 选 .bag → openBagFile
  void onPlayPause();     // Space / ▶ 按钮
  void onStepForward();   // → 键：下一 pose 帧
  void onStepBackward();  // ← 键：上一 pose 帧
  void onSeekPressed();   // 进度条按下：暂停回写
  void onSeekReleased();  // 进度条松开：执行 seek
  void onSpeedChanged(int idx);  // 0.5× / 1× / 2×

 private:
  enum class Source { Live, Replay };

  // 裸 socket 探测 ROS_MASTER_URI 端口（ros::master::check 要求已 ros::init）
  static bool masterReachable();
  bool tryStartRos();  // 幂等；成功后 listener_ 就绪
  void refreshConnLabel(const SlamSnapshot* snap);
  void updatePoseMapLabels(const SlamSnapshot& snap);  // 两种模式共用

  void buildPlayerBar();
  void setSource(Source m);             // 只切分派与控件态，不触碰 ROS 生命周期
  void updatePlayerBarState();          // 回放控件的显示/隐藏与可用态
  void openBagFile(const QString& path);  // 加载（带进度）→ 成功切回放并自动播放
  static QString defaultBagDir();       // 对话框起始目录：仓库 app_runtime/data/log
  void tickLive();
  void tickReplay();
  void refreshReplayUi();  // 播放按钮态 / 进度条回写 / 时间与帧号标签

  SlamView* view_ = nullptr;
  QWidget* playerBar_ = nullptr;  // 底部播放条（两种模式都可见，回放才可用）
  QLabel* connLabel_ = nullptr;   // 未连接 / 已连接 · 等待数据 / 数据正常 / 回放
  QLabel* poseLabel_ = nullptr;   // 位姿（无数据时占位）
  QLabel* mapLabel_ = nullptr;    // 地图尺寸 / 分辨率 / 代数
  QLabel* replayLabel_ = nullptr; // 回放时间 / 帧号
  QString viewLabelText_;

  // 播放条控件（回放模式可用）
  QPushButton* btnLive_ = nullptr;   // checkable：实时
  QPushButton* btnReplay_ = nullptr; // checkable：回放
  QPushButton* btnOpen_ = nullptr;   // 打开 Bag…
  QPushButton* btnPrev_ = nullptr;   // |◀ 上一帧
  QPushButton* btnPlay_ = nullptr;   // ▶ / ⏸
  QPushButton* btnNext_ = nullptr;   // ▶| 下一帧
  QSlider* seekSlider_ = nullptr;    // 0..1000（拖动结束才 seek）
  QComboBox* speedBox_ = nullptr;    // 0.5× / 1× / 2×

  std::unique_ptr<SlamListener> listener_;
  std::unique_ptr<BagPlayer> player_;
  QElapsedTimer frameClock_;     // advance 的 dt 来源（单调时钟）
  bool sliderDragging_ = false;  // 拖动中暂停进度条回写
  Source source_ = Source::Live;
  bool rosRunning_ = false;
  QTimer* tickTimer_ = nullptr;
  QTimer* connectTimer_ = nullptr;
};

}  // namespace flat_sim_viewer
