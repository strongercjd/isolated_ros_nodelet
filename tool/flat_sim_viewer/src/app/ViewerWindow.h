// flat_sim_viewer —— 主窗口：SlamView + 播放条 + 状态栏 + 数据泵
//
// 数据源（source_）：
//   Live            实时订阅 /slam2d/* + /fastbuild_task/decision
//                   （SlamListener，原有行为 + 决策点覆盖层）
//   Replay          本地 rosbag 回放（BagPlayer，slam 回放：播放/暂停/进度/倍速/单步）
//   FastbuildReplay fastbuild 回放：决策日志（custom_ros_nodelet.log）步进
//                   + slam bag 地图背景 + 位姿箭头（点云由视图抑制，位姿照常显示）
// 切换规则：不触碰 ROS 生命周期（ros::init 只能一次、rosRunning_ 单向），
// 只按模式短路 onTick 的分派；切回实时时 latch 的 map 订阅即收最后一版。
//
// 事件模型（全单线程，主线程）：
//   connectTimer_（1s）master 可达探测 → tryStartRos()（幂等）→ 更新连接标签
//   tickTimer_（30ms）Live：ros::spinOnce() → snapshot() → 视图刷新
//                        Replay：player_->advance() → snapshot() → 视图刷新
//                        FastbuildReplay：fb_->advance() → snapshot() → 视图刷新
//             ＋ 状态栏（连接 / 位姿 / 地图 / 回放时间）
//   Ctrl+C：roscpp 置 ros::ok()=false，Live 分支轮询到后关窗
#pragma once

#include <memory>

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
class FastbuildPlayer;
class SlamListener;
class SlamView;

class ViewerWindow : public QMainWindow {
  Q_OBJECT
 public:
  explicit ViewerWindow(QWidget* parent = nullptr);
  ~ViewerWindow() override;  // unique_ptr<SlamListener> 析构需要完整类型，定义在 cpp

 private Q_SLOTS:
  void onTick();          // 数据泵：按 source_ 分派 tickLive / tickReplay / tickFastbuildReplay
  void onConnectPoll();   // master 探测与重连
  void onQuit();          // ESC / Ctrl+C → 关窗

  // 播放条
  void onOpenBag();       // [slam 回放] QFileDialog 选 .bag → openBagFile
  void onOpenDecision();  // [fastbuild回放] 选决策日志（custom_ros_nodelet.log）
  void onOpenSlamBag();   // [fastbuild回放] 选 slam 地图 bag（背景）
  void onPlayPause();     // Space / ▶ 按钮
  void onStepForward();   // → 键：下一 pose 帧 / 决策记录
  void onStepBackward();  // ← 键：上一 pose 帧 / 决策记录
  void onSeekPressed();   // 进度条按下：暂停回写
  void onSeekReleased();  // 进度条松开：执行 seek
  void onSpeedChanged(int idx);  // 0.5× / 1× / 2×

 private:
  enum class Source { Live, Replay, FastbuildReplay };

  // 裸 socket 探测 ROS_MASTER_URI 端口（ros::master::check 要求已 ros::init）
  static bool masterReachable();
  bool tryStartRos();  // 幂等；成功后 listener_ 就绪
  void refreshConnLabel(const SlamSnapshot* snap);
  void updatePoseMapLabels(const SlamSnapshot& snap);  // 实时 / slam 回放共用
  bool sourceReplayReady() const;  // 当前回放数据源已就绪（Replay 或 FastbuildReplay）

  void buildPlayerBar();
  void setSource(Source m);             // 只切分派与控件态，不触碰 ROS 生命周期
  void updatePlayerBarState();          // 回放控件的显示/隐藏与可用态
  void openBagFile(const QString& path);  // [slam回放] 加载（带进度）→ 切回放并自动播放
  void openSlamBagFile(const QString& path);  // [fastbuild回放] 加载 slam 背景 bag
  static QString defaultBagDir();       // 对话框起始目录：仓库 app_runtime/data/log
  void tickLive();
  void tickReplay();
  void tickFastbuildReplay();
  void refreshReplayUi();      // 回放 UI 刷新：slam 走本函数，fb 分派到 refreshFastbuildUi
  void refreshFastbuildUi();   // fb 模式：记录号 / 时间 / 按钮态 / 进度条

  SlamView* view_ = nullptr;
  QWidget* playerBar_ = nullptr;  // 底部播放条（三种模式都可见，回放才可用）
  QLabel* connLabel_ = nullptr;   // 未连接 / 已连接 · 等待数据 / 数据正常 / 回放
  QLabel* poseLabel_ = nullptr;   // 位姿（无数据时占位）
  QLabel* mapLabel_ = nullptr;    // 地图尺寸 / 分辨率 / 代数
  QLabel* replayLabel_ = nullptr; // 回放时间 / 帧号
  QString viewLabelText_;

  // 播放条控件（回放模式可用）
  QPushButton* btnLive_ = nullptr;   // checkable：实时
  QPushButton* btnReplay_ = nullptr; // checkable：slam 回放
  QPushButton* btnFbReplay_ = nullptr; // checkable：fastbuild回放
  QPushButton* btnOpen_ = nullptr;   // [slam回放] 打开 Bag…
  QPushButton* btnOpenDecision_ = nullptr; // [fastbuild回放] 打开决策日志…
  QPushButton* btnOpenSlam_ = nullptr;     // [fastbuild回放] 打开 slam Bag…
  QPushButton* btnPrev_ = nullptr;   // |◀ 上一帧 / 上一条决策
  QPushButton* btnPlay_ = nullptr;   // ▶ / ⏸
  QPushButton* btnNext_ = nullptr;   // ▶| 下一帧 / 下一条决策
  QSlider* seekSlider_ = nullptr;    // 0..1000（拖动结束才 seek）
  QComboBox* speedBox_ = nullptr;    // 0.5× / 1× / 2×

  std::unique_ptr<SlamListener> listener_;
  std::unique_ptr<BagPlayer> player_;
  std::unique_ptr<FastbuildPlayer> fb_;
  QElapsedTimer frameClock_;     // advance 的 dt 来源（单调时钟）
  bool sliderDragging_ = false;  // 拖动中暂停进度条回写
  Source source_ = Source::Live;
  bool rosRunning_ = false;
  QTimer* tickTimer_ = nullptr;
  QTimer* connectTimer_ = nullptr;
};

}  // namespace flat_sim_viewer
