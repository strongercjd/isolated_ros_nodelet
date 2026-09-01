// flat_sim_viewer —— 主窗口实现
#include "app/ViewerWindow.h"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QComboBox>
#include <QCoreApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QShortcut>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include "replay/BagPlayer.h"
#include "ros/SlamListener.h"
#include "view/SlamView.h"

namespace flat_sim_viewer {

namespace {

// 秒 → "mm:ss.s"（回放时间标签）
QString formatTimeSec(double sec) {
  if (sec < 0 || !std::isfinite(sec)) sec = 0;
  const int m = (int)(sec / 60.0);
  const double s = sec - m * 60.0;
  return QString::asprintf("%02d:%04.1f", m, s);
}

}  // namespace

ViewerWindow::ViewerWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(QString::fromUtf8(
      "flat_sim_viewer — SLAM 建图查看（滚轮缩放 / 拖拽平移 / v 跟随 / ESC 退出）"));
  resize(900, 700);

  buildPlayerBar();

  // central = 视图（拉伸） + 底部播放条
  view_ = new SlamView(this);
  view_->setFocus();  // 初始就把按键焦点给视图
  QWidget* central = new QWidget(this);
  QVBoxLayout* vlay = new QVBoxLayout(central);
  vlay->setContentsMargins(0, 0, 0, 0);
  vlay->setSpacing(0);
  vlay->addWidget(view_, 1);
  vlay->addWidget(playerBar_);
  setCentralWidget(central);

  connLabel_ = new QLabel(this);
  poseLabel_ = new QLabel(this);
  mapLabel_ = new QLabel(this);
  viewLabelText_ = view_->statusText();
  for (QLabel* l : {connLabel_, poseLabel_, mapLabel_}) {
    l->setMargin(4);
    statusBar()->addWidget(l);
  }
  refreshConnLabel(nullptr);

  QObject::connect(view_, &SlamView::quitRequested, this, &ViewerWindow::onQuit);
  QObject::connect(view_, &SlamView::viewChanged, this, [this]() {
    viewLabelText_ = view_->statusText();
  });

  // 快捷键（QShortcut 拦截优先于 SlamView::keyPressEvent；ESC/q/v 仍归视图）
  {
    auto* sc = new QShortcut(QKeySequence(Qt::Key_Space), this);
    QObject::connect(sc, &QShortcut::activated, this, &ViewerWindow::onPlayPause);
    sc = new QShortcut(QKeySequence(Qt::Key_Right), this);
    QObject::connect(sc, &QShortcut::activated, this, &ViewerWindow::onStepForward);
    sc = new QShortcut(QKeySequence(Qt::Key_Left), this);
    QObject::connect(sc, &QShortcut::activated, this, &ViewerWindow::onStepBackward);
  }

  connectTimer_ = new QTimer(this);
  connectTimer_->setInterval(1000);  // 未连接时每秒重探 master
  QObject::connect(connectTimer_, &QTimer::timeout, this, &ViewerWindow::onConnectPoll);

  tickTimer_ = new QTimer(this);
  tickTimer_->setTimerType(Qt::PreciseTimer);
  tickTimer_->setInterval(30);  // ~33 Hz，可视化足够
  QObject::connect(tickTimer_, &QTimer::timeout, this, &ViewerWindow::onTick);

  connectTimer_->start();
  tickTimer_->start();
  updatePlayerBarState();
}

ViewerWindow::~ViewerWindow() = default;

// ---------------------------------------------------------------- 播放条

void ViewerWindow::buildPlayerBar() {
  playerBar_ = new QWidget(this);
  QHBoxLayout* h = new QHBoxLayout(playerBar_);
  h->setContentsMargins(6, 2, 6, 2);
  h->setSpacing(4);

  btnLive_ = new QPushButton(QString::fromUtf8("实时"), playerBar_);
  btnReplay_ = new QPushButton(QString::fromUtf8("回放"), playerBar_);
  btnLive_->setCheckable(true);
  btnReplay_->setCheckable(true);
  btnLive_->setChecked(true);
  auto* group = new QButtonGroup(playerBar_);  // 互斥，随 playerBar_ 释放
  group->setExclusive(true);
  group->addButton(btnLive_);
  group->addButton(btnReplay_);
  QObject::connect(btnLive_, &QAbstractButton::toggled, this, [this](bool on) {
    if (on) setSource(Source::Live);
  });
  QObject::connect(btnReplay_, &QAbstractButton::toggled, this, [this](bool on) {
    if (!on) return;
    setSource(Source::Replay);
    if (!(player_ && player_->isOpen())) onOpenBag();  // 切入回放默认选文件
  });

  btnOpen_ = new QPushButton(QString::fromUtf8("打开 Bag…"), playerBar_);
  QObject::connect(btnOpen_, &QAbstractButton::clicked, this, &ViewerWindow::onOpenBag);

  btnPrev_ = new QPushButton(QString::fromUtf8("|◀"), playerBar_);
  btnPlay_ = new QPushButton(QString::fromUtf8("▶"), playerBar_);
  btnNext_ = new QPushButton(QString::fromUtf8("▶|"), playerBar_);
  QObject::connect(btnPrev_, &QAbstractButton::clicked, this, &ViewerWindow::onStepBackward);
  QObject::connect(btnPlay_, &QAbstractButton::clicked, this, &ViewerWindow::onPlayPause);
  QObject::connect(btnNext_, &QAbstractButton::clicked, this, &ViewerWindow::onStepForward);

  seekSlider_ = new QSlider(Qt::Horizontal, playerBar_);
  seekSlider_->setRange(0, 1000);
  seekSlider_->setValue(0);
  QObject::connect(seekSlider_, &QSlider::sliderPressed, this, &ViewerWindow::onSeekPressed);
  QObject::connect(seekSlider_, &QSlider::sliderReleased, this, &ViewerWindow::onSeekReleased);

  speedBox_ = new QComboBox(playerBar_);
  speedBox_->addItem("0.5×");
  speedBox_->addItem("1×");
  speedBox_->addItem("2×");
  speedBox_->setCurrentIndex(1);
  QObject::connect(speedBox_, &QComboBox::currentIndexChanged, this,
                   &ViewerWindow::onSpeedChanged);

  replayLabel_ = new QLabel(QString::fromUtf8("回放：未加载"), playerBar_);

  h->addWidget(btnLive_);
  h->addWidget(btnReplay_);
  h->addWidget(btnOpen_);
  h->addWidget(btnPrev_);
  h->addWidget(btnPlay_);
  h->addWidget(btnNext_);
  h->addWidget(seekSlider_, 1);  // 进度条吃掉剩余宽度
  h->addWidget(speedBox_);
  h->addWidget(replayLabel_);
}

void ViewerWindow::setSource(Source m) {
  source_ = m;
  if (m == Source::Live && player_) player_->pause();
  updatePlayerBarState();
  refreshConnLabel(nullptr);
}

void ViewerWindow::updatePlayerBarState() {
  // 实时模式隐藏全部回放控件（只留 实时|回放 切换）；回放模式显示，未加载时置灰
  const bool replay = source_ == Source::Replay;
  const bool ready = replay && player_ && player_->isOpen();
  for (QWidget* w : {static_cast<QWidget*>(btnOpen_), static_cast<QWidget*>(btnPrev_),
                     static_cast<QWidget*>(btnPlay_), static_cast<QWidget*>(btnNext_),
                     static_cast<QWidget*>(seekSlider_), static_cast<QWidget*>(speedBox_),
                     static_cast<QWidget*>(replayLabel_)}) {
    w->setVisible(replay);
  }
  btnOpen_->setEnabled(replay);
  btnPlay_->setEnabled(ready);
  btnPrev_->setEnabled(ready);
  btnNext_->setEnabled(ready);
  seekSlider_->setEnabled(ready);
  speedBox_->setEnabled(ready);
}

QString ViewerWindow::defaultBagDir() {
  // .../tool/flat_sim_viewer/install/bin → 上 4 级为仓库根
  QDir dir = QCoreApplication::applicationDirPath();
  if (dir.cdUp() && dir.cdUp() && dir.cdUp() && dir.cdUp()) {
    const QString cand = dir.filePath("app_runtime/data/log");
    if (QFileInfo(cand).isDir()) return cand;
  }
  return QDir::homePath();
}

void ViewerWindow::onOpenBag() {
  const QString path = QFileDialog::getOpenFileName(
      this, QString::fromUtf8("打开 SLAM 日志（map_log_*.bag）"), defaultBagDir(),
      "ROS Bag (*.bag)");
  if (!path.isEmpty()) openBagFile(path);
}

void ViewerWindow::openBagFile(const QString& path) {
  auto candidate = std::make_unique<BagPlayer>();
  // 无取消按钮：建索引一般 1–3s，取消语义（半开文件）不值得复杂化
  QProgressDialog dlg(
      QString::fromUtf8("正在加载 %1 …").arg(QFileInfo(path).fileName()),
      QString(), 0, 1000, this);
  dlg.setWindowModality(Qt::WindowModal);
  dlg.setMinimumDuration(200);
  const bool ok = candidate->open(path.toStdString(), [&dlg](double p) {
    dlg.setValue((int)(p * 1000.0));
    QCoreApplication::processEvents();
  });
  if (!ok) {
    QMessageBox::warning(this, QString::fromUtf8("打开 Bag 失败"),
                         path + "\n" + QString::fromStdString(candidate->error()));
    if (source_ == Source::Replay) {
      // 回放模式下加载失败：退回实时，避免停在无数据状态
      btnLive_->setChecked(true);
      setSource(Source::Live);
    }
    return;
  }

  player_ = std::move(candidate);
  btnReplay_->setChecked(true);  // 可能已在回放模式（toggled 不触发）
  setSource(Source::Replay);
  player_->play();  // 加载完成直接播放
  frameClock_.start();
  view_->setSnapshot(player_->snapshot());  // 第一帧立即上屏
  refreshReplayUi();
}

void ViewerWindow::onPlayPause() {
  if (source_ != Source::Replay || !(player_ && player_->isOpen())) return;
  if (player_->isPlaying())
    player_->pause();
  else
    player_->play();
  frameClock_.restart();  // 防暂停期间累积的 dt 在恢复瞬间快进
  refreshReplayUi();
}

void ViewerWindow::onStepForward() {
  if (source_ != Source::Replay || !(player_ && player_->isOpen())) return;
  player_->stepForward();
  frameClock_.restart();
  refreshReplayUi();
}

void ViewerWindow::onStepBackward() {
  if (source_ != Source::Replay || !(player_ && player_->isOpen())) return;
  player_->stepBackward();
  frameClock_.restart();
  refreshReplayUi();
}

void ViewerWindow::onSeekPressed() { sliderDragging_ = true; }

void ViewerWindow::onSeekReleased() {
  sliderDragging_ = false;
  if (!(player_ && player_->isOpen())) return;
  const double dur = player_->durationSec();
  if (dur <= 0) return;
  const double frac = seekSlider_->value() / 1000.0;
  player_->seekToTime(player_->bagStart() + ros::Duration(frac * dur));
  frameClock_.restart();
  refreshReplayUi();
}

void ViewerWindow::onSpeedChanged(int idx) {
  const double speeds[] = {0.5, 1.0, 2.0};
  if (player_) player_->setSpeed(speeds[idx >= 0 && idx <= 2 ? idx : 1]);
  refreshReplayUi();
}

void ViewerWindow::refreshReplayUi() {
  if (!(player_ && player_->isOpen())) {
    replayLabel_->setText(QString::fromUtf8("回放：未加载"));
    return;
  }
  const double dur = player_->durationSec();
  const double cur = (player_->playhead() - player_->bagStart()).toSec();
  const size_t frame = player_->frameIndex() + 1;
  replayLabel_->setText(QString::fromUtf8("%1 / %2 · 帧 %3/%4 · %5×")
                            .arg(formatTimeSec(cur), formatTimeSec(dur))
                            .arg((qulonglong)frame)
                            .arg((qulonglong)player_->frameCount())
                            .arg(player_->speed(), 0, 'f', 1));
  btnPlay_->setText(player_->isPlaying() ? QString::fromUtf8("⏸")
                                         : QString::fromUtf8("▶"));
  if (!sliderDragging_ && dur > 0)
    seekSlider_->setValue((int)(cur / dur * 1000.0));
}

// ---------------------------------------------------------------- 连接（实时）

bool ViewerWindow::masterReachable() {
  const char* env = std::getenv("ROS_MASTER_URI");
  std::string uri = env && *env ? env : "http://127.0.0.1:11311";
  const size_t slash = uri.find("//");
  std::string hostport = slash == std::string::npos ? uri : uri.substr(slash + 2);
  const size_t colon = hostport.find(':');
  const std::string host = colon == std::string::npos ? hostport : hostport.substr(0, colon);
  const std::string port = colon == std::string::npos ? "11311" : hostport.substr(colon + 1);
  if (host.empty() || port.empty()) return false;

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
    if (res) freeaddrinfo(res);
    return false;
  }
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  bool ok = false;
  if (fd >= 0) {
    // ::connect / ::close：避开 QObject::connect / QWidget::close 的名字遮蔽
    ok = ::connect(fd, res->ai_addr, res->ai_addrlen) == 0;
    ::close(fd);
  }
  freeaddrinfo(res);
  return ok;
}

bool ViewerWindow::tryStartRos() {
  if (rosRunning_) return true;
  if (!masterReachable()) return false;
  // NoRosout：不挂 rosout appender，master 异常时不刷屏（同 flat_sim 的理由）
  int argc = 1;
  char arg0[] = "flat_sim_viewer";
  char* argv[] = {arg0, nullptr};
  ros::init(argc, argv, "flat_sim_viewer", ros::init_options::NoRosout);
  listener_.reset(new SlamListener());
  rosRunning_ = true;
  std::fprintf(stderr, "[flat_sim_viewer] rosmaster 就绪（%s），已订阅 /slam2d/*\n",
               std::getenv("ROS_MASTER_URI") ? std::getenv("ROS_MASTER_URI") : "?");
  return true;
}

void ViewerWindow::refreshConnLabel(const SlamSnapshot* snap) {
  if (source_ == Source::Replay) {
    if (player_ && player_->isOpen()) {
      connLabel_->setText(QString::fromUtf8("● 回放：%1").arg(
          QString::fromStdString(player_->fileName().substr(
              player_->fileName().find_last_of('/') + 1))));
    } else {
      connLabel_->setText(QString::fromUtf8("● 回放：未加载（点击 打开 Bag… 按钮）"));
    }
    return;
  }
  if (!rosRunning_) {
    connLabel_->setText(QString::fromUtf8("● 未连接 rosmaster（重试中）"));
    return;
  }
  const bool hasData = snap && (snap->map.seq > 0 || snap->hasPose);
  connLabel_->setText(hasData ? QString::fromUtf8("● 数据正常")
                              : QString::fromUtf8("● 已连接 · 等待 SLAM 数据"));
}

void ViewerWindow::onConnectPoll() {
  if (rosRunning_) return;
  tryStartRos();
  if (source_ == Source::Live) refreshConnLabel(nullptr);
}

// ---------------------------------------------------------------- 数据泵

void ViewerWindow::onTick() {
  if (source_ == Source::Replay) {
    tickReplay();
  } else {
    tickLive();
  }
}

void ViewerWindow::tickLive() {
  if (!rosRunning_) {  // 未连接：tick 不做事，探测在 connectTimer_
    return;
  }
  if (!ros::ok()) {  // Ctrl+C（ROS 已接管信号）
    close();
    return;
  }
  ros::spinOnce();  // 派发 /slam2d/* 回调（与 SlamListener 同队列）
  const SlamSnapshot snap = listener_->snapshot();
  view_->setSnapshot(snap);

  refreshConnLabel(&snap);
  updatePoseMapLabels(snap);
  statusBar()->showMessage(viewLabelText_);
}

void ViewerWindow::tickReplay() {
  if (!(player_ && player_->isOpen())) return;
  if (player_->isPlaying()) {
    const double dt = frameClock_.restart() / 1000.0;
    player_->advance(dt);
  } else {
    frameClock_.restart();  // 时钟保持新鲜，恢复播放瞬间 dt 不跳变
  }
  const SlamSnapshot& snap = player_->snapshot();
  view_->setSnapshot(snap);
  updatePoseMapLabels(snap);
  refreshReplayUi();
}

void ViewerWindow::updatePoseMapLabels(const SlamSnapshot& snap) {
  if (snap.hasPose) {
    poseLabel_->setText(QString::asprintf("x=%+.2f  y=%+.2f  θ=%+.1f°", snap.poseX,
                                          snap.poseY, snap.poseYaw * 180.0 / M_PI));
  } else {
    poseLabel_->setText(QString::fromUtf8("x=—  y=—  θ=—"));
  }
  if (snap.map.seq > 0) {
    mapLabel_->setText(QString::asprintf("%d×%d @%.2fm  seq=%u", snap.map.width,
                                         snap.map.height, snap.map.resolution,
                                         snap.map.seq));
  } else {
    mapLabel_->setText(QString::fromUtf8("地图：未收到"));
  }
}

void ViewerWindow::onQuit() { close(); }

}  // namespace flat_sim_viewer
