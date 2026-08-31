// flat_sim_viewer —— 主窗口实现
#include "app/ViewerWindow.h"

#include <QLabel>
#include <QStatusBar>
#include <QTimer>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include "ros/SlamListener.h"
#include "view/SlamView.h"

namespace flat_sim_viewer {

ViewerWindow::ViewerWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(QString::fromUtf8(
      "flat_sim_viewer — SLAM 建图查看（滚轮缩放 / 拖拽平移 / v 跟随 / ESC 退出）"));
  resize(900, 700);

  view_ = new SlamView(this);
  view_->setFocus();  // 初始就把按键焦点给视图
  setCentralWidget(view_);

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

  connectTimer_ = new QTimer(this);
  connectTimer_->setInterval(1000);  // 未连接时每秒重探 master
  QObject::connect(connectTimer_, &QTimer::timeout, this, &ViewerWindow::onConnectPoll);

  tickTimer_ = new QTimer(this);
  tickTimer_->setTimerType(Qt::PreciseTimer);
  tickTimer_->setInterval(30);  // ~33 Hz，可视化足够
  QObject::connect(tickTimer_, &QTimer::timeout, this, &ViewerWindow::onTick);

  connectTimer_->start();
  tickTimer_->start();
}

ViewerWindow::~ViewerWindow() = default;

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
  refreshConnLabel(nullptr);
}

void ViewerWindow::onTick() {
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
  statusBar()->showMessage(viewLabelText_);
}

void ViewerWindow::onQuit() { close(); }

}  // namespace flat_sim_viewer
