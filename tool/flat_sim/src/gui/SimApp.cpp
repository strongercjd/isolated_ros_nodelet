// flat_sim —— Qt GUI 控制器实现
// 仅当 CMake 定义了 FLAT_SIM_HAVE_GUI（找到 Qt6）时参与编译。
#ifdef FLAT_SIM_HAVE_GUI

#include "gui/SimApp.h"

#include <QApplication>
#include <QTimer>

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include "gui/SimView.h"
#include "ros/SimRunner.h"

namespace flat_sim {

SimApp::SimApp(SimRunner& runner, const WorldDesc& world, QObject* parent)
    : QObject(parent), runner_(runner) {
  view_ = new SimView(world);
  view_->setAttribute(Qt::WA_DeleteOnClose);  // 关窗即销毁，不留悬空视图
  view_->setWindowTitle(QString::fromUtf8(
      (world.name + " - flat_sim (ESC=quit l=laser r=reset)").c_str()));
  QObject::connect(view_, &SimView::resetRequested, this, &SimApp::onReset);
  QObject::connect(view_, &SimView::quitRequested, qApp, &QApplication::quit);

  // 仿真步进：固定步长定时器（与 headless 循环同节奏；超时不追帧）
  stepTimer_ = new QTimer(this);
  stepTimer_->setTimerType(Qt::PreciseTimer);
  const int stepMs = std::max(1, (int)std::lround(runner_.dt() * 1000.0));
  QObject::connect(stepTimer_, &QTimer::timeout, this, &SimApp::onStep);

  // Ctrl+C 检测：roscpp 置 ros::ok()=false 但不会终止进程
  quitTimer_ = new QTimer(this);
  QObject::connect(quitTimer_, &QTimer::timeout, this, &SimApp::onQuitPoll);

  stepTimer_->start(stepMs);
  quitTimer_->start(200);
}

void SimApp::start() { view_->show(); }

void SimApp::onStep() {
  if (!runner_.stepOnce()) {
    qApp->quit();
    return;
  }
  view_->setSimulator(&runner_.sim());  // 内部触发重绘
}

void SimApp::onQuitPoll() {
  if (runner_.rosRunning() && !ros::ok()) qApp->quit();
}

void SimApp::onReset() { runner_.reset(); }

}  // namespace flat_sim

#endif  // FLAT_SIM_HAVE_GUI
