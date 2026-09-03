// flat_sim —— Qt GUI 控制器实现（QMainWindow 菜单栏 + SimView + 步进定时器）
// 仅当 CMake 定义了 FLAT_SIM_HAVE_GUI（找到 Qt6）时参与编译。
#ifdef FLAT_SIM_HAVE_GUI

#include "gui/SimApp.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

#include <ros/ros.h>

#include "format/WorldLoader.h"
#include "format/WorldWriter.h"
#include "ros/SimRunner.h"

namespace flat_sim {
namespace {

QString qs(const std::string& s) { return QString::fromUtf8(s.c_str()); }

// 未保存改动确认弹窗：返回 1=保存，2=放弃，0=取消
int promptDirty(QWidget* parent, const QString& what) {
  QMessageBox box(parent);
  box.setWindowTitle(QStringLiteral("未保存的改动"));
  box.setIcon(QMessageBox::Question);
  box.setText(QStringLiteral("当前有未保存的%1改动。").arg(what));
  box.setInformativeText(QStringLiteral("保存为新 .fworld 文件后再继续？"));
  QPushButton* save = box.addButton(QStringLiteral("保存(&S)"), QMessageBox::AcceptRole);
  QPushButton* disc = box.addButton(QStringLiteral("放弃改动(&D)"), QMessageBox::DestructiveRole);
  QPushButton* cancel = box.addButton(QStringLiteral("取消(&C)"), QMessageBox::RejectRole);
  box.setDefaultButton(save);
  box.setEscapeButton(cancel);
  box.exec();
  const QAbstractButton* hit = box.clickedButton();
  if (hit == save) return 1;
  if (hit == disc) return 2;
  return 0;
}

}  // namespace

SimApp::SimApp(SimRunner& runner, const WorldDesc& world, QWidget* parent)
    : QMainWindow(parent), runner_(runner),
      worldName_(world.name), sourceFile_(world.sourceFile) {
  // ---- 画布视图 ----
  view_ = new SimView(world);
  wireView(view_, runner.editGrid());

  buildMenus();

  // ---- 仿真步进：固定步长定时器（与 headless 循环同节奏；超时不追帧）----
  stepTimer_ = new QTimer(this);
  stepTimer_->setTimerType(Qt::PreciseTimer);
  const int stepMs = std::max(1, (int)std::lround(runner_.dt() * 1000.0));
  QObject::connect(stepTimer_, &QTimer::timeout, this, &SimApp::onStep);

  // Ctrl+C 检测：roscpp 置 ros::ok()=false 但不会终止进程
  quitTimer_ = new QTimer(this);
  QObject::connect(quitTimer_, &QTimer::timeout, this, &SimApp::onQuitPoll);

  stepTimer_->start(stepMs);
  quitTimer_->start(200);

  resize(980, 720);
  setWindowTitle(makeTitle());
}

QString SimApp::makeTitle() const {
  QString t = lastSaved_.empty() ? qs(sourceFile_.empty() ? worldName_ : sourceFile_)
                                 : qs(lastSaved_);
  if (view_ && view_->dirty()) t.prepend("* ");
  t += QStringLiteral(" — flat_sim");
  return t;
}

void SimApp::start() { show(); }

// 把视图装进主窗口：与运行时共享编辑墙格层、连信号（视图/运行时热切换时重建）
void SimApp::wireView(SimView* v, const std::shared_ptr<GridLayer>& grid) {
  v->setAttribute(Qt::WA_DeleteOnClose);
  setCentralWidget(v);
  // 与仿真运行时共享同一个编辑墙格层实例（改动即时挡激光/挡运动）
  v->setEditGrid(grid);
  // 防御：视图内部若新建格层（正常不会发生），通知运行时替换
  QObject::connect(v, &SimView::editGridReplaced, this,
                   [this](std::shared_ptr<GridLayer> g) { runner_.replaceEditGrid(std::move(g)); });
  // ESC/q：走 close()（而非直接 quit），以便触发未保存提示
  QObject::connect(v, &SimView::quitRequested, this, [this] { close(); });
  // 未保存改动 → 窗口标题前缀 * 提示
  QObject::connect(v, &SimView::dirtyChanged, this, [this] { setWindowTitle(makeTitle()); });
}

// ---- 菜单栏 ----
void SimApp::buildMenus() {
  // 文件
  QMenu* fileMenu = menuBar()->addMenu(QStringLiteral("文件(&F)"));
  QAction* openAct = fileMenu->addAction(QStringLiteral("打开…  (.fworld)"));
  openAct->setShortcut(QKeySequence::Open);
  QObject::connect(openAct, &QAction::triggered, this, &SimApp::onOpenWorld);
  fileMenu->addSeparator();
  QAction* saveAct = fileMenu->addAction(QStringLiteral("另存为…  (.fworld)"));
  saveAct->setShortcut(QKeySequence::SaveAs);
  QObject::connect(saveAct, &QAction::triggered, this, &SimApp::onSaveAs);
  fileMenu->addSeparator();
  QAction* quitAct = fileMenu->addAction(QStringLiteral("退出(&Q)"));
  QObject::connect(quitAct, &QAction::triggered, this, [this] { close(); });

  // 模式（互斥）
  QMenu* modeMenu = menuBar()->addMenu(QStringLiteral("模式(&M)"));
  QActionGroup* modeGroup = new QActionGroup(this);
  modeGroup->setExclusive(true);
  viewModeAct_ = modeMenu->addAction(QStringLiteral("查看模式"));
  viewModeAct_->setCheckable(true);
  viewModeAct_->setChecked(true);
  editModeAct_ = modeMenu->addAction(QStringLiteral("编辑模式（暂停仿真）"));
  editModeAct_->setCheckable(true);
  modeGroup->addAction(viewModeAct_);
  modeGroup->addAction(editModeAct_);
  QObject::connect(modeGroup, &QActionGroup::triggered, this, &SimApp::onModeAction);

  // 工具
  QMenu* toolMenu = menuBar()->addMenu(QStringLiteral("工具(&T)"));
  measureAct_ = toolMenu->addAction(QStringLiteral("测量距离（编辑模式 Ctrl+左键 两点）"));
  measureAct_->setCheckable(true);
  measureAct_->setEnabled(false);  // 测量仅编辑模式可勾选（查看模式禁用）
  QObject::connect(measureAct_, &QAction::toggled, this, [this](bool on) {
    view_->setMeasureEnabled(on);
    // 光标跟随模式：编辑态(含关测量后)用十字，查看态用箭头
    view_->setCursor(on || view_->mode() == SimView::Mode::Edit
                         ? Qt::CrossCursor
                         : Qt::ArrowCursor);
  });
  QAction* cellAct = toolMenu->addAction(QStringLiteral("编辑墙格子边长…"));
  QObject::connect(cellAct, &QAction::triggered, this, &SimApp::onSetCellSize);
  toolMenu->addSeparator();
  gridAct_ = toolMenu->addAction(QStringLiteral("显示编辑网格线"));
  gridAct_->setCheckable(true);
  gridAct_->setChecked(view_->gridVisible());
  // 经 this + view_ 成员转发，视图热切换后无需重建连接
  QObject::connect(gridAct_, &QAction::toggled, this,
                   [this](bool on) { view_->setGridVisible(on); });
  annoAct_ = toolMenu->addAction(QStringLiteral("显示测量标注"));
  annoAct_->setCheckable(true);
  annoAct_->setChecked(true);
  QObject::connect(annoAct_, &QAction::toggled, this,
                   [this](bool on) { view_->setAnnotationsVisible(on); });

  // 视图
  QMenu* viewMenu = menuBar()->addMenu(QStringLiteral("视图(&V)"));
  QAction* fitAct = viewMenu->addAction(QStringLiteral("整窗适配（居中缩放）"));
  QObject::connect(fitAct, &QAction::triggered, this, [this] { view_->fitViewToWorld(); });

  // 运行
  QMenu* runMenu = menuBar()->addMenu(QStringLiteral("运行(&R)"));
  QAction* resetAct = runMenu->addAction(QStringLiteral("复位机器人"));
  QObject::connect(resetAct, &QAction::triggered, this, [this] { runner_.reset(); });
  laserAct_ = runMenu->addAction(QStringLiteral("显示激光束"));
  laserAct_->setCheckable(true);
  laserAct_->setChecked(view_->laserVisible());
  QObject::connect(laserAct_, &QAction::toggled, this,
                   [this](bool on) { view_->setLaserVisible(on); });

  // 帮助
  QMenu* helpMenu = menuBar()->addMenu(QStringLiteral("帮助(&H)"));
  QAction* helpAct = helpMenu->addAction(QStringLiteral("操作说明"));
  QObject::connect(helpAct, &QAction::triggered, this, &SimApp::onShowHelp);
}

// ---- 模式切换 ----
void SimApp::onModeAction(QAction* act) {
  applyMode(act == editModeAct_ ? SimView::Mode::Edit : SimView::Mode::View);
}

void SimApp::applyMode(SimView::Mode m) {
  if (m == SimView::Mode::Edit) {
    // 进入编辑：暂停仿真；测量只允许在编辑模式勾选（左键仍画墙，Ctrl+左键测距）
    setPaused(true);
    view_->setMode(SimView::Mode::Edit);
    measureAct_->setEnabled(true);
  } else {
    // 退出编辑：未保存改动先确认
    if (view_->mode() == SimView::Mode::Edit && !resolveEditDirty()) {
      syncModeActions(SimView::Mode::Edit);  // 留在编辑
      return;
    }
    setPaused(false);
    measureAct_->setChecked(false);  // 查看模式不能测量：取消勾选并禁用
    measureAct_->setEnabled(false);
    view_->setMode(SimView::Mode::View);
  }
  syncModeActions(m);
  setWindowTitle(makeTitle());
}

void SimApp::syncModeActions(SimView::Mode m) {
  inSyncActions_ = true;
  editModeAct_->setChecked(m == SimView::Mode::Edit);
  viewModeAct_->setChecked(m == SimView::Mode::View);
  inSyncActions_ = false;
}

void SimApp::setPaused(bool paused) {
  if (paused) {
    if (stepTimer_->isActive()) stepTimer_->stop();
  } else if (!stepTimer_->isActive()) {
    stepTimer_->start();
  }
}

// 返回 true = 改动已解决（保存/放弃），可离开编辑；false = 用户取消留在编辑
bool SimApp::resolveEditDirty() {
  if (!view_->dirty()) return true;
  const int r = promptDirty(this, QStringLiteral("墙与测量"));
  if (r == 1) return saveWorldAs();       // 保存成功才算解决
  if (r == 2) {
    view_->discardChanges();
    return true;
  }
  return false;                            // 取消
}

void SimApp::closeEvent(QCloseEvent* e) {
  if (view_->dirty() && !resolveEditDirty()) {
    e->ignore();
    return;
  }
  e->accept();
}

// ---- 文件 ----
bool SimApp::saveWorldAs() {
  QString dir = lastSaved_.empty()
                    ? (sourceFile_.empty() ? QDir::currentPath()
                                           : QFileInfo(qs(sourceFile_)).absolutePath())
                    : QFileInfo(qs(lastSaved_)).absolutePath();
  QString defName = QString::fromUtf8((worldName_ + ".fworld").c_str());
  QString path = QFileDialog::getSaveFileName(this, QStringLiteral("另存为新 .fworld"),
                                              dir + QLatin1Char('/') + defName,
                                              QStringLiteral("flat_sim 世界 (*.fworld)"));
  if (path.isEmpty()) return false;
  if (!path.endsWith(QStringLiteral(".fworld"), Qt::CaseInsensitive)) path += QStringLiteral(".fworld");

  WorldDesc snap = view_->worldForSave();
  snap.name = worldName_.empty() ? "world" : worldName_;
  snap.sourceFile = path.toStdString();
  const std::string err = saveWorldFile(snap, path.toStdString());
  if (!err.empty()) {
    QMessageBox::warning(this, QStringLiteral("保存失败"), qs(err));
    return false;
  }
  view_->markClean();
  lastSaved_ = path.toStdString();
  setWindowTitle(makeTitle());
  std::printf("[flat_sim] 已另存世界: %s（原文件未改动）\n", path.toStdString().c_str());
  return true;
}

void SimApp::onSaveAs() { saveWorldAs(); }

void SimApp::onOpenWorld() {
  // 当前有未保存改动 → 与退出/切回查看一致弹窗（保存 / 放弃 / 取消）
  if (view_ && view_->dirty() && !resolveEditDirty()) return;

  const QString dir = sourceFile_.empty() ? QDir::currentPath()
                                          : QFileInfo(qs(sourceFile_)).absolutePath();
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("打开 .fworld 世界"), dir,
      QStringLiteral("flat_sim 世界 (*.fworld)"));
  if (path.isEmpty()) return;

  WorldDesc loaded;
  try {
    loaded = loadWorld(path.toStdString());
  } catch (const std::exception& ex) {
    QMessageBox::warning(this, QStringLiteral("打开失败"),
                         QStringLiteral("无法解析该文件：\n") + qs(ex.what()));
    return;
  }

  // ---- 热切换：先停步进定时器，重建 sim_/桥后再启动，避免并发步进 ----
  if (stepTimer_) stepTimer_->stop();
  const std::shared_ptr<GridLayer> newGrid = runner_.loadWorld(loaded);

  // 重建视图：旧视图连同旧格层/旧 sim 引用一起废弃（deleteLater 安全出事件循环）
  SimView* old = view_;
  view_ = nullptr;
  old->deleteLater();
  view_ = new SimView(loaded);
  wireView(view_, newGrid);

  // 元信息切到新文件；回到查看模式（编辑墙若带 active 则以格子显示）
  worldName_ = loaded.name;
  sourceFile_ = loaded.sourceFile;
  lastSaved_.clear();
  view_->setSimulator(&runner_.sim());

  // 视图开关与动作状态对齐（动作连接经 view_ 成员，无需重建）
  measureAct_->setChecked(false);
  measureAct_->setEnabled(false);  // 新打开的世界回到查看模式，测量仍禁用
  view_->setGridVisible(gridAct_->isChecked());
  view_->setAnnotationsVisible(annoAct_->isChecked());
  view_->setLaserVisible(laserAct_->isChecked());
  syncModeActions(SimView::Mode::View);
  view_->fitViewToWorld();

  setPaused(false);  // 恢复运行（进入编辑时的暂停一并解除）
  setWindowTitle(makeTitle());
  std::printf("[flat_sim] 已打开世界: %s\n", path.toStdString().c_str());
}

void SimApp::onSetCellSize() {
  bool ok = false;
  const double v = QInputDialog::getDouble(
      this, QStringLiteral("编辑墙格子边长"),
      QStringLiteral("设置编辑模式的格子边长（米，范围 0.05–1.0；\n已有墙体将按格心迁移到新网格）："),
      view_->cellSize(), 0.05, 1.0, 3, &ok);
  if (!ok) return;
  view_->setCellSize(v);
  setWindowTitle(makeTitle());
}

void SimApp::onShowHelp() {
  const QString text =
      QStringLiteral(
          "<b>flat_sim — 双模式仿真视图</b><br><br>"
          "<b>查看模式</b>（默认）：<br>"
          "· 滚轮：以光标为中心缩放<br>"
          "· 左键拖拽 / 中键拖拽：平移<br>"
          "· 查看模式不可测量；测量只在编辑模式提供<br>"
          "· 顶部/左侧刻度条 + 右下角比例尺可判断区域尺寸<br><br>"
          "<b>编辑模式</b>（菜单「模式 → 编辑模式」，进入后仿真自动暂停）：<br>"
          "· 区域被划分为正交小格子（默认 0.1 m）；原墙/箱体会被栅格化成同样的小格子<br>"
          "· 左键单击/按住滑动：经过的格子变成<b>墙体</b>（挡住激光与机器人）<br>"
          "· 右键单击/按住滑动：清除该格的占用——在<b>原墙上也能抠出真正的洞</b>，"
          "激光与小车可以穿过（保存后再次打开依然如此）<br>"
          "· Ctrl+左/右拖拽：平移画面；滚轮：缩放<br>"
          "· 原几何与机器人在编辑中不会被修改或移动<br>"
          "· 墙体统一为灰色（与原障碍同色）：看网格线即可区分「被栅格成格子的墙」"
          "与平滑的原几何；编辑保存后再打开的文件以格子呈现<br>"
          "· 「工具 → 编辑墙格子边长…」可调格子大小（0.05–1 m），已有墙会迁移<br><br>"
          "<b>测量标注</b>（仅编辑模式，随 .fworld 保存，不影响仿真）：<br>"
          "· 勾选「工具 → 测量距离」后：<b>Ctrl+左键</b> 依次点两点，"
          "即生成带长度数值的测量标注<br>"
          "· 未勾选时 Ctrl+左键 仍用于平移；测量勾选不影响左键画墙<br>"
          "· 删除单条标注：<b>Ctrl+右键</b> 点在该条标注线上即可（查看/编辑都可用）<br><br>"
          "<b>文件打开与保存</b>：<br>"
          "· 「文件 → 打开…」可切换另一 <i>.fworld</i>（若当前有未保存改动会先弹窗确认）<br>"
          "· 改动只在你点「文件 → 另存为…」时写入磁盘，保存为新的 <i>.fworld</i> 文件，"
          "原文件不会被改动<br>"
          "· 从编辑切回查看、打开其他文件、或关闭窗口时，若还有未保存改动会弹窗："
          "保存 / 放弃 / 取消<br>"
          "· ESC 或 q：退出程序");
  QMessageBox::about(this, QStringLiteral("flat_sim 操作说明"), text);
}

// ---- 定时器 ----
void SimApp::onStep() {
  if (!runner_.stepOnce()) {
    close();  // 仿真请求退出（Ctrl+C 后）→ 走关窗路径
    return;
  }
  view_->setSimulator(&runner_.sim());  // 内部触发重绘
}

void SimApp::onQuitPoll() {
  if (runner_.rosRunning() && !ros::ok()) close();
}

}  // namespace flat_sim

#endif  // FLAT_SIM_HAVE_GUI
