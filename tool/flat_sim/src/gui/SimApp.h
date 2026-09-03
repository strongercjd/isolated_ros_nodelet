// flat_sim —— Qt GUI 控制器：QMainWindow（菜单栏） + SimView 画布 + 步进定时器
//
// 职责：
//   * QTimer（PreciseTimer，步长 ms）→ SimRunner::stepOnce() → view->update()；
//     Ctrl+C 检测（quitTimer_ 轮询 ros::ok()）。
//   * 菜单栏：文件(另存为 .fworld / 退出)、模式(查看/编辑)、工具(测量 / 格子边长 /
//     显示标注/网格线)、视图(复位)、运行(复位机器人 / 显示激光)、帮助(操作说明)。
//   * 双模式协调：进入编辑模式即暂停仿真（停步进定时器）；退出编辑若未保存则
//     弹窗 保存/放弃/取消；"另存为"走 WorldWriter 写新 .fworld，不改动原文件。
//
// 编入 flat_sim_node 可执行目标（依赖 ROS 层 SimRunner，不进 flat_sim_gui 库）。
#pragma once

#include <QMainWindow>
#include <QString>

#include <string>

#include "core/World.h"
#include "gui/SimView.h"

class QAction;
class QActionGroup;
class QTimer;

namespace flat_sim {

class SimRunner;

class SimApp : public QMainWindow {
  Q_OBJECT
 public:
  // 不立刻 show()；main 中创建后调用 start() 再进入事件循环
  SimApp(SimRunner& runner, const WorldDesc& world, QWidget* parent = nullptr);

  void start();

 protected:
  // 关窗（含 ESC/quit）时若还有未保存改动，先弹窗确认
  void closeEvent(QCloseEvent* e) override;

 private Q_SLOTS:
  void onStep();      // 仿真单步 + 重绘
  void onQuitPoll();  // Ctrl+C 检测
  void onModeAction(QAction* act);  // 模式菜单：查看/编辑互斥
  void onOpenWorld();               // 文件/打开…：加载另一 .fworld 并热切换
  void onSaveAs();                  // 文件/另存为…
  void onSetCellSize();             // 工具/设置格子边长…
  void onShowHelp();                // 帮助/操作说明

 private:
  void buildMenus();
  // 把视图 v 装进主窗口并连接信号；grid 为与运行时共享的编辑墙格层
  void wireView(SimView* v, const std::shared_ptr<GridLayer>& grid);
  // 切换目标模式（含进入编辑暂停、退出编辑的未保存确认与恢复运行）
  void applyMode(SimView::Mode m);
  void syncModeActions(SimView::Mode m);
  void setPaused(bool paused);
  QString makeTitle() const;  // 窗口标题：路径（+* 未保存标记）
  // 返回 true 表示"改动已解决，可以离开编辑"；false=用户取消留在编辑
  bool resolveEditDirty();
  // 打开"另存为"对话框并写入；成功返回 true（写完后基线已更新）
  bool saveWorldAs();

  SimRunner& runner_;
  SimView* view_ = nullptr;
  QTimer* stepTimer_ = nullptr;
  QTimer* quitTimer_ = nullptr;

  QAction* viewModeAct_ = nullptr;
  QAction* editModeAct_ = nullptr;
  QAction* measureAct_ = nullptr;
  QAction* gridAct_ = nullptr;
  QAction* annoAct_ = nullptr;
  QAction* laserAct_ = nullptr;

  std::string worldName_;    // 世界名（默认文件名用）
  std::string sourceFile_;   // 原文件路径（另存为的默认目录用）
  std::string lastSaved_;    // 最近一次保存/加载的 .fworld 路径（标题显示）
  bool inSyncActions_ = false;  // 程序化 setChecked 期间抑制模式信号
};

}  // namespace flat_sim
