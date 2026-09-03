// flat_sim —— 2D 俯视仿真视图（Qt6 Widgets 自绘，paintEvent + QPainter）
//
// 本文件不依赖 ROS / format：只 include core 层（World / Simulator / GridLayer）。
// 由外层控制器（SimApp，编入可执行文件）提供菜单/工具栏并驱动两种模式。
//
// 双模式（查看 / 编辑，默认查看）：
//   查看模式：左键拖拽平移、滚轮以光标为中心缩放；不能选测量（测量仅编辑模式）。
//   编辑模式：进入时把原障碍栅格化进共享 GridLayer 并置 active（权威占用，
//             此后显示/碰撞以格层为准）。区域按正交小格子划分（边长可调），
//             左键点/滑 = 画墙，右键点/滑 = 清除占用——原墙格也能抠出真正的洞，
//             激光与小车可穿过（存盘后重开仍在）。原几何与机器人不被修改。
//             测量（勾选后）：Ctrl+左键 点两点成一条距离标注（随 .fworld 保存，
//             不影响仿真）；Ctrl+右键 点在标注线上则删除该条。
//   进入编辑模式由控制器负责暂停仿真；此处只负责交互与绘制。
//
// 视图：世界系 +x 东、+y 北（屏幕 y 翻转）；缩放 = 像素/米，中心点缩放。
#pragma once

#include <QWidget>

#include <memory>
#include <optional>
#include <vector>

#include "core/Simulator.h"
#include "core/World.h"

namespace flat_sim {

class SimView : public QWidget {
  Q_OBJECT
 public:
  enum class Mode { View, Edit };

  explicit SimView(const WorldDesc& world, QWidget* parent = nullptr);

  QSize sizeHint() const override;

  // 每仿真步后由控制器设置并触发重绘（sim 生命周期由外部保证）
  void setSimulator(const Simulator* sim);

  // 编辑墙层（与 Simulator 共享同一实例；由控制器注入）
  void setEditGrid(std::shared_ptr<GridLayer> g);
  std::shared_ptr<GridLayer> editGrid() const { return grid_; }

  Mode mode() const { return mode_; }
  void setMode(Mode m);

  void setMeasureEnabled(bool on) {
    measureOn_ = on;
    if (!on) draftA_ = std::nullopt;
    update();
  }
  bool measureEnabled() const { return measureOn_; }

  void setAnnotationsVisible(bool v) {
    showAnnotations_ = v;
    update();
  }
  void setGridVisible(bool v) {
    showGrid_ = v;
    update();
  }
  bool gridVisible() const { return showGrid_; }
  bool laserVisible() const { return showLaser_; }
  void setLaserVisible(bool v) {
    showLaser_ = v;
    update();
  }

  void addAnnotation(Vec2 a, Vec2 b) {
    annotations_.push_back({a, b});
    setDirty(true);
    update();
  }
  const std::vector<Annotation>& annotations() const { return annotations_; }

  double cellSize() const;
  // 修改格子边长并迁移已有墙；返回实际生效边长
  double setCellSize(double meters);

  bool dirty() const { return dirty_; }
  void markClean();       // 保存/加载后：把当前内容作为干净基线
  void discardChanges();  // 放弃未保存改动：恢复到上一次 markClean 的内容

  void fitViewToWorld();  // 整窗适配（复位视图）

  // 另存为用的世界快照：原几何 + 当前编辑墙层 + 标注（.fworld 可往返）
  WorldDesc worldForSave() const;

  // 编辑墙层的墙是否全部为空（供「帮助/状态」提示判断）
  bool hasWalls() const {
    return grid_ && grid_->inited() && grid_->wallCount() > 0;
  }

 Q_SIGNALS:
  void resetRequested();  // r 键：复位机器人（外部联动发布 /slam2d/reset）
  void quitRequested();   // ESC / q：退出事件循环
  void editGridReplaced(std::shared_ptr<flat_sim::GridLayer> grid);  // 视图新建格层时通知控制器
  void dirtyChanged();    // 内容是否有未保存改动的状态翻转

 protected:
  void paintEvent(QPaintEvent* e) override;
  void keyPressEvent(QKeyEvent* e) override;
  void resizeEvent(QResizeEvent* e) override;
  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;

 private:
  // ---- 坐标 / 视图 ----
  QPointF toScreen(Vec2 w) const;
  Vec2 toWorld(QPointF sp) const;
  void zoomAt(QPointF sp, double factor);
  void panByPx(QPoint d);
  // 计算世界可视范围建议值：优先 world size，否则全部几何体包围盒
  void worldExtents(double& x0, double& x1, double& y0, double& y1) const;
  // 确保网格已配置（进入编辑模式前调用）
  void ensureGrid();
  void applyCellSize(double meters);   // 迁移旧墙后重划
  void migrateAndResize(double newCell);

  // ---- 编辑 / 测量内部 ----
  bool paintCellAt(QPoint sp, bool set);  // 返回是否真正改了墙
  void strokeTo(QPoint sp, bool set);     // 从上一格到当前格连笔画线
  void beginStroke(QPoint sp, bool set);
  void placeMeasurePoint(Vec2 w);       // 测量打点（第 2 点后生成一条标注）
  bool deleteAnnotationAt(QPointF sp);  // Ctrl+右键点在某条标注线上则删除；返回是否删到
  void setDirty(bool d);
  void restoreBaseline();
  void drawHoverOutline(QPainter& p) const;

  // ---- 绘制 ----
  void drawWorld(QPainter& p);
  void drawObstacles(QPainter& p);
  void drawEditedWalls(QPainter& p);
  void drawGridLines(QPainter& p);
  void drawAnnotations(QPainter& p);
  void drawRobotLasers(QPainter& p);
  void drawRobots(QPainter& p);
  void drawRulers(QPainter& p);
  void drawStatus(QPainter& p);

  WorldDesc world_;                     // 静态快照（原障碍等；编辑不动它）
  const Simulator* sim_ = nullptr;
  std::shared_ptr<GridLayer> grid_;     // 编辑墙层（与 Simulator 共享）
  Mode mode_ = Mode::View;
  bool measureOn_ = false;
  bool showAnnotations_ = true;
  bool showGrid_ = true;
  bool showLaser_ = true;               // 激光束显示开关（l 键）

  // 标注
  std::vector<Annotation> annotations_;  // 当前（含未保存改动）
  std::vector<Annotation> annBase_;      // 干净基线
  std::optional<Vec2> draftA_;           // 测量起点（取第二点前置空）

  // 视图
  double scale_ = 40.0;        // 像素 / 米
  double centerX_ = 0.0;       // 视图中心（世界坐标）
  double centerY_ = 0.0;
  bool viewValid_ = false;     // 尚未完成首次适配
  static constexpr double kMinScale = 2.0;
  static constexpr double kMaxScale = 8000.0;

  // 鼠标状态机
  bool midDown_ = false;
  bool panning_ = false;
  QPoint pressPos_, lastPos_;
  QPointF lastMousePx_;  // 最近一次鼠标像素位置（测量预览线的另一端）
  bool stroke_ = false;   // 编辑笔画进行中（左=画 true / 右=擦 false）
  bool strokeSet_ = true;
  int cellLastI_ = -1, cellLastJ_ = -1;
  std::optional<std::pair<int, int>> hoverCell_;
  // Ctrl+点按待判：区分"单击操作"(打测量点/删标注线) 与 "拖拽平移"
  bool pressIsCtrlLeft_ = false;   // 编辑+测量态：Ctrl+左键
  bool pressIsCtrlRight_ = false;  // Ctrl+右键（两种模式都可删线）

  // 脏基线
  bool dirty_ = false;
  bool hasGridSnap_ = false;
  GridLayer gridSnap_;
};

}  // namespace flat_sim
