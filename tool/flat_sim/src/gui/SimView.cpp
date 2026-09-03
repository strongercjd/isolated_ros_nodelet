// flat_sim —— 2D 俯视仿真视图（Qt6 实现）
// 双模式交互（查看/编辑）+ 缩放/平移/测量 + 编辑墙格层绘制。
// 仅当 CMake 定义了 FLAT_SIM_HAVE_GUI（找到 Qt6）时参与编译。
#ifdef FLAT_SIM_HAVE_GUI

#include "gui/SimView.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>

#include "core/Geometry.h"
#include "core/GridLayerTools.h"

namespace flat_sim {
namespace {

const QColor kBackground(250, 250, 250);
const QColor kBorder(70, 70, 70);        // 世界边界
// 墙体统一灰色：原障碍(墙/箱)与编辑新增的格子墙都恢复之前的灰色；
// 靠"网格线 / 是否被栅格成格子"区分来源（编辑后保存的文件以格子呈现）。
const QColor kObstacle(155, 155, 155);   // 原障碍（未权威化时的解析几何表达）
const QColor kObstacleEdge(120, 120, 120);
const QColor kGridLine(222, 222, 222);   // 网格线
const QColor kEditWall(155, 155, 155);   // 占据格（用户画的墙 或 栅格化的原墙）
const QColor kEditWallBorder(120, 120, 120);
const QColor kAnno(50, 105, 200);        // 测量标注（蓝）
const QColor kLaser(110, 170, 255, 90);
const QColor kHeading(40, 40, 40);
const QColor kRulerLine(150, 150, 150);
const QColor kRulerText(90, 90, 90);
const QColor kStatusBg(255, 255, 255, 215);
const QColor kHelpText(60, 60, 60);

QColor colorOf(const std::string& name) {
  static const std::map<std::string, QColor> kColors = {
      {"red", QColor(215, 65, 60)},     {"green", QColor(70, 165, 80)},
      {"blue", QColor(70, 110, 220)},   {"yellow", QColor(225, 195, 60)},
      {"orange", QColor(235, 150, 60)}, {"purple", QColor(160, 95, 200)},
      {"black", QColor(50, 50, 50)},    {"gray", QColor(130, 130, 130)},
      {"grey", QColor(130, 130, 130)},
  };
  const auto it = kColors.find(name);
  return it != kColors.end() ? it->second : QColor(215, 65, 60);
}

double clampf(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

std::string fmtLen(double m) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3g", m);
  return buf;
}
std::string fmtCrd(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3g", v);
  return buf;
}

// 点到线段的距离（屏幕像素，删除测量标注的命中判定）
double pointSegDist(const QPointF& p, const QPointF& a, const QPointF& b) {
  const QPointF ab = b - a;
  const double len2 = ab.x() * ab.x() + ab.y() * ab.y();
  double t = len2 > 0.0 ? (((p.x() - a.x()) * ab.x() + (p.y() - a.y()) * ab.y()) / len2) : 0.0;
  t = clampf(t, 0.0, 1.0);
  const QPointF q = a + ab * t;
  return std::hypot(p.x() - q.x(), p.y() - q.y());
}

// 选一个"漂亮"的刻度间距：1/2/5×10^k，使相邻刻度像素间距 >= pxMin
double niceStep(double pxMin, double scale) {
  double raw = pxMin / scale;
  const double mag = std::pow(10.0, std::floor(std::log10(raw)));
  const double norm = raw / mag;
  const double s = (norm <= 1.0 ? 1.0 : norm <= 2.0 ? 2.0 : norm <= 5.0 ? 5.0 : 10.0) * mag;
  return s > 0.0 ? s : 1.0;
}

// 两格（含端点）之间的整格直线：Bresenham，把经过的格逐一交给 visit
void cellLine(int i0, int j0, int i1, int j1,
              const std::function<void(int, int)>& visit) {
  int di = i1 > i0 ? 1 : -1;
  int dj = j1 > j0 ? 1 : -1;
  int dx = std::abs(i1 - i0), dy = std::abs(j1 - j0);
  int err = dx - dy;
  int i = i0, j = j0;
  while (true) {
    visit(i, j);
    if (i == i1 && j == j1) break;
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; i += di; }
    if (e2 < dx) { err += dx; j += dj; }
  }
}

}  // namespace

SimView::SimView(const WorldDesc& world, QWidget* parent)
    : QWidget(parent), world_(world) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);  // 无按键也收到 move（悬停高亮 / 测量预览）
  // 加载进来的 .fworld 已带标注：直接作为初始内容与干净基线
  annotations_ = world_.annotations;
  annBase_ = world_.annotations;
}

QSize SimView::sizeHint() const { return QSize(880, 660); }

void SimView::setSimulator(const Simulator* sim) {
  sim_ = sim;
  update();
}

void SimView::setEditGrid(std::shared_ptr<GridLayer> g) {
  grid_ = std::move(g);
  // 首次注入即以当前格层为干净基线（后续 discardChanges 恢复到这里）
  if (grid_ && grid_->inited()) {
    gridSnap_.copyFrom(*grid_);
    hasGridSnap_ = true;
  } else {
    hasGridSnap_ = false;
  }
  if (mode_ == Mode::Edit) ensureGrid();
  update();
}

// ---- 坐标变换 ----
QPointF SimView::toScreen(Vec2 w) const {
  const double W = std::max(1.0, (double)width());
  const double H = std::max(1.0, (double)height());
  return QPointF((w.x - centerX_) * scale_ + W / 2.0, H / 2.0 - (w.y - centerY_) * scale_);
}

Vec2 SimView::toWorld(QPointF sp) const {
  const double W = std::max(1.0, (double)width());
  const double H = std::max(1.0, (double)height());
  return Vec2{centerX_ + (sp.x() - W / 2.0) / scale_, centerY_ - (sp.y() - H / 2.0) / scale_};
}

void SimView::zoomAt(QPointF sp, double factor) {
  const double old = scale_;
  const double ns = clampf(old * factor, kMinScale, kMaxScale);
  if (std::fabs(ns - old) < 1e-12) return;
  const Vec2 w = toWorld(sp);  // 缩放前光标下的世界点
  scale_ = ns;
  const double W = std::max(1.0, (double)width());
  const double H = std::max(1.0, (double)height());
  centerX_ = w.x - (sp.x() - W / 2.0) / ns;
  centerY_ = w.y + (sp.y() - H / 2.0) / ns;  // 保持该点仍在光标下
  update();
}

void SimView::panByPx(QPoint d) {
  if (scale_ <= 0.0 || (d.x() == 0 && d.y() == 0)) return;
  centerX_ -= (double)d.x() / scale_;
  centerY_ += (double)d.y() / scale_;
  update();
}

void SimView::worldExtents(double& x0, double& x1, double& y0, double& y1) const {
  if (world_.hasSize) {
    x0 = -world_.width / 2.0; x1 = world_.width / 2.0;
    y0 = -world_.height / 2.0; y1 = world_.height / 2.0;
    return;
  }
  x0 = y0 = 1e18; x1 = y1 = -1e18;
  auto expand = [&](Vec2 p) {
    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
    y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
  };
  for (const BoxObstacle& b : world_.boxes) {
    const double hl = b.len / 2.0, hw = b.wid / 2.0;
    expand(flat_sim::toWorld(b.pose, Vec2{-hl, -hw}));
    expand(flat_sim::toWorld(b.pose, Vec2{hl, -hw}));
    expand(flat_sim::toWorld(b.pose, Vec2{hl, hw}));
    expand(flat_sim::toWorld(b.pose, Vec2{-hl, hw}));
  }
  for (const CircleObstacle& c : world_.circles) {
    expand(c.center + Vec2{c.radius, c.radius});
    expand(c.center - Vec2{c.radius, c.radius});
  }
  for (const RobotDesc& r : world_.robots) {
    expand(r.pose.p + Vec2{r.radius, r.radius});
    expand(r.pose.p - Vec2{r.radius, r.radius});
  }
  // 编辑墙也纳入适配范围（改完墙后即便缩小也能看到全貌）
  if (grid_ && grid_->inited()) {
    expand(Vec2{grid_->minX(), grid_->minY()});
    expand(Vec2{grid_->maxX(), grid_->maxY()});
  }
  if (x0 > x1) { x0 = -10; x1 = 10; y0 = -10; y1 = 10; }
}

void SimView::fitViewToWorld() {
  double x0, x1, y0, y1;
  worldExtents(x0, x1, y0, y1);
  const double W = std::max(40.0, (double)width());
  const double H = std::max(30.0, (double)height());
  const double padX = 70.0, padY = 44.0;  // 留白：刻度条 + 状态
  double s = std::min((W - 2 * padX) / (x1 - x0), (H - 2 * padY) / (y1 - y0));
  if (!(s > 0.0) || !std::isfinite(s)) s = 40.0;
  scale_ = clampf(s, kMinScale, kMaxScale);
  centerX_ = (x0 + x1) / 2.0;
  centerY_ = (y0 + y1) / 2.0;
  viewValid_ = true;
  update();
}

// ---- 模式与格层 ----
void SimView::setMode(Mode m) {
  if (mode_ == m) return;
  mode_ = m;
  if (m == Mode::Edit) {
    ensureGrid();
    setCursor(Qt::CrossCursor);
  } else {
    setCursor(Qt::ArrowCursor);
  }
  draftA_ = std::nullopt;
  stroke_ = false;
  panning_ = false;
  update();
}

void SimView::ensureGrid() {
  if (!grid_) {
    grid_ = std::make_shared<GridLayer>();
    Q_EMIT editGridReplaced(grid_);
  }
  const double keepCell = grid_->inited() ? grid_->cell : 0.1;

  // 权威化所需的覆盖矩形：至少盖住 世界可视范围 与 全部原障碍（即使超出 world size），
  // 保证切换成"格层权威占用"后不会有原障碍落在网格外变成"幽灵"。
  double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18;
  auto expand = [&](Vec2 p) {
    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
    y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
  };
  if (world_.hasSize) {
    expand(Vec2{-world_.width / 2.0, -world_.height / 2.0});
    expand(Vec2{world_.width / 2.0, world_.height / 2.0});
  }
  for (const BoxObstacle& b : world_.boxes) {
    const double hl = b.len / 2.0, hw = b.wid / 2.0;
    expand(flat_sim::toWorld(b.pose, Vec2{-hl, -hw}));
    expand(flat_sim::toWorld(b.pose, Vec2{hl, -hw}));
    expand(flat_sim::toWorld(b.pose, Vec2{hl, hw}));
    expand(flat_sim::toWorld(b.pose, Vec2{-hl, hw}));
  }
  for (const CircleObstacle& c : world_.circles) {
    expand(c.center + Vec2{c.radius, c.radius});
    expand(c.center - Vec2{c.radius, c.radius});
  }
  if (x0 > x1) { x0 = -10; x1 = 10; y0 = -10; y1 = 10; }

  // 覆盖不足(未初始化 或 盖不住上面的矩形) → 并入旧网格范围后重划，迁移旧占用
  const bool covered =
      grid_->inited() && grid_->minX() <= x0 + 1e-9 && grid_->minY() <= y0 + 1e-9 &&
      grid_->maxX() >= x1 - 1e-9 && grid_->maxY() >= y1 - 1e-9;
  if (!covered) {
    if (grid_->inited()) {  // 保留旧墙：并入旧范围（不缩小已画区域）
      x0 = std::min(x0, grid_->minX()); x1 = std::max(x1, grid_->maxX());
      y0 = std::min(y0, grid_->minY()); y1 = std::max(y1, grid_->maxY());
    }
    GridLayer old;
    const bool haveOld = grid_->inited();
    if (haveOld) old.copyFrom(*grid_);
    grid_->configure(x0, x1, y0, y1, keepCell);
    if (haveOld) {  // 迁移：旧墙格心落在新网格的哪些格子上，就置为墙
      for (int j = 0; j < old.rows; ++j)
        for (int i = 0; i < old.cols; ++i)
          if (old.at(i, j)) {
            const double cx = old.ox + (i + 0.5) * old.cell;
            const double cy = old.oy + (j + 0.5) * old.cell;
            grid_->fillWorldRect(cx, cy, cx, cy);
          }
    }
  }

  // 首次权威化：把原障碍栅格化进占据层并置 active。此后显示/碰撞都以本层为准，
  // 右键清掉的格（含原墙上的）就真的打通。权威化本身不算"用户改动"，
  // 因此把含原障碍的完整占用更新为干净基线（discard 时不会连原障碍一起删掉）。
  if (!grid_->active) {
    bakeWorldObstacles(world_, *grid_);
    grid_->active = true;
    gridSnap_.copyFrom(*grid_);
    hasGridSnap_ = true;
  }
  update();
}

double SimView::cellSize() const {
  return grid_ && grid_->inited() ? grid_->cell : 0.1;
}

void SimView::migrateAndResize(double newCell) {
  if (!grid_ || !grid_->inited()) return;
  GridLayer old;
  old.copyFrom(*grid_);
  const double x0 = old.minX(), x1 = old.maxX(), y0 = old.minY(), y1 = old.maxY();
  grid_->configure(x0, x1, y0, y1, newCell);
  // 迁移：旧墙格心落在新网格的哪些格子上，就置为墙
  for (int j = 0; j < old.rows; ++j)
    for (int i = 0; i < old.cols; ++i)
      if (old.at(i, j)) {
        const double cx = old.ox + (i + 0.5) * old.cell;
        const double cy = old.oy + (j + 0.5) * old.cell;
        grid_->fillWorldRect(cx, cy, cx, cy);
      }
  setDirty(true);
  update();
}

double SimView::setCellSize(double meters) {
  if (!(meters > 0.0)) return cellSize();
  if (!grid_ || !grid_->inited()) ensureGrid();
  if (!grid_->inited()) return cellSize();
  if (std::fabs(grid_->cell - meters) > 1e-12) migrateAndResize(meters);
  return grid_->cell;
}

void SimView::markClean() {
  annBase_ = annotations_;
  if (grid_ && grid_->inited()) {
    gridSnap_.copyFrom(*grid_);
    hasGridSnap_ = true;
  } else {
    hasGridSnap_ = false;
  }
  dirty_ = false;
}

void SimView::restoreBaseline() {
  annotations_ = annBase_;
  if (grid_) {
    if (hasGridSnap_) grid_->copyFrom(gridSnap_);
    else grid_->clearWalls();
  }
  dirty_ = false;
  update();
}

void SimView::discardChanges() { restoreBaseline(); }

void SimView::setDirty(bool d) {
  const bool was = dirty_;
  dirty_ = d;
  if (was != d) Q_EMIT dirtyChanged();
}

// 另存为快照：原世界几何 + 当前编辑墙/标注（不带旧 editGrid/annotations 残留值）
WorldDesc SimView::worldForSave() const {
  WorldDesc w = world_;
  w.editGrid = grid_ && grid_->inited() ? *grid_ : GridLayer{};
  w.annotations = annotations_;
  w.sourceFile.clear();  // 保存目标文件名由调用方在写文件时回填
  return w;
}

// ---- 编辑：格子笔画 ----
bool SimView::paintCellAt(QPoint sp, bool set) {
  if (!grid_ || !grid_->inited()) return false;
  int i, j;
  if (!grid_->cellOf(toWorld(QPointF(sp)), i, j)) return false;
  if (grid_->at(i, j) == (set ? 1u : 0u)) return false;  // 已是目标状态，无变化
  grid_->set(i, j, set);
  setDirty(true);
  return true;
}

void SimView::beginStroke(QPoint sp, bool set) {
  stroke_ = true;
  strokeSet_ = set;
  cellLastI_ = cellLastJ_ = -1;
  int i, j;
  if (!grid_ || !grid_->cellOf(toWorld(QPointF(sp)), i, j)) return;
  paintCellAt(sp, set);
  cellLastI_ = i;
  cellLastJ_ = j;
  update();
}

void SimView::strokeTo(QPoint sp, bool set) {
  if (!stroke_ || !grid_ || !grid_->inited()) return;
  int i, j;
  if (!grid_->cellOf(toWorld(QPointF(sp)), i, j)) return;
  if (cellLastI_ < 0) {  // 起点未记录（原障碍上按下）：从当前格续起
    cellLastI_ = i;
    cellLastJ_ = j;
    return;
  }
  cellLine(cellLastI_, cellLastJ_, i, j, [&](int ci, int cj) {
    grid_->set(ci, cj, set ? 1 : 0);
  });
  setDirty(true);
  cellLastI_ = i;
  cellLastJ_ = j;
  update();
}

// 测量打点：第一点先落到 draftA_（画预览点/虚线），第二点生成一条标注
void SimView::placeMeasurePoint(Vec2 w) {
  if (!draftA_) {
    draftA_ = w;
  } else {
    annotations_.push_back(Annotation{draftA_.value(), w});
    setDirty(true);
    draftA_ = std::nullopt;
  }
  update();
}

// Ctrl+右键命中：点到"屏幕标注线段"的距离 <= 阈值即删该条；返回是否删到
bool SimView::deleteAnnotationAt(QPointF sp) {
  const double kHitPx = 7.0;
  for (size_t k = 0; k < annotations_.size(); ++k) {
    const QPointF s1 = toScreen(annotations_[k].from);
    const QPointF s2 = toScreen(annotations_[k].to);
    if (pointSegDist(sp, s1, s2) <= kHitPx) {
      annotations_.erase(annotations_.begin() + (long)k);
      setDirty(true);
      update();
      return true;
    }
  }
  return false;
}

// ---- 事件 ----
void SimView::resizeEvent(QResizeEvent*) {
  if (!viewValid_) fitViewToWorld();  // 仅首次适配；之后保留用户缩放/平移
}

void SimView::wheelEvent(QWheelEvent* e) {
  const double factor = e->angleDelta().y() > 0 ? 1.25 : 0.8;
  zoomAt(e->position(), factor);
  e->accept();
}

void SimView::mousePressEvent(QMouseEvent* e) {
  setFocus();
  const QPoint pos = e->position().toPoint();
  const bool ctrl = (e->modifiers() & Qt::ControlModifier) != 0;

  if (e->button() == Qt::MiddleButton) {
    panning_ = true;
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  if (e->button() == Qt::LeftButton) {
    pressPos_ = pos;
    lastPos_ = pos;  // 首个 move 的增量以按下点为基准，避免跳变
    panning_ = false;
    pressIsCtrlLeft_ = false;
    if (mode_ == Mode::Edit) {
      if (ctrl) {
        if (measureOn_) {
          pressIsCtrlLeft_ = true;  // 待判：Ctrl+左 单击=测点，拖动超阈值=平移
        } else {
          panning_ = true;  // 未开测量：Ctrl+左拖 平移
          setCursor(Qt::ClosedHandCursor);
        }
      } else {
        beginStroke(pos, /*set=*/true);  // 左键画墙（测量勾选时仍是画墙）
      }
    } else {
      panning_ = true;  // 查看模式：左键直接进入拖拽平移
      setCursor(Qt::ClosedHandCursor);
    }
    return;
  }
  if (e->button() == Qt::RightButton) {
    pressPos_ = pos;  // Ctrl+右键单击判定与左键共用 pressPos_
    lastPos_ = pos;
    if (ctrl) {
      pressIsCtrlRight_ = true;  // 待判：Ctrl+右 单击在某标注线上=删该条，拖动=平移
      panning_ = false;
    } else if (mode_ == Mode::Edit) {
      beginStroke(pos, /*set=*/false);  // 右键擦墙（打通）
    }
  }
}

void SimView::mouseMoveEvent(QMouseEvent* e) {
  const QPoint pos = e->position().toPoint();
  const QPoint delta = pos - lastPos_;
  lastPos_ = pos;
  lastMousePx_ = e->position();  // 测量预览线 / 悬停跟随

  if (panning_) { panByPx(delta); return; }

  // 待判的 Ctrl+点按：移动超过阈值才转成平移（区分"单击操作"与"拖拽平移"）
  if (pressIsCtrlLeft_ || pressIsCtrlRight_) {
    if ((pos - pressPos_).manhattanLength() >= 4) {
      pressIsCtrlLeft_ = false;
      pressIsCtrlRight_ = false;
      panning_ = true;
      setCursor(Qt::ClosedHandCursor);
    }
    return;
  }

  if (mode_ == Mode::Edit) {
    const bool left = e->buttons() & Qt::LeftButton;
    const bool right = e->buttons() & Qt::RightButton;
    if (stroke_ && (left || right)) {
      strokeTo(pos, strokeSet_);
      return;
    }
    // 悬停高亮（编辑提示当前将画/擦的格子）
    int i, j;
    if (grid_ && grid_->cellOf(toWorld(QPointF(pos)), i, j))
      hoverCell_ = std::make_pair(i, j);
    else
      hoverCell_ = std::nullopt;
    update();
    return;
  }

  // 查看模式：仅平移（已在上方处理）；有测量首点未完成时刷新预览
  if (draftA_) update();
}

void SimView::mouseReleaseEvent(QMouseEvent* e) {
  const QPoint pos = e->position().toPoint();
  const bool wasClick = (pos - pressPos_).manhattanLength() < 4;

  if (e->button() == Qt::MiddleButton) {
    panning_ = false;
    setCursor(mode_ == Mode::Edit ? Qt::CrossCursor : Qt::ArrowCursor);
    return;
  }
  if (e->button() == Qt::RightButton) {
    if (pressIsCtrlRight_) {  // Ctrl+右键：单击在标注线上 → 删该条
      pressIsCtrlRight_ = false;
      const bool wasPan = panning_;
      panning_ = false;
      if (!wasPan && wasClick) deleteAnnotationAt(QPointF(pos));
      setCursor(mode_ == Mode::Edit ? Qt::CrossCursor : Qt::ArrowCursor);
      return;
    }
    if (mode_ == Mode::Edit && stroke_ && !panning_) stroke_ = false;
    panning_ = false;
    setCursor(mode_ == Mode::Edit ? Qt::CrossCursor : Qt::ArrowCursor);
    return;
  }
  if (e->button() == Qt::LeftButton) {
    const bool wasPan = panning_;
    panning_ = false;
    if (mode_ == Mode::Edit) {
      if (pressIsCtrlLeft_) {  // Ctrl+左单击 + 测量勾选 → 打测量点
        pressIsCtrlLeft_ = false;
        setCursor(Qt::CrossCursor);
        if (!wasPan && wasClick && measureOn_) placeMeasurePoint(toWorld(QPointF(pos)));
        return;
      }
      if (stroke_) stroke_ = false;
      setCursor(Qt::CrossCursor);
      return;
    }
    setCursor(Qt::ArrowCursor);
  }
}

void SimView::keyPressEvent(QKeyEvent* e) {
  switch (e->key()) {
    case Qt::Key_Escape:
    case Qt::Key_Q:
      Q_EMIT quitRequested();
      break;
    case Qt::Key_F:
      fitViewToWorld();
      break;
    case Qt::Key_L:
      showLaser_ = !showLaser_;
      update();
      break;
    default:
      QWidget::keyPressEvent(e);
  }
}

// ---- 绘制 ----
void SimView::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.fillRect(rect(), kBackground);
  p.setRenderHint(QPainter::Antialiasing, true);

  drawWorld(p);       // 世界边界
  drawObstacles(p);   // 原几何障碍
  drawGridLines(p);   // 网格线（编辑模式恒显；查看模式随开关）
  drawEditedWalls(p); // 用户编辑的墙（压过网格线）
  drawHoverOutline(p);

  drawRobotLasers(p);
  drawRobots(p);

  drawAnnotations(p); // 测量标注在最上层
  drawRulers(p);
  drawStatus(p);
}

void SimView::drawWorld(QPainter& p) {
  double x0, x1, y0, y1;
  worldExtents(x0, x1, y0, y1);
  QPolygonF border;
  border << toScreen(Vec2{x0, y0}) << toScreen(Vec2{x1, y0})
         << toScreen(Vec2{x1, y1}) << toScreen(Vec2{x0, y1});
  p.setPen(QPen(kBorder, 2));
  p.setBrush(Qt::NoBrush);
  p.drawPolygon(border);
}

void SimView::drawObstacles(QPainter& p) {
  // 权威占用：原几何已被栅格化进格层，由 drawEditedWalls 以灰色格子呈现；
  // 这里若再画平滑几何会盖住抠出来的洞。
  if (gridAuthoritative(grid_.get())) return;
  p.setPen(QPen(kObstacleEdge, 1));
  p.setBrush(kObstacle);
  for (const BoxObstacle& b : world_.boxes) {
    const double hl = b.len / 2.0, hw = b.wid / 2.0;
    QPolygonF pts;
    pts << toScreen(flat_sim::toWorld(b.pose, Vec2{-hl, -hw}))
        << toScreen(flat_sim::toWorld(b.pose, Vec2{hl, -hw}))
        << toScreen(flat_sim::toWorld(b.pose, Vec2{hl, hw}))
        << toScreen(flat_sim::toWorld(b.pose, Vec2{-hl, hw}));
    p.drawPolygon(pts);
  }
  for (const CircleObstacle& c : world_.circles) {
    const QPointF center = toScreen(c.center);
    p.drawEllipse(center, c.radius * scale_, c.radius * scale_);
  }
}

// 按行合并 run 绘制用户编辑的墙（墙格多时仍高效）
void SimView::drawEditedWalls(QPainter& p) {
  if (!grid_ || !grid_->inited() || grid_->wallCount() == 0) return;
  const double cs = grid_->cell;
  p.setPen(QPen(kEditWallBorder, 1));
  p.setBrush(kEditWall);
  for (int j = 0; j < grid_->rows; ++j) {
    int i = 0;
    while (i < grid_->cols) {
      if (!grid_->at(i, j)) { ++i; continue; }
      const int i0 = i;
      while (i < grid_->cols && grid_->at(i, j)) ++i;
      const int i1 = i - 1;
      const QPointF a = toScreen(Vec2{grid_->ox + i0 * cs, grid_->oy + j * cs});
      const QPointF b = toScreen(Vec2{grid_->ox + (i1 + 1) * cs, grid_->oy + (j + 1) * cs});
      p.drawRect(QRectF(QPointF(a.x(), b.y()), QPointF(b.x(), a.y())));
    }
  }
}

void SimView::drawGridLines(QPainter& p) {
  if (!grid_ || !grid_->inited()) return;
  if (mode_ != Mode::Edit && !showGrid_) return;  // 查看模式仅当开启"显示格子"
  const double cs = grid_->cell;
  // 只画当前视口涉及的网格线
  const double W = std::max(1.0, (double)width());
  const double H = std::max(1.0, (double)height());
  const double vx0 = centerX_ - (W / 2.0) / scale_ - cs;
  const double vx1 = centerX_ + (W / 2.0) / scale_ + cs;
  const double vy0 = centerY_ - (H / 2.0) / scale_ - cs;
  const double vy1 = centerY_ + (H / 2.0) / scale_ + cs;
  const int iMin = std::max(0, (int)std::floor((std::max(vx0, grid_->minX()) - grid_->ox) / cs));
  const int iMax = std::min(grid_->cols - 1, (int)std::ceil((std::min(vx1, grid_->maxX()) - grid_->ox) / cs) - 1);
  const int jMin = std::max(0, (int)std::floor((std::max(vy0, grid_->minY()) - grid_->oy) / cs));
  const int jMax = std::min(grid_->rows - 1, (int)std::ceil((std::min(vy1, grid_->maxY()) - grid_->oy) / cs) - 1);
  p.setPen(QPen(kGridLine, 1));
  // 整条网格线绘制（列线 x=ox+i*cs，行线 y=oy+j*cs）
  for (int i = iMin; i <= iMax + 1; ++i) {
    const double x = grid_->ox + i * cs;
    p.drawLine(toScreen(Vec2{x, grid_->minY()}), toScreen(Vec2{x, grid_->maxY()}));
  }
  for (int j = jMin; j <= jMax + 1; ++j) {
    const double y = grid_->oy + j * cs;
    p.drawLine(toScreen(Vec2{grid_->minX(), y}), toScreen(Vec2{grid_->maxX(), y}));
  }
}

void SimView::drawHoverOutline(QPainter& p) const {
  if (mode_ != Mode::Edit || !hoverCell_ || !grid_ || !grid_->inited()) return;
  const int i = hoverCell_->first, j = hoverCell_->second;
  const double cs = grid_->cell;
  const QPointF a = toScreen(Vec2{grid_->ox + i * cs, grid_->oy + j * cs});
  const QPointF b = toScreen(Vec2{grid_->ox + (i + 1) * cs, grid_->oy + (j + 1) * cs});
  p.setPen(QPen(QColor(60, 150, 90), 2));
  p.setBrush(Qt::NoBrush);
  p.drawRect(QRectF(QPointF(a.x(), b.y()), QPointF(b.x(), a.y())));
}

void SimView::drawAnnotations(QPainter& p) {
  if (!showAnnotations_) return;
  p.setPen(QPen(kAnno, 2));
  QFont f = p.font();
  f.setPointSize(9);
  p.setFont(f);
  p.setBrush(kAnno);
  for (const Annotation& a : annotations_) {
    const QPointF s1 = toScreen(a.from), s2 = toScreen(a.to);
    p.drawLine(s1, s2);
    const double r = 3.0;
    p.drawEllipse(s1, r, r);
    p.drawEllipse(s2, r, r);
    const double len = std::hypot(a.to.x - a.from.x, a.to.y - a.from.y);
    const QPointF mid((s1.x() + s2.x()) / 2.0, (s1.y() + s2.y()) / 2.0 - 14.0);
    // 测量值用白色底衬避免压在墙/障碍上看不清
    const QString txt = QString::fromUtf8((fmtLen(len) + " m").c_str());
    QRectF box(mid.x() - 26, mid.y() - 9, 52, 14);
    p.fillRect(box, QColor(255, 255, 255, 200));
    p.drawText(mid, txt);
  }
  // 测量：第一点标记 + 到光标当前位置的预览虚线（帮助瞄准第二点）
  if (draftA_) {
    const QPointF s1 = toScreen(draftA_.value());
    p.drawEllipse(s1, 4, 4);
    if (!lastMousePx_.isNull()) {
      QPen dash(kAnno);
      dash.setStyle(Qt::DashLine);
      dash.setWidthF(1.0);
      p.setPen(dash);
      p.drawLine(s1, lastMousePx_);
      p.setPen(QPen(kAnno, 2));  // 还原，避免影响后续绘制
    }
  }
}

void SimView::drawRobotLasers(QPainter& p) {
  if (!sim_ || !showLaser_) return;
  p.setPen(QPen(kLaser, 1));
  for (const Simulator::RobotState& r : sim_->robots()) {
    for (const Simulator::Scan& s : r.scans) {
      for (size_t k = 0; k < s.ranges.size(); ++k) {
        const double d = s.ranges[k];
        if (!std::isfinite(d)) continue;
        const double ang = s.angleStart + s.angleInc * (double)k;
        const Vec2 tip = s.origin.p + Vec2{d * std::cos(ang), d * std::sin(ang)};
        p.drawLine(toScreen(s.origin.p), toScreen(tip));
      }
    }
  }
}

void SimView::drawRobots(QPainter& p) {
  if (!sim_) return;
  for (const Simulator::RobotState& r : sim_->robots()) {
    p.setPen(Qt::NoPen);
    p.setBrush(colorOf(r.color));
    const QPointF c = toScreen(r.pose.p);
    p.drawEllipse(c, r.radius * scale_, r.radius * scale_);
    const Vec2 tip = r.pose.p + Vec2{1.9 * r.radius * std::cos(r.pose.yaw),
                                     1.9 * r.radius * std::sin(r.pose.yaw)};
    p.setPen(QPen(kHeading, 2));
    p.drawLine(c, toScreen(tip));
  }
}

// 刻度条：顶部横条给 x 读数，左部竖条给 y 读数（随平移缩放移动，CAD 式）
void SimView::drawRulers(QPainter& p) {
  const double W = std::max(1.0, (double)width());
  const double H = std::max(1.0, (double)height());
  const int topH = 20, leftW = 44;

  QFont f = p.font();
  f.setPointSize(8);
  p.setFont(f);

  // ---- 顶部横条（世界 x 读数） ----
  const double vx0 = centerX_ - (W / 2.0) / scale_;
  const double vx1 = centerX_ + (W / 2.0) / scale_;
  const double stepX = niceStep(110, scale_);
  const double xStart = std::ceil(vx0 / stepX) * stepX;
  for (double x = xStart; x <= vx1 + 1e-9; x += stepX) {
    const double sx = toScreen(Vec2{x, 0}).x();
    if (sx < leftW + 30 || sx > W - 12) continue;  // 给文字留边
    p.fillRect(QRectF(sx - 1, 0, 2, 7), kRulerLine);
    p.setPen(kRulerText);
    p.drawText(QPointF(sx - 22, 16), QString::fromUtf8(fmtCrd(x).c_str()));
  }
  p.fillRect(QRectF(0, topH - 1, W, 1), QColor(0, 0, 0, 26));

  // ---- 左部竖条（世界 y 读数） ----
  const double vy0 = centerY_ - (H / 2.0) / scale_;
  const double vy1 = centerY_ + (H / 2.0) / scale_;
  const double stepY = niceStep(70, scale_);
  const double yStart = std::ceil(vy0 / stepY) * stepY;
  for (double y = yStart; y <= vy1 + 1e-9; y += stepY) {
    const double sy = toScreen(Vec2{0, y}).y();
    if (sy < topH + 14 || sy > H - 8) continue;
    p.fillRect(QRectF(0, sy - 1, 7, 2), kRulerLine);
    p.setPen(kRulerText);
    p.drawText(QPointF(2, sy + 3), QString::fromUtf8(fmtCrd(y).c_str()));
  }
  p.fillRect(QRectF(leftW - 1, 0, 1, H), QColor(0, 0, 0, 26));

  // ---- 右下角固定比例尺 ----
  const double barMeters = niceStep(110, scale_);
  const double px = barMeters * scale_;
  const double bx = W - 48 - px, by = H - 18;
  p.setPen(QPen(QColor(90, 90, 90), 2));
  p.drawLine(QPointF(bx, by), QPointF(bx + px, by));
  p.drawLine(QPointF(bx, by - 4), QPointF(bx, by + 4));
  p.drawLine(QPointF(bx + px, by - 4), QPointF(bx + px, by + 4));
  p.setPen(kRulerText);
  p.drawText(QPointF(bx + px / 2 - 26, by - 8),
             QString::fromUtf8((fmtCrd(barMeters) + " m").c_str()));
}

void SimView::drawStatus(QPainter& p) {
  double x0, x1, y0, y1;
  worldExtents(x0, x1, y0, y1);
  const double W = std::max(1.0, (double)width());
  const double H = std::max(1.0, (double)height());
  const double vw = W / scale_, vh = H / scale_;

  std::string s;
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "世界 %.3g x %.3g m   视口 %.3g x %.3g m   缩放 %.0f px/m",
                  x1 - x0, y1 - y0, vw, vh, scale_);
    s = buf;
  }
  if (dirty_) s += "   [有未保存改动]";
  if (mode_ == Mode::Edit) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "编辑模式 格子 %.3g m — 左键画墙 / 右键打通(含原墙) / Ctrl+拖 平移",
                  cellSize());
    s += "\n";
    s += buf;
    if (measureOn_)
      s += "\n测量：Ctrl+左键 点两点测距；Ctrl+右键 点在线上删除该条";
  }
  QFont f = p.font();
  f.setPointSize(9);
  p.setFont(f);
  int nLines = 1;
  for (char c : s)
    if (c == '\n') ++nLines;
  const int lineH = 15;
  const int boxH = 12 + nLines * lineH;
  const double W2 = std::min(780.0, W - 20.0);
  const QRectF box(10, H - boxH - 8, W2, boxH);
  p.fillRect(box, kStatusBg);
  p.setPen(kHelpText);
  p.drawText(box.adjusted(8, 3, -6, -3), Qt::TextWordWrap, QString::fromUtf8(s.c_str()));
}

}  // namespace flat_sim

#endif  // FLAT_SIM_HAVE_GUI
