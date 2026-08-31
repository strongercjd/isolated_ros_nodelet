// flat_sim —— 2D 俯视仿真视图（Qt6 实现；绘制逻辑自 SDL 版 Gui.cpp 迁移）
// 仅当 CMake 定义了 FLAT_SIM_HAVE_GUI（找到 Qt6）时参与编译。
#ifdef FLAT_SIM_HAVE_GUI

#include "gui/SimView.h"

#include <QKeyEvent>
#include <QPainter>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "core/Geometry.h"

namespace flat_sim {
namespace {

const QColor kBackground(250, 250, 250);
const QColor kBorder(70, 70, 70);       // 世界边界
const QColor kObstacle(150, 150, 150);  // 墙 / 障碍
const QColor kLaser(90, 150, 255, 80);  // 激光射线（半透明）
const QColor kHeading(40, 40, 40);      // 朝向指示线

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

}  // namespace

SimView::SimView(const WorldDesc& world, QWidget* parent)
    : QWidget(parent), world_(world) {
  // 按键（l / r / ESC）需要键盘焦点；默认 NoFocus 收不到 keyPressEvent
  setFocusPolicy(Qt::StrongFocus);
}

QSize SimView::sizeHint() const { return QSize(880, 660); }

void SimView::setSimulator(const Simulator* sim) {
  sim_ = sim;
  update();
}

QPointF SimView::toScreen(Vec2 w) const {
  return QPointF((w.x - minX_) * scale_ + kPad,
                 (double)height() - ((w.y - minY_) * scale_ + kPad));
}

// 视口适配（自 SDL 版 Gui.cpp 迁移；整窗都是仿真视图，按全窗口拟合）
void SimView::fitView() {
  double minX, minY, maxX, maxY;
  if (world_.hasSize) {
    minX = -world_.width / 2.0;
    maxX = world_.width / 2.0;
    minY = -world_.height / 2.0;
    maxY = world_.height / 2.0;
  } else {
    minX = minY = 1e18;
    maxX = maxY = -1e18;
    auto expand = [&](Vec2 p) {
      minX = std::min(minX, p.x);
      maxX = std::max(maxX, p.x);
      minY = std::min(minY, p.y);
      maxY = std::max(maxY, p.y);
    };
    for (const BoxObstacle& b : world_.boxes) {
      const double hl = b.len / 2.0, hw = b.wid / 2.0;
      for (const Vec2& c : {Vec2{-hl, -hw}, Vec2{hl, -hw}, Vec2{hl, hw}, Vec2{-hl, hw}})
        expand(toWorld(b.pose, c));
    }
    for (const CircleObstacle& c : world_.circles) {
      expand(c.center + Vec2{c.radius, c.radius});
      expand(c.center - Vec2{c.radius, c.radius});
    }
    for (const RobotDesc& r : world_.robots) {
      expand(r.pose.p + Vec2{r.radius, r.radius});
      expand(r.pose.p - Vec2{r.radius, r.radius});
    }
    if (minX > maxX) {  // 空世界兜底
      minX = -10.0; maxX = 10.0; minY = -10.0; maxY = 10.0;
    }
  }
  const double margin = 0.5;  // 米
  minX -= margin; minY -= margin; maxX += margin; maxY += margin;

  const double viewW = std::max(80.0, (double)width());
  const double viewH = std::max(60.0, (double)height());
  scale_ = std::min((viewW - 2 * kPad) / (maxX - minX),
                    (viewH - 2 * kPad) / (maxY - minY));
  if (!(scale_ > 0.0) || !std::isfinite(scale_)) scale_ = 40.0;
  const double cx = (minX + maxX) / 2.0, cy = (minY + maxY) / 2.0;
  minX_ = cx - (viewW / 2.0 - kPad) / scale_;
  minY_ = cy - (viewH / 2.0 - kPad) / scale_;
}

void SimView::resizeEvent(QResizeEvent*) { fitView(); }

void SimView::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.fillRect(rect(), kBackground);

  const WorldDesc& w = world_;

  // 世界边界
  if (w.hasSize) {
    const double hw = w.width / 2.0, hh = w.height / 2.0;
    QPolygonF border;
    border << toScreen({-hw, -hh}) << toScreen({hw, -hh}) << toScreen({hw, hh})
           << toScreen({-hw, hh});
    p.setPen(QPen(kBorder, 2));
    p.setBrush(Qt::NoBrush);
    p.drawPolygon(border);
  }

  // 障碍：矩形（含旋转）与圆
  p.setPen(Qt::NoPen);
  p.setBrush(kObstacle);
  for (const BoxObstacle& b : w.boxes) {
    const double hl = b.len / 2.0, hw = b.wid / 2.0;
    QPolygonF pts;
    for (const Vec2& c : {Vec2{-hl, -hw}, Vec2{hl, -hw}, Vec2{hl, hw}, Vec2{-hl, hw}})
      pts << toScreen(toWorld(b.pose, c));
    p.drawPolygon(pts);
  }
  for (const CircleObstacle& c : w.circles) {
    const QPointF center = toScreen(c.center);
    p.drawEllipse(center, c.radius * scale_, c.radius * scale_);
  }

  // 机器人 + 激光
  if (!sim_) return;
  for (const Simulator::RobotState& r : sim_->robots()) {
    if (showLasers_) {
      p.setPen(QPen(kLaser, 1));
      for (const Simulator::Scan& s : r.scans) {
        for (size_t k = 0; k < s.ranges.size(); ++k) {
          const double ang = s.angleStart + s.angleInc * (double)k;
          const double d = s.ranges[k];
          if (!std::isfinite(d)) continue;  // 无回波光束不画
          const Vec2 tip = s.origin.p + Vec2{d * std::cos(ang), d * std::sin(ang)};
          p.drawLine(toScreen(s.origin.p), toScreen(tip));
        }
      }
    }
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

void SimView::keyPressEvent(QKeyEvent* e) {
  switch (e->key()) {
    case Qt::Key_Escape:
    case Qt::Key_Q:
      Q_EMIT quitRequested();
      break;
    case Qt::Key_L:
      showLasers_ = !showLasers_;
      update();
      break;
    case Qt::Key_R:
      Q_EMIT resetRequested();
      break;
    default:
      QWidget::keyPressEvent(e);
  }
}

}  // namespace flat_sim

#endif  // FLAT_SIM_HAVE_GUI
