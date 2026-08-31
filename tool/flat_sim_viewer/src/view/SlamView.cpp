// flat_sim_viewer —— SLAM 建图视图（Qt6 实现；视图数学自 SDL 版 SlamView 迁移）
#include "view/SlamView.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace flat_sim_viewer {
namespace {

// 与 lidarslam_2d 的 cv_ui 画布保持一致的颜色 / 尺度
const QColor kGridLine(96, 96, 96);     // 5m 网格线
const QColor kInputCloud(255, 0, 0);    // 配准前点云（红）
const QColor kMappingCloud(0, 0, 255);  // 配准后点云（蓝）
const QColor kPoseArrow(0, 0, 255);     // 位姿箭头（蓝）
const double kDefaultMPerPx = 0.01;     // cv_ui scalar：100 px/m
const double kGridStep = 5.0;           // 网格间距（米）

}  // namespace

SlamView::SlamView(QWidget* parent) : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);  // v / ESC 按键需要键盘焦点
  setMouseTracking(false);
}

void SlamView::setSnapshot(const SlamSnapshot& snap) {
  snap_ = snap;
  update();
}

QString SlamView::statusText() const {
  return QString::asprintf("%.3f m/px · %s", mPerPx_, follow_ ? "跟随中" : "自由视图");
}

// 对应 SDL 版 updateTexture：occ 0-100（50=未观测）→ 灰度（自由白 / 占用黑）；
// 消息 row0 = y 最小（标准语义），图像 row0 = 显示顶部（y 最大）→ 行翻转。
void SlamView::rebuildMapImage(const SlamSnapshot::Map& m) {
  if (!m.data || m.width <= 0 || m.height <= 0) return;
  QImage img(m.width, m.height, QImage::Format_Grayscale8);
  const int8_t* src = m.data->data();
  for (int row = 0; row < m.height; ++row) {
    const int8_t* srow = src + (size_t)row * m.width;
    uint8_t* drow = img.scanLine(m.height - 1 - row);
    for (int col = 0; col < m.width; ++col)
      drow[col] = (uint8_t)(255 - srow[col] * 255 / 100);
  }
  mapImg_ = std::move(img);
  mapSeq_ = m.seq;
}

void SlamView::paintEvent(QPaintEvent*) {
  QPainter p(this);
  if (follow_ && snap_.hasPose) {  // 视图中心锁定最新位姿
    cx_ = snap_.poseX;
    cy_ = snap_.poseY;
  }

  // 黑底（地图未到时也只有网格，一眼可辨）
  p.fillRect(rect(), Qt::black);

  // ---- 地图（QImage 仅代数变化时重建）----
  if (snap_.map.data && snap_.map.seq != mapSeq_) rebuildMapImage(snap_.map);
  if (!mapImg_.isNull()) {
    const double x0 = snap_.map.originX;
    const double y0 = snap_.map.originY;
    const double x1 = x0 + snap_.map.width * snap_.map.resolution;
    const double y1 = y0 + snap_.map.height * snap_.map.resolution;
    const QRectF dst(QPointF(sx(x0), sy(y1)), QPointF(sx(x1), sy(y0)));
    p.setRenderHint(QPainter::SmoothPixmapTransform, false);  // 保持像素观感
    p.drawImage(dst, mapImg_);
  }

  // ---- 5m 网格线 ----
  p.setPen(QPen(kGridLine, 1));
  const double wxMin = wxOf(0), wxMax = wxOf(width());
  const double wyMin = wyOf(height()), wyMax = wyOf(0);
  for (double gx = std::floor(wxMin / kGridStep) * kGridStep; gx <= wxMax; gx += kGridStep)
    p.drawLine(QPointF(sx(gx), 0), QPointF(sx(gx), height() - 1));
  for (double gy = std::floor(wyMin / kGridStep) * kGridStep; gy <= wyMax; gy += kGridStep)
    p.drawLine(QPointF(0, sy(gy)), QPointF(width() - 1, sy(gy)));

  // ---- 点云：2×2 像素块（100 px/m 下约 2cm，与原画布观感一致）----
  auto drawCloud = [&](const std::vector<float>& xy, const QColor& c) {
    for (size_t i = 0; i + 1 < xy.size(); i += 2) {
      const int s = (int)std::lround(sx(xy[i]));
      const int t = (int)std::lround(sy(xy[i + 1]));
      p.fillRect(QRect(s - 1, t - 1, 2, 2), c);
    }
  };
  if (snap_.hasMapping) drawCloud(snap_.mappingXY, kMappingCloud);  // 蓝：配准后
  if (snap_.hasInput) drawCloud(snap_.inputXY, kInputCloud);        // 红：配准前

  // ---- 位姿箭头（蓝色，几何同原工程：L=0.15m，杆半宽 r/2=0.025m，头半宽 r=0.05m）----
  if (snap_.hasPose) {
    const double cosA = std::cos(snap_.poseYaw), sinA = std::sin(snap_.poseYaw);
    const double L = 0.15, r = 0.05;
    auto toWin = [&](double lx, double ly) {
      return QPointF(sx(snap_.poseX + lx * cosA - ly * sinA),
                     sy(snap_.poseY + lx * sinA + ly * cosA));
    };
    // 拆成杆 + 头两个凸形
    QPolygonF shaft{toWin(0, -r / 2), toWin(L / 2, -r / 2), toWin(L / 2, r / 2), toWin(0, r / 2)};
    QPolygonF head{toWin(L / 2, -r), toWin(L, 0), toWin(L / 2, r)};
    p.setPen(Qt::NoPen);
    p.setBrush(kPoseArrow);
    p.drawPolygon(shaft);
    p.drawPolygon(head);
  }
}

void SlamView::wheelEvent(QWheelEvent* e) {
  const double mx = e->position().x(), my = e->position().y();
  const double wx = wxOf(mx), wy = wyOf(my);  // 光标下的世界点
  const double dy = e->angleDelta().y();
  const double f = dy > 0 ? 0.9 : 1.1;  // 滚轮上 = 放大
  mPerPx_ = std::min(0.2, std::max(0.001, mPerPx_ * f));
  if (!follow_) {  // 跟随模式中心由 paintEvent 锁到 pose，缩放只改倍率
    // 缩放后平移中心，使光标下的世界点仍映到光标
    cx_ = wx - (mx - width() / 2.0) * mPerPx_;
    cy_ = wy + (my - height() / 2.0) * mPerPx_;
  }
  update();
  Q_EMIT viewChanged();
}

void SlamView::mousePressEvent(QMouseEvent* e) {
  if (e->button() == Qt::LeftButton) {
    dragging_ = true;
    lastMouse_ = e->position();
  }
}

void SlamView::mouseMoveEvent(QMouseEvent* e) {
  if (!dragging_) return;
  const QPointF d = e->position() - lastMouse_;
  lastMouse_ = e->position();
  follow_ = false;  // 手动平移即脱离跟随（v 键恢复）
  cx_ -= d.x() * mPerPx_;
  cy_ += d.y() * mPerPx_;
  update();
  Q_EMIT viewChanged();
}

void SlamView::mouseReleaseEvent(QMouseEvent*) { dragging_ = false; }

void SlamView::keyPressEvent(QKeyEvent* e) {
  switch (e->key()) {
    case Qt::Key_Escape:
    case Qt::Key_Q:
      Q_EMIT quitRequested();
      break;
    case Qt::Key_V:
      mPerPx_ = kDefaultMPerPx;
      follow_ = true;  // 中心交回 paintEvent 锁定 pose（无 pose 时保持现值）
      update();
      Q_EMIT viewChanged();
      break;
    default:
      QWidget::keyPressEvent(e);
  }
}

}  // namespace flat_sim_viewer
