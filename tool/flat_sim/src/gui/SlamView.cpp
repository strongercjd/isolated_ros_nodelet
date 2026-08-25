// flat_sim —— 右侧 SLAM 建图视图（SDL2 实现）
// 仅当 CMake 定义了 FLAT_SIM_HAVE_GUI（找到 SDL2）时参与编译。
#ifdef FLAT_SIM_HAVE_GUI

#include "gui/SlamView.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace flat_sim {
namespace {

// 与 lidarslam_2d 的 cv_ui 画布保持一致的颜色 / 尺度
const SDL_Color kGridLine{96, 96, 96, 255};     // 5m 网格线
const SDL_Color kInputCloud{255, 0, 0, 255};    // 配准前点云（红）
const SDL_Color kMappingCloud{0, 0, 255, 255};  // 配准后点云（蓝）
const SDL_Color kPoseArrow{0, 0, 255, 255};     // 位姿箭头（蓝）
const double kDefaultMPerPx = 0.01;             // cv_ui scalar：100 px/m
const double kGridStep = 5.0;                   // 网格间距（米）

// 凸多边形扇形填充（实现参照 Gui::Impl::polygon；箭头拆成两个凸形调用）
void fillConvex(SDL_Renderer* ren, const std::vector<SDL_FPoint>& pts, SDL_Color c) {
  if (pts.size() < 3) return;
#if SDL_VERSION_ATLEAST(2, 0, 18)
  std::vector<SDL_Vertex> vs;
  vs.reserve(pts.size());
  for (const SDL_FPoint& p : pts) vs.push_back(SDL_Vertex{p, c, {0.0f, 0.0f}});
  std::vector<int> idx;
  idx.reserve((pts.size() - 2) * 3);
  for (size_t i = 1; i + 1 < vs.size(); ++i)
    idx.insert(idx.end(), {0, (int)i, (int)i + 1});
  SDL_RenderGeometry(ren, nullptr, vs.data(), (int)vs.size(), idx.data(), (int)idx.size());
#else
  SDL_SetRenderDrawColor(ren, c.r, c.g, c.b, c.a);
  for (size_t i = 0; i < pts.size(); ++i) {
    const SDL_FPoint& a = pts[i];
    const SDL_FPoint& b = pts[(i + 1) % pts.size()];
    SDL_RenderDrawLineF(ren, a.x, a.y, b.x, b.y);
  }
#endif
}

}  // namespace

struct SlamView::Impl {
  int px = 0, py = 0, pw = 0, ph = 0;  // pane（窗口系）
  double mPerPx = kDefaultMPerPx;      // 米 / 像素（越小越放大）
  double cx = 0.0, cy = 0.0;           // 视图中心（世界系）
  // 跟随位姿：机器人初始不在原点（box_house 为 (6,-4)），固定 (0,0) 中心会把
  // 箭头放出视口；默认锁定最新 pose，拖拽即脱离，v 键恢复。
  bool follow = true;

  SDL_Texture* mapTex = nullptr;
  int texW = 0, texH = 0;
  uint32_t texSeq = 0;  // 已上传的地图代数；0 = 尚无

  double pxPerM() const { return 1.0 / mPerPx; }
  // 世界 → 窗口（y 翻转）
  double sx(double wx) const { return px + pw / 2.0 + (wx - cx) * pxPerM(); }
  double sy(double wy) const { return py + ph / 2.0 - (wy - cy) * pxPerM(); }
  // 窗口 → 世界
  double wxOf(int s) const { return cx + (s - px - pw / 2.0) * mPerPx; }
  double wyOf(int s) const { return cy - (s - py - ph / 2.0) * mPerPx; }

  void updateTexture(SDL_Renderer* ren, const SlamSnapshot::Map& m) {
    if (!m.data || m.width <= 0 || m.height <= 0) return;
    if (!mapTex || texW != m.width || texH != m.height) {
      if (mapTex) SDL_DestroyTexture(mapTex);
      mapTex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
                                 m.width, m.height);
      texW = m.width;
      texH = m.height;
      texSeq = 0;  // 尺寸变化，强制全量重填
    }
    if (!mapTex) return;

    uint8_t* pixels = nullptr;
    int pitch = 0;
    if (SDL_LockTexture(mapTex, nullptr, (void**)&pixels, &pitch) != 0) return;
    const int8_t* src = m.data->data();
    for (int row = 0; row < m.height; ++row) {
      // 消息 row0 = y 最小（标准语义）；纹理 row0 = 显示顶部（y 最大）→ 行翻转
      uint8_t* dst = pixels + (size_t)(m.height - 1 - row) * pitch;
      const int8_t* srow = src + (size_t)row * m.width;
      for (int col = 0; col < m.width; ++col) {
        // occ 0-100（50=未观测）→ 灰度：自由白 / 占用黑，对应原 value+127 的还原
        const int occ = srow[col];
        const uint8_t g = (uint8_t)(255 - occ * 255 / 100);
        dst[col * 3 + 0] = g;
        dst[col * 3 + 1] = g;
        dst[col * 3 + 2] = g;
      }
    }
    SDL_UnlockTexture(mapTex);
    texSeq = m.seq;
  }
};

SlamView::SlamView() : impl_(new Impl) {}
SlamView::~SlamView() {
  if (impl_->mapTex) SDL_DestroyTexture(impl_->mapTex);
}

void SlamView::setPane(int x, int y, int w, int h) {
  impl_->px = x;
  impl_->py = y;
  impl_->pw = w;
  impl_->ph = h;
}

bool SlamView::inside(int mx, int my) const {
  return mx >= impl_->px && mx < impl_->px + impl_->pw && my >= impl_->py &&
         my < impl_->py + impl_->ph;
}

void SlamView::handleWheel(int mx, int my, int dy) {
  Impl& v = *impl_;
  const double wx = v.wxOf(mx), wy = v.wyOf(my);  // 光标下的世界点
  const double f = dy > 0 ? 0.9 : 1.1;            // 滚轮上 = 放大
  v.mPerPx = std::min(0.2, std::max(0.001, v.mPerPx * f));
  if (v.follow) return;  // 跟随模式中心由 draw 锁到 pose，缩放只改倍率
  // 缩放后平移中心，使光标下的世界点仍映到光标
  v.cx = wx - (mx - v.px - v.pw / 2.0) * v.mPerPx;
  v.cy = wy + (my - v.py - v.ph / 2.0) * v.mPerPx;
}

void SlamView::handleDrag(int dx, int dy) {
  Impl& v = *impl_;
  v.follow = false;  // 手动平移即脱离跟随（v 键恢复）
  v.cx -= dx * v.mPerPx;
  v.cy += dy * v.mPerPx;
}

void SlamView::resetView() {
  impl_->mPerPx = kDefaultMPerPx;
  impl_->follow = true;  // 中心交回 draw 锁定 pose（无 pose 时保持现值）
}

void SlamView::draw(SDL_Renderer* ren, const SlamSnapshot& snap) {
  Impl& v = *impl_;
  if (v.pw <= 0 || v.ph <= 0) return;
  if (v.follow && snap.hasPose) {  // 视图中心锁定最新位姿
    v.cx = snap.poseX;
    v.cy = snap.poseY;
  }
  const SDL_Rect clip{v.px, v.py, v.pw, v.ph};
  SDL_RenderSetClipRect(ren, &clip);

  // 黑底（地图未到时也只有网格，一眼可辨）
  SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
  SDL_RenderFillRect(ren, &clip);

  // ---- 地图（纹理仅代数变化时重填）----
  if (snap.map.data && snap.map.seq != v.texSeq) v.updateTexture(ren, snap.map);
  if (v.mapTex) {
    const double x0 = snap.map.originX;
    const double y0 = snap.map.originY;
    const double x1 = x0 + snap.map.width * snap.map.resolution;
    const double y1 = y0 + snap.map.height * snap.map.resolution;
    const SDL_FRect dst{(float)v.sx(x0), (float)v.sy(y1),
                        (float)((x1 - x0) * v.pxPerM()), (float)((y1 - y0) * v.pxPerM())};
    SDL_RenderCopyF(ren, v.mapTex, nullptr, &dst);
  }

  // ---- 5m 网格线（与地图同层之上）----
  SDL_SetRenderDrawColor(ren, kGridLine.r, kGridLine.g, kGridLine.b, 255);
  const double wxMin = v.wxOf(v.px), wxMax = v.wxOf(v.px + v.pw);
  const double wyMin = v.wyOf(v.py + v.ph), wyMax = v.wyOf(v.py);
  for (double gx = std::floor(wxMin / kGridStep) * kGridStep; gx <= wxMax; gx += kGridStep) {
    const int s = (int)std::lround(v.sx(gx));
    SDL_RenderDrawLine(ren, s, v.py, s, v.py + v.ph - 1);
  }
  for (double gy = std::floor(wyMin / kGridStep) * kGridStep; gy <= wyMax; gy += kGridStep) {
    const int s = (int)std::lround(v.sy(gy));
    SDL_RenderDrawLine(ren, v.px, s, v.px + v.pw - 1, s);
  }

  // ---- 点云：2×2 像素块（100 px/m 下约 2cm，与原画布观感一致）----
  auto drawCloud = [&](const std::vector<float>& xy, SDL_Color c) {
    SDL_SetRenderDrawColor(ren, c.r, c.g, c.b, 255);
    for (size_t i = 0; i + 1 < xy.size(); i += 2) {
      const int s = (int)std::lround(v.sx(xy[i]));
      const int t = (int)std::lround(v.sy(xy[i + 1]));
      const SDL_Rect r{s - 1, t - 1, 2, 2};
      SDL_RenderFillRect(ren, &r);
    }
  };
  if (snap.hasMapping) drawCloud(snap.mappingXY, kMappingCloud);  // 蓝：配准后
  if (snap.hasInput) drawCloud(snap.inputXY, kInputCloud);        // 红：配准前

  // ---- 位姿箭头（蓝色，几何同原工程：L=0.15m，杆半宽 r/2=0.025m，头半宽 r=0.05m）----
  if (snap.hasPose) {
    const double cosA = std::cos(snap.poseYaw), sinA = std::sin(snap.poseYaw);
    const double L = 0.15, r = 0.05;
    auto toWin = [&](double lx, double ly) {
      return SDL_FPoint{(float)v.sx(snap.poseX + lx * cosA - ly * sinA),
                        (float)v.sy(snap.poseY + lx * sinA + ly * cosA)};
    };
    // 拆成两个凸形（杆 + 头），扇形剖分才正确
    fillConvex(ren, {toWin(0, -r / 2), toWin(L / 2, -r / 2), toWin(L / 2, r / 2), toWin(0, r / 2)},
               kPoseArrow);
    fillConvex(ren, {toWin(L / 2, -r), toWin(L, 0), toWin(L / 2, r)}, kPoseArrow);
  }

  SDL_RenderSetClipRect(ren, nullptr);
}

}  // namespace flat_sim

#endif  // FLAT_SIM_HAVE_GUI
