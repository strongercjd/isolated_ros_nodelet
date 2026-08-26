// flat_sim —— 2D 俯视图 GUI（SDL2 实现）
// 仅当 CMake 定义了 FLAT_SIM_HAVE_GUI（找到 SDL2）时参与编译。
#ifdef FLAT_SIM_HAVE_GUI

#include "gui/Gui.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

#include "core/Geometry.h"
#include "gui/SlamView.h"
#include "ros/SlamSnapshot.h"

namespace flat_sim {
namespace {

const SDL_Color kBackground{250, 250, 250, 255};
const SDL_Color kBorder{70, 70, 70, 255};       // 世界边界
const SDL_Color kObstacle{150, 150, 150, 255};  // 墙 / 障碍
const SDL_Color kLaser{90, 150, 255, 80};       // 激光射线（半透明）
const SDL_Color kHeading{40, 40, 40, 255};      // 朝向指示线

SDL_Color colorOf(const std::string& name) {
  static const std::map<std::string, SDL_Color> kColors = {
      {"red", {215, 65, 60, 255}},    {"green", {70, 165, 80, 255}},
      {"blue", {70, 110, 220, 255}},  {"yellow", {225, 195, 60, 255}},
      {"orange", {235, 150, 60, 255}}, {"purple", {160, 95, 200, 255}},
      {"black", {50, 50, 50, 255}},   {"gray", {130, 130, 130, 255}},
      {"grey", {130, 130, 130, 255}},
  };
  const auto it = kColors.find(name);
  return it != kColors.end() ? it->second : SDL_Color{215, 65, 60, 255};
}

}  // namespace

struct Gui::Impl {
  SDL_Window* win = nullptr;
  SDL_Renderer* ren = nullptr;
  WorldDesc world;
  double scale = 40.0;  // 像素 / 米
  double minX = 0.0, minY = 0.0;
  int winW = 0, winH = 0;
  const double pad = 24.0;  // 视图边距（像素）
  bool showLasers = true;

  SlamView slamView;                  // 右半：SLAM 建图视图
  std::function<void()> resetHook;    // r 复位时联动（main 里发布 /slam2d/reset）
  bool dragging = false;              // 左键拖拽 SLAM 视图

  // 分屏：x < sepX 为仿真视图，x >= sepX+2 为 SLAM 视图，中间 2px 分隔线
  int sepX() const { return winW / 2; }

  // 世界 → 屏幕（y 翻转：世界 +y 朝上，屏幕 +y 朝下）
  Vec2 toScreen(Vec2 w) const {
    return {(w.x - minX) * scale + pad, (double)winH - ((w.y - minY) * scale + pad)};
  }

  void line(Vec2 a, Vec2 b, SDL_Color c) {
    SDL_SetRenderDrawColor(ren, c.r, c.g, c.b, c.a);
    const Vec2 sa = toScreen(a), sb = toScreen(b);
    SDL_RenderDrawLineF(ren, (float)sa.x, (float)sa.y, (float)sb.x, (float)sb.y);
  }

  // 凸多边形填充（扇形三角剖分）；SDL < 2.0.18 退化为描边
  void polygon(const std::vector<Vec2>& pts, SDL_Color c) {
    if (pts.size() < 3) return;
#if SDL_VERSION_ATLEAST(2, 0, 18)
    std::vector<SDL_Vertex> vs;
    vs.reserve(pts.size());
    for (const Vec2& p : pts) {
      const Vec2 s = toScreen(p);
      vs.push_back(SDL_Vertex{{(float)s.x, (float)s.y}, c, {0.0f, 0.0f}});
    }
    std::vector<int> idx;
    idx.reserve((pts.size() - 2) * 3);
    for (size_t i = 1; i + 1 < vs.size(); ++i)
      idx.insert(idx.end(), {0, (int)i, (int)i + 1});
    SDL_RenderGeometry(ren, nullptr, vs.data(), (int)vs.size(), idx.data(), (int)idx.size());
#else
    for (size_t i = 0; i < pts.size(); ++i) line(pts[i], pts[(i + 1) % pts.size()], c);
#endif
  }

  void circle(Vec2 c, double r, SDL_Color fill) {
    const int seg = 40;
    std::vector<Vec2> pts;
    pts.reserve(seg);
    for (int i = 0; i < seg; ++i) {
      const double a = 2.0 * kPi * i / seg;
      pts.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)});
    }
    polygon(pts, fill);
  }
};

Gui::Gui(const WorldDesc& world, int winW, int winH) : impl_(new Impl) {
  impl_->world = world;
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::fprintf(stderr, "[flat_sim][GUI] SDL 初始化失败: %s\n", SDL_GetError());
    return;
  }

  // 视图范围：优先世界 size（原点居中），否则用全部几何体的包围盒
  double minX, minY, maxX, maxY;
  if (world.hasSize) {
    minX = -world.width / 2.0;
    maxX = world.width / 2.0;
    minY = -world.height / 2.0;
    maxY = world.height / 2.0;
  } else {
    minX = minY = 1e18;
    maxX = maxY = -1e18;
    auto expand = [&](Vec2 p) {
      minX = std::min(minX, p.x);
      maxX = std::max(maxX, p.x);
      minY = std::min(minY, p.y);
      maxY = std::max(maxY, p.y);
    };
    for (const BoxObstacle& b : world.boxes) {
      const double hl = b.len / 2.0, hw = b.wid / 2.0;
      for (const Vec2& c : {Vec2{-hl, -hw}, Vec2{hl, -hw}, Vec2{hl, hw}, Vec2{-hl, hw}})
        expand(toWorld(b.pose, c));
    }
    for (const CircleObstacle& c : world.circles) {
      expand(c.center + Vec2{c.radius, c.radius});
      expand(c.center - Vec2{c.radius, c.radius});
    }
    for (const RobotDesc& r : world.robots) {
      expand(r.pose.p + Vec2{r.radius, r.radius});
      expand(r.pose.p - Vec2{r.radius, r.radius});
    }
    if (minX > maxX) {  // 空世界兜底
      minX = -10.0; maxX = 10.0; minY = -10.0; maxY = 10.0;
    }
  }
  const double margin = 0.5;  // 米
  minX -= margin; minY -= margin; maxX += margin; maxY += margin;

  // 左半为仿真视图：按半宽拟合（右半给 SLAM 建图视图）
  const int viewW = winW / 2;
  impl_->scale = std::min((viewW - 2 * impl_->pad) / (maxX - minX),
                          (winH - 2 * impl_->pad) / (maxY - minY));
  if (!(impl_->scale > 0.0) || !std::isfinite(impl_->scale)) impl_->scale = 40.0;
  impl_->winW = winW;
  impl_->winH = winH;
  const double cx = (minX + maxX) / 2.0, cy = (minY + maxY) / 2.0;
  impl_->minX = cx - (viewW / 2.0 - impl_->pad) / impl_->scale;
  impl_->minY = cy - (winH / 2.0 - impl_->pad) / impl_->scale;
  impl_->slamView.setPane(winW / 2 + 2, 0, winW - winW / 2 - 2, winH);

  // 标题用纯 ASCII：SDL 写 X11 的 WM_NAME 时若 locale 非 UTF-8，中文/长破折号
  // 会被按字节塞入，窗口装饰条显示成乱码（_NET_WM_NAME 正常但不少 WM 读旧属性）。
  impl_->win = SDL_CreateWindow(
      (world.name + " - flat_sim (ESC=quit l=laser r=reset v=view mouse=slam-zoom)").c_str(),
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, winW, winH,
                                SDL_WINDOW_SHOWN);
  if (!impl_->win) {
    std::fprintf(stderr, "[flat_sim][GUI] 创建窗口失败: %s\n", SDL_GetError());
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    return;
  }
  impl_->ren = SDL_CreateRenderer(impl_->win, -1, SDL_RENDERER_ACCELERATED);
  if (!impl_->ren)  // 无 GPU 环境退化为软件渲染
    impl_->ren = SDL_CreateRenderer(impl_->win, -1, SDL_RENDERER_SOFTWARE);
  if (!impl_->ren) {
    std::fprintf(stderr, "[flat_sim][GUI] 创建渲染器失败: %s\n", SDL_GetError());
    SDL_DestroyWindow(impl_->win);
    impl_->win = nullptr;
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
  }
}

Gui::~Gui() {
  if (impl_->ren) SDL_DestroyRenderer(impl_->ren);
  if (impl_->win) SDL_DestroyWindow(impl_->win);
  SDL_QuitSubSystem(SDL_INIT_VIDEO);
}

bool Gui::valid() const { return impl_->ren != nullptr; }

void Gui::setResetHook(std::function<void()> hook) { impl_->resetHook = std::move(hook); }

bool Gui::poll(Simulator& sim) {
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_QUIT:
        return false;
      case SDL_KEYDOWN:
        switch (e.key.keysym.sym) {
          case SDLK_ESCAPE:
          case SDLK_q:
            return false;
          case SDLK_l:
            impl_->showLasers = !impl_->showLasers;
            break;
          case SDLK_r:
            sim.reset();
            if (impl_->resetHook) impl_->resetHook();
            break;
          case SDLK_v:
            impl_->slamView.resetView();
            break;
          default:
            break;
        }
        break;
      case SDL_MOUSEWHEEL: {
        // 滚轮事件里取当前光标（SDL_MOUSEWHEEL 自带坐标需 2.0.18+，取全局状态更稳）
        int mx = 0, my = 0;
        SDL_GetMouseState(&mx, &my);
        if (impl_->slamView.inside(mx, my)) impl_->slamView.handleWheel(mx, my, e.wheel.y);
        break;
      }
      case SDL_MOUSEBUTTONDOWN:
        if (e.button.button == SDL_BUTTON_LEFT && impl_->slamView.inside(e.button.x, e.button.y))
          impl_->dragging = true;
        break;
      case SDL_MOUSEBUTTONUP:
        if (e.button.button == SDL_BUTTON_LEFT) impl_->dragging = false;
        break;
      case SDL_MOUSEMOTION:
        if (impl_->dragging && (e.motion.state & SDL_BUTTON_LMASK))
          impl_->slamView.handleDrag(e.motion.xrel, e.motion.yrel);
        break;
      default:
        break;
    }
  }
  return true;
}

void Gui::draw(const Simulator& sim, const SlamSnapshot* slam) {
  Impl& g = *impl_;
  const WorldDesc& w = g.world;
  SDL_SetRenderDrawColor(g.ren, kBackground.r, kBackground.g, kBackground.b, 255);
  SDL_RenderClear(g.ren);

  // 左半：仿真视图（裁剪到分隔线，避免越界画到右半）
  const SDL_Rect leftClip{0, 0, g.sepX(), g.winH};
  SDL_RenderSetClipRect(g.ren, &leftClip);

  // 世界边界
  if (w.hasSize) {
    const double hw = w.width / 2.0, hh = w.height / 2.0;
    g.line({-hw, -hh}, {hw, -hh}, kBorder);
    g.line({hw, -hh}, {hw, hh}, kBorder);
    g.line({hw, hh}, {-hw, hh}, kBorder);
    g.line({-hw, hh}, {-hw, -hh}, kBorder);
  }

  // 障碍：矩形（含旋转）与圆
  for (const BoxObstacle& b : w.boxes) {
    const double hl = b.len / 2.0, hw = b.wid / 2.0;
    std::vector<Vec2> pts;
    pts.reserve(4);
    for (const Vec2& c : {Vec2{-hl, -hw}, Vec2{hl, -hw}, Vec2{hl, hw}, Vec2{-hl, hw}})
      pts.push_back(toWorld(b.pose, c));
    g.polygon(pts, kObstacle);
  }
  for (const CircleObstacle& c : w.circles) g.circle(c.center, c.radius, kObstacle);

  // 机器人 + 激光
  for (const Simulator::RobotState& r : sim.robots()) {
    if (g.showLasers) {
      SDL_SetRenderDrawBlendMode(g.ren, SDL_BLENDMODE_BLEND);
      for (const Simulator::Scan& s : r.scans) {
        for (size_t k = 0; k < s.ranges.size(); ++k) {
          const double ang = s.angleStart + s.angleInc * (double)k;
          const double d = s.ranges[k];
          if (!std::isfinite(d)) continue;  // 无回波光束不画
          g.line(s.origin.p, s.origin.p + Vec2{d * std::cos(ang), d * std::sin(ang)}, kLaser);
        }
      }
      SDL_SetRenderDrawBlendMode(g.ren, SDL_BLENDMODE_NONE);
    }
    g.circle(r.pose.p, r.radius, colorOf(r.color));
    const Vec2 tip = r.pose.p + Vec2{1.9 * r.radius * std::cos(r.pose.yaw),
                                     1.9 * r.radius * std::sin(r.pose.yaw)};
    g.line(r.pose.p, tip, kHeading);
  }
  SDL_RenderSetClipRect(g.ren, nullptr);

  // 分隔线（2px）
  SDL_SetRenderDrawColor(g.ren, kBorder.r, kBorder.g, kBorder.b, 255);
  SDL_Rect sep{g.sepX(), 0, 2, g.winH};
  SDL_RenderFillRect(g.ren, &sep);

  // 右半：SLAM 建图视图（无数据时 SlamView 自画黑底 + 网格占位）
  if (slam) g.slamView.draw(g.ren, *slam);

  SDL_RenderPresent(g.ren);
}

}  // namespace flat_sim

#endif  // FLAT_SIM_HAVE_GUI
