// flat_sim_node —— 自研 2D 机器人仿真节点（纯平面 / 无 TF / 无 URDF）
//
// 用法：flat_sim_node --world <文件.fworld> [--gui | --headless]
// 日常请通过 tool/flat_sim/run_flat_sim.sh 启动（自动带 rosmaster 与环境）。
//
// 两条运行路径共用 SimRunner::stepOnce()（单步顺序唯一来源）：
//   --headless  纯 while 循环 + sleep_until 固定步长（不依赖 Qt）
//   --gui       Qt6 事件循环 + QTimer 步进（gui/SimApp）
//
// 无 rosmaster 时也能跑（纯本地仿真，不初始化 ROS、不刷屏）；
// master 后启动会自动探测到并挂上话题桥。
// SLAM 建图视图已拆分至独立工具 tool/flat_sim_viewer。
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "format/TextFormat.h"
#include "format/WorldLoader.h"
#include "ros/SimRunner.h"
#ifdef FLAT_SIM_HAVE_GUI
#include <QApplication>

#include "gui/SimApp.h"
#endif

namespace {

void usage() {
  std::puts(
      "flat_sim_node —— 自研 2D 机器人仿真（纯平面 / 无 TF / 无 URDF）\n"
      "\n"
      "用法: flat_sim_node --world <文件.fworld> [--gui | --headless] [-h]\n"
      "\n"
      "  --world <file>   世界文件（.fworld；机器人可内嵌或 robot_file 引用 .frobot）\n"
      "  --gui            启动 2D 俯视窗口（默认）\n"
      "  --headless       无窗口运行（CI / 无显示器环境）\n"
      "  -h, --help       本说明\n"
      "\n"
      "ROS 参数: ~sim_time:=true   消息时间戳用仿真时间（默认 wall time）\n"
      "GUI 按键: ESC / q 退出  |  l 开关激光显示  |  r 复位机器人并同步复位 SLAM\n"
      "SLAM 建图视图：另见 tool/flat_sim_viewer（独立查看工具）");
}

// headless 主循环：固定步长，按墙钟对齐（慢机自动重对齐，不追赶）
void runHeadless(flat_sim::SimRunner& runner) {
  using clk = std::chrono::steady_clock;
  const auto stepDur = std::chrono::microseconds((long long)(runner.dt() * 1e6));
  auto next = clk::now();
  while (true) {
    if (!runner.stepOnce()) break;  // Ctrl+C（ROS 已接管信号）
    next += stepDur;
    std::this_thread::sleep_until(next);
    if (clk::now() > next + stepDur) next = clk::now();
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string worldPath;
  bool headless = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--world" && i + 1 < argc) {
      worldPath = argv[++i];
    } else if (a == "--headless") {
      headless = true;
    } else if (a == "--gui") {
      headless = false;
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      usage();
      return 1;
    }
  }
  if (worldPath.empty()) {
    std::fprintf(stderr, "缺少 --world <文件.fworld>\n");
    usage();
    return 1;
  }

  // ---- 加载世界（格式错误 → 报出文件:行号，非 0 退出）----
  flat_sim::WorldDesc worldLoaded;
  try {
    worldLoaded = flat_sim::loadWorld(worldPath);
  } catch (const flat_sim::format::FormatError& e) {
    std::fprintf(stderr, "[flat_sim] 世界加载失败: %s\n", e.what());
    return 1;
  }

  // ---- 仿真运行时（ROS 延迟初始化 + 固定步长单步都在 SimRunner）----
  flat_sim::SimRunner runner(std::move(worldLoaded), argc, argv);
  const flat_sim::WorldDesc& world = runner.world();
  const flat_sim::Simulator& sim = runner.sim();
  const double dt = runner.dt();

  if (!runner.tryStartRos()) {
    std::fprintf(stderr,
                 "[flat_sim][警告] 未检测到 rosmaster（ROS_MASTER_URI=%s），"
                 "先以纯本地模式运行（话题不可用）。run_flat_sim.sh 会自动拉起 rosmaster。\n",
                 std::getenv("ROS_MASTER_URI") ? std::getenv("ROS_MASTER_URI") : "未设置");
  }

  // ---- 启动摘要 ----
  std::printf("[flat_sim] 世界 \"%s\"（%s）：%zu 个矩形障碍、%zu 个圆形障碍、%zu 台机器人，"
              "步长 %d ms（%.0f Hz），模式 %s\n",
              world.name.c_str(), world.sourceFile.c_str(), world.boxes.size(),
              world.circles.size(), world.robots.size(), world.timestepMs, 1.0 / dt,
              headless ? "headless" : "GUI");
  for (const flat_sim::Simulator::RobotState& r : sim.robots()) {
    std::printf("[flat_sim] 机器人 %s：circle r=%.2fm，激光 %zu 个；"
                "/%s/odom、/%s/base_scan、/%s/cmd_vel；另订阅 /heartbeat（500ms 看门狗）\n",
                r.name.c_str(), r.radius, r.scans.size(), r.name.c_str(), r.name.c_str(),
                r.name.c_str());
  }
  std::fflush(stdout);

  // ---- 运行：GUI（Qt6）或 headless 纯循环 ----
  if (!headless) {
#ifdef FLAT_SIM_HAVE_GUI
    if (!qEnvironmentVariableIsSet("DISPLAY") &&
        !qEnvironmentVariableIsSet("WAYLAND_DISPLAY")) {
      std::fprintf(stderr,
                   "[flat_sim] GUI 初始化失败（无显示器 / DISPLAY 未设置？）。"
                   "可改用 --headless 运行。\n");
      return 1;
    }
    QApplication app(argc, argv);
    flat_sim::SimApp simApp(runner, world);
    simApp.start();
    app.exec();
#else
    std::fprintf(stderr,
                 "[flat_sim] 本构建未启用 GUI（编译时未找到 Qt6）。"
                 "请用 --headless，或安装 qt6-base-dev 后重新 ./build.sh。\n");
    return 1;
#endif
  } else {
    runHeadless(runner);
  }

  std::printf("[flat_sim] 退出：仿真 %.1f s，共 %llu 步\n", sim.simTime(),
              (unsigned long long)sim.steps());
  if (runner.rosRunning()) ros::shutdown();
  return 0;
}
