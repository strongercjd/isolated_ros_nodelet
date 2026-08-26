// flat_sim_node —— 自研 2D 机器人仿真节点（纯平面 / 无 TF / 无 URDF）
//
// 用法：flat_sim_node --world <文件.fworld> [--gui | --headless]
// 日常请通过 tool/flat_sim/run_flat_sim.sh 启动（自动带 rosmaster 与环境）。
//
// 无 rosmaster 时也能跑（纯本地仿真，不初始化 ROS、不刷屏）；
// master 后启动会自动探测到并挂上话题桥。
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "core/Simulator.h"
#include "format/TextFormat.h"
#include "format/WorldLoader.h"
#include "ros/RosBridge.h"
#include "ros/SlamListener.h"
#ifdef FLAT_SIM_HAVE_GUI
#include "gui/Gui.h"
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
      "          |  v 恢复 SLAM 视图跟随；右半视图：滚轮缩放 / 左键拖拽平移(脱离跟随)");
}

// 探测 ROS_MASTER_URI（http://host:port）端口是否可连。
// 用裸 socket 而不用 ros::master::check()：后者要求 ros::init 已执行。
bool masterReachable() {
  const char* env = std::getenv("ROS_MASTER_URI");
  std::string uri = env && *env ? env : "http://127.0.0.1:11311";
  const size_t slash = uri.find("//");
  std::string hostport = slash == std::string::npos ? uri : uri.substr(slash + 2);
  const size_t colon = hostport.find(':');
  const std::string host = colon == std::string::npos ? hostport : hostport.substr(0, colon);
  const std::string port = colon == std::string::npos ? "11311" : hostport.substr(colon + 1);
  if (host.empty() || port.empty()) return false;

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
    if (res) freeaddrinfo(res);
    return false;
  }
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  bool ok = false;
  if (fd >= 0) {
    ok = connect(fd, res->ai_addr, res->ai_addrlen) == 0;
    close(fd);
  }
  freeaddrinfo(res);
  return ok;
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
  flat_sim::WorldDesc world;
  try {
    world = flat_sim::loadWorld(worldPath);
  } catch (const flat_sim::format::FormatError& e) {
    std::fprintf(stderr, "[flat_sim] 世界加载失败: %s\n", e.what());
    return 1;
  }
  flat_sim::Simulator sim(world);
  const double dt = world.timestepMs / 1000.0;

  // ---- ROS（无 TF；无 master 时延迟初始化）----
  // NoRosout：不挂 rosout appender（日志直接走 stdout/stderr），
  // 否则 master 异常时它会不停重试注册 /rosout，把控制台刷爆。
  // 注意：ros::init / NodeHandle 必须等 master 可达后才碰——roscpp 一旦
  // ros::start() 就会开后台线程向 master 注册并无限重试刷屏。
  std::unique_ptr<flat_sim::RosBridge> bridge;
  std::unique_ptr<flat_sim::SlamListener> slamListener;
  bool rosRunning = false;
  auto tryStartRos = [&]() -> bool {
    if (rosRunning) return true;
    if (!masterReachable()) return false;
    ros::init(argc, argv, "flat_sim", ros::init_options::NoRosout);
    bool useSimTime = false;
    {
      ros::NodeHandle pnh("~");
      pnh.param("sim_time", useSimTime, false);
    }
    bridge.reset(new flat_sim::RosBridge(sim, useSimTime));
    // SLAM 视图数据源（回调随主循环的 bridge->spinOnce() 派发）
    slamListener.reset(new flat_sim::SlamListener());
    rosRunning = true;
    std::fprintf(stderr, "[flat_sim] rosmaster 就绪（%s），话题已启用\n",
                 std::getenv("ROS_MASTER_URI") ? std::getenv("ROS_MASTER_URI") : "?");
    return true;
  };
  if (!tryStartRos()) {
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

  // ---- GUI（可选；headless 或未编译 GUI 时跳过）----
#ifdef FLAT_SIM_HAVE_GUI
  std::unique_ptr<flat_sim::Gui> gui;
  if (!headless) {
    gui.reset(new flat_sim::Gui(world));
    // r 键复位机器人时联动复位 SLAM（否则 odom 跳变会打爆建图）
    gui->setResetHook([&]() {
      if (slamListener) slamListener->publishReset();
    });
    if (!gui->valid()) {
      std::fprintf(stderr,
                   "[flat_sim] GUI 初始化失败（无显示器 / DISPLAY 未设置？）。"
                   "可改用 --headless 运行。\n");
      return 1;
    }
  }
#else
  if (!headless) {
    std::fprintf(stderr,
                 "[flat_sim] 本构建未启用 GUI（编译时未找到 SDL2）。"
                 "请用 --headless，或安装 libsdl2-dev 后重新 ./build.sh。\n");
    return 1;
  }
#endif

  // ---- 主循环：固定步长，按墙钟对齐（慢机自动重对齐，不追赶）----
  using clk = std::chrono::steady_clock;
  const auto stepDur = std::chrono::microseconds((long long)(dt * 1e6));
  auto next = clk::now();
  uint64_t spinCount = 0;
  while (true) {
    if (rosRunning && !ros::ok()) break;  // Ctrl+C（ROS 已接管信号）
    if (bridge) {
      bridge->spinOnce();               // 先收心跳 / cmd_vel
      bridge->applyHeartbeatWatchdog();  // 超时则本步起强制 0 速
    }
    sim.step(dt);
    if (!rosRunning && ++spinCount % 100 == 0) tryStartRos();  // 每 100 步探一次 master
    if (bridge) bridge->publish(dt);
#ifdef FLAT_SIM_HAVE_GUI
    if (gui) {
      if (!gui->poll(sim)) break;  // 关窗 / ESC → 退出
      // 右半 SLAM 视图：无 ROS（master 未起）时传 nullptr 显示网格占位
      flat_sim::SlamSnapshot slamSnap;
      if (slamListener) slamSnap = slamListener->snapshot();
      gui->draw(sim, slamListener ? &slamSnap : nullptr);
    }
#endif
    next += stepDur;
    std::this_thread::sleep_until(next);
    if (clk::now() > next + stepDur) next = clk::now();
  }

  std::printf("[flat_sim] 退出：仿真 %.1f s，共 %llu 步\n", sim.simTime(),
              (unsigned long long)sim.steps());
  if (rosRunning) ros::shutdown();
  return 0;
}
