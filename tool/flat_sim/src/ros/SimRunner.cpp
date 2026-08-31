// flat_sim —— 仿真运行时实现（master 探测与主循环自 main.cpp 迁入）
#include "ros/SimRunner.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>

#include <ros/ros.h>

namespace flat_sim {

SimRunner::SimRunner(WorldDesc world, int argc, char** argv)
    : world_(std::move(world)),
      dt_(world_.timestepMs / 1000.0),
      sim_(world_),
      argc_(argc),
      argv_(argv) {}

// 探测 ROS_MASTER_URI（http://host:port）端口是否可连。
// 用裸 socket 而不用 ros::master::check()：后者要求 ros::init 已执行。
bool SimRunner::masterReachable() {
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

bool SimRunner::tryStartRos() {
  if (rosRunning_) return true;
  if (!masterReachable()) return false;
  // NoRosout：不挂 rosout appender（日志直接走 stdout/stderr），
  // 否则 master 异常时它会不停重试注册 /rosout，把控制台刷爆。
  // 注意：ros::init / NodeHandle 必须等 master 可达后才碰——roscpp 一旦
  // ros::start() 就会开后台线程向 master 注册并无限重试刷屏。
  ros::init(argc_, argv_, "flat_sim", ros::init_options::NoRosout);
  bool useSimTime = false;
  {
    ros::NodeHandle pnh("~");
    pnh.param("sim_time", useSimTime, false);
  }
  bridge_.reset(new RosBridge(sim_, useSimTime));
  rosRunning_ = true;
  std::fprintf(stderr, "[flat_sim] rosmaster 就绪（%s），话题已启用\n",
               std::getenv("ROS_MASTER_URI") ? std::getenv("ROS_MASTER_URI") : "?");
  return true;
}

bool SimRunner::stepOnce() {
  if (rosRunning_ && !ros::ok()) return false;  // Ctrl+C（ROS 已接管信号）
  if (bridge_) {
    bridge_->spinOnce();                // 先收心跳 / cmd_vel
    bridge_->applyHeartbeatWatchdog();  // 超时则本步起强制 0 速
  }
  sim_.step(dt_);
  if (!rosRunning_ && ++spinCount_ % 100 == 0) tryStartRos();  // 每 100 步探一次 master
  if (bridge_) bridge_->publish(dt_);
  return true;
}

void SimRunner::reset() {
  sim_.reset();
  // 联动复位 SLAM（否则 odom 跳变会打爆建图）
  if (bridge_) bridge_->publishSlamReset();
}

}  // namespace flat_sim
