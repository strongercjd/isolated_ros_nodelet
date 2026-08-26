// flat_sim —— ROS 话题桥接（无 TF）
//
//   发布：/<name>/odom       nav_msgs/Odometry
//         /<name>/base_scan  sensor_msgs/LaserScan（第 2 个激光起 base_scan_2 ...）
//   订阅：/<name>/cmd_vel    geometry_msgs/Twist
//         /heartbeat         std_msgs/Empty（控制节点心跳；首次收到后启用看门狗）
//
// 心跳看门狗：首次收到 /heartbeat 后，若连续 500ms 未再收到，视为控制节点停止，
// 对所有机器人强制 0 速，并忽略后续 cmd_vel，直到心跳恢复。
// 未收到过心跳时不干预（纯遥操等场景仍可用）。
//
// 明确不做（需求第 10 节）：不发布 / 订阅 TF，不发 /robot_description。
// frame_id 仅是字符串占位（odom / base_link / <name>/laser），
// 坐标变换由应用层自行处理。
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "core/Simulator.h"

namespace flat_sim {

class RosBridge {
 public:
  // useSimTime=true：消息时间戳用累计仿真时间；false：用 wall time（默认，README 写明）
  RosBridge(Simulator& sim, bool useSimTime);

  // 每个仿真步调用一次（发布里程计与激光）
  void publish(double dt);
  // 心跳看门狗：应在 spinOnce 之后、step 之前调用
  void applyHeartbeatWatchdog();
  void spinOnce() { ros::spinOnce(); }

 private:
  struct PerRobot {
    std::string name;
    ros::Publisher odomPub;
    std::vector<ros::Publisher> scanPubs;
    ros::Subscriber cmdSub;
  };

  void onHeartbeat(const std_msgs::Empty::ConstPtr& msg);
  void onCmdVel(const std::string& name, double v, double w);

  Simulator& sim_;
  ros::NodeHandle nh_;
  bool useSimTime_;
  std::vector<PerRobot> pubs_;

  ros::Subscriber heartbeatSub_;
  using Clock = std::chrono::steady_clock;
  Clock::time_point lastHeartbeat_{};
  bool heartbeatSeen_ = false;
  bool heartbeatTimedOut_ = false;
  static constexpr double kHeartbeatTimeoutSec = 0.5;  // 500ms
};

}  // namespace flat_sim
