// flat_sim —— ROS 话题桥接（无 TF）
//
//   发布：/<name>/odom       nav_msgs/Odometry
//         /<name>/base_scan  sensor_msgs/LaserScan（第 2 个激光起 base_scan_2 ...）
//   订阅：/<name>/cmd_vel    geometry_msgs/Twist
//
// 明确不做（需求第 10 节）：不发布 / 订阅 TF，不发 /robot_description。
// frame_id 仅是字符串占位（odom / base_link / <name>/laser），
// 坐标变换由应用层自行处理。
#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include "core/Simulator.h"

namespace flat_sim {

class RosBridge {
 public:
  // useSimTime=true：消息时间戳用累计仿真时间；false：用 wall time（默认，README 写明）
  RosBridge(Simulator& sim, bool useSimTime);

  // 每个仿真步调用一次（发布里程计与激光）
  void publish(double dt);
  void spinOnce() { ros::spinOnce(); }

 private:
  struct PerRobot {
    std::string name;
    ros::Publisher odomPub;
    std::vector<ros::Publisher> scanPubs;
    ros::Subscriber cmdSub;
  };

  Simulator& sim_;
  ros::NodeHandle nh_;
  bool useSimTime_;
  std::vector<PerRobot> pubs_;
};

}  // namespace flat_sim
