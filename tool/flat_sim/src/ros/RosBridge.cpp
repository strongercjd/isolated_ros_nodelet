// flat_sim —— ROS 桥接实现（四元数直接用三角函数算，不引入 tf）
#include "ros/RosBridge.h"

#include <cmath>
#include <cstdio>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

namespace flat_sim {

RosBridge::RosBridge(Simulator& sim, bool useSimTime) : sim_(sim), nh_(), useSimTime_(useSimTime) {
  for (const Simulator::RobotState& r : sim_.robots()) {
    PerRobot pr;
    pr.name = r.name;
    pr.odomPub = nh_.advertise<nav_msgs::Odometry>("/" + r.name + "/odom", 20);
    for (size_t li = 0; li < r.scans.size(); ++li) {
      const std::string topic = li == 0 ? "base_scan" : "base_scan_" + std::to_string(li + 1);
      pr.scanPubs.push_back(nh_.advertise<sensor_msgs::LaserScan>("/" + r.name + "/" + topic, 5));
    }
    const std::string name = r.name;  // 名字在运行期不变，按值捕获进回调
    pr.cmdSub = nh_.subscribe<geometry_msgs::Twist>(
        "/" + name + "/cmd_vel", 20,
        [this, name](const geometry_msgs::Twist::ConstPtr& m) {
          onCmdVel(name, m->linear.x, m->angular.z);
        });
    pubs_.push_back(std::move(pr));
  }

  heartbeatSub_ = nh_.subscribe<std_msgs::Empty>(
      "/heartbeat", 10, &RosBridge::onHeartbeat, this);
}

void RosBridge::onHeartbeat(const std_msgs::Empty::ConstPtr&) {
  lastHeartbeat_ = Clock::now();
  if (!heartbeatSeen_) {
    heartbeatSeen_ = true;
    std::fprintf(stderr, "[flat_sim] 收到控制心跳 /heartbeat，启用 500ms 看门狗\n");
  }
  if (heartbeatTimedOut_) {
    heartbeatTimedOut_ = false;
    std::fprintf(stderr, "[flat_sim] 控制心跳恢复，解除 0 速锁定\n");
  }
}

void RosBridge::onCmdVel(const std::string& name, double v, double w) {
  if (heartbeatTimedOut_) return;  // 心跳超时期间忽略速度指令
  sim_.setCmd(name, v, w);
}

void RosBridge::applyHeartbeatWatchdog() {
  if (!heartbeatSeen_) return;
  const double age =
      std::chrono::duration<double>(Clock::now() - lastHeartbeat_).count();
  if (age <= kHeartbeatTimeoutSec) return;

  if (!heartbeatTimedOut_) {
    heartbeatTimedOut_ = true;
    std::fprintf(stderr,
                 "[flat_sim][警告] 连续 %.0f ms 未收到 /heartbeat，强制所有机器人 0 速\n",
                 kHeartbeatTimeoutSec * 1000.0);
  }
  for (const PerRobot& pr : pubs_) sim_.setCmd(pr.name, 0.0, 0.0);
}

void RosBridge::publish(double dt) {
  const ros::Time stamp = useSimTime_ ? ros::Time(sim_.simTime()) : ros::Time::now();

  for (size_t i = 0; i < pubs_.size(); ++i) {
    const Simulator::RobotState& r = sim_.robots()[i];

    nav_msgs::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = "odom";      // 占位字符串，flat_sim 不维护 TF
    od.child_frame_id = "base_link";  // 同上
    od.pose.pose.position.x = r.pose.p.x;
    od.pose.pose.position.y = r.pose.p.y;
    od.pose.pose.position.z = 0.0;
    od.pose.pose.orientation.w = std::cos(r.pose.yaw / 2.0);
    od.pose.pose.orientation.z = std::sin(r.pose.yaw / 2.0);
    od.twist.twist.linear.x = r.curV;
    od.twist.twist.angular.z = r.curW;
    pubs_[i].odomPub.publish(od);

    for (size_t li = 0; li < r.scans.size() && li < pubs_[i].scanPubs.size(); ++li) {
      const Simulator::Scan& s = r.scans[li];
      sensor_msgs::LaserScan msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = r.name + "/laser";  // 占位字符串，flat_sim 不维护 TF
      msg.angle_min = (float)(-s.fov / 2.0);
      msg.angle_max = (float)(s.fov / 2.0);
      msg.angle_increment = s.samples > 1 ? (float)(s.fov / (s.samples - 1)) : 0.0f;
      msg.time_increment = 0.0f;
      msg.scan_time = (float)dt;
      msg.range_min = (float)s.rangeMin;
      msg.range_max = (float)s.rangeMax;
      msg.ranges = s.ranges;
      msg.intensities.clear();
      pubs_[i].scanPubs[li].publish(msg);
    }
  }
}

}  // namespace flat_sim
