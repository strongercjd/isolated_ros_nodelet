// flat_sim —— ROS 桥接实现（四元数直接用三角函数算，不引入 tf）
#include "ros/RosBridge.h"

#include <cmath>

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
          sim_.setCmd(name, m->linear.x, m->angular.z);
        });
    pubs_.push_back(std::move(pr));
  }
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
