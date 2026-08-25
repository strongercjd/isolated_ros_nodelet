// slam2d 输入适配：odom/scan 配对 + LaserScan → 雷达系点云
// 改编自 lidarslam_2d 的 unity::RosmsgReader：
//  - flat_sim 的 odom 与 scan 同频同戳发布（RosBridge::publish 用同一 stamp），
//    无需四元数插值对齐（slerp），也无需 tf（yaw 直接由四元数算出）；
//  - 队列语义上移到 nodelet 层（回调线程 → worker 线程）。
// 线程说明：pushOdom/makeFrame 均在 manager 的 ros::spin() 回调线程调用，无需加锁。
#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <deque>
#include <utility>

#include "unity/point_cloud.h"
#include "unity/transform.h"

namespace slam2d_nodelet
{

// 一帧 SLAM 输入：配对好的 odom 位姿（世界系）+ 雷达系点云
struct InputFrame
{
    unity::Rigid2f odom_pose;
    unity::PointCloud cloud; // 雷达系（与原工程 Process 入参语义一致）
    ros::Time stamp;
};

class SlamInput
{
public:
    // odom 回调：缓存最近 kOdomHistory 条
    void pushOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odoms_.emplace_back(msg->header.stamp, toRigid2f(*msg));
        while (odoms_.size() > kOdomHistory)
            odoms_.pop_front();
    }

    // scan 回调：找同戳 odom 配对；找不到则取时间上最近且早于 scan 的一条（±容差）。
    // 仍配不上返回 false（丢帧，宁缺毋滥）。
    bool makeFrame(const sensor_msgs::LaserScan::ConstPtr &scan, InputFrame &out) const
    {
        const ros::Time &t = scan->header.stamp;
        const std::pair<ros::Time, unity::Rigid2f> *best = nullptr;
        for (const auto &od : odoms_)
        {
            if (od.first == t)
            {
                best = &od;
                break; // 同戳精确命中
            }
        }
        if (best == nullptr)
        {
            // 兜底：最近且不晚于 scan 的 odom
            double best_dt = kSyncTolerance + 1.0;
            for (const auto &od : odoms_)
            {
                const double dt = (t - od.first).toSec();
                if (dt >= 0.0 && dt <= kSyncTolerance && dt < best_dt)
                {
                    best_dt = dt;
                    best = &od;
                }
            }
        }
        if (best == nullptr)
            return false;

        out.stamp = t;
        out.odom_pose = best->second;
        scanToCloud(*scan, out.cloud);
        return !out.cloud.empty();
    }

    size_t odomCount() const { return odoms_.size(); }

private:
    // 四元数 → yaw（平面假设 roll=pitch=0），替代 tf::Matrix3x3().getRPY()
    static float quatYaw(double w, double x, double y, double z)
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    static unity::Rigid2f toRigid2f(const nav_msgs::Odometry &odom)
    {
        const auto &q = odom.pose.pose.orientation;
        const float yaw = quatYaw(q.w, q.x, q.y, q.z);
        return unity::Rigid2f(yaw,
                              static_cast<float>(odom.pose.pose.position.x),
                              static_cast<float>(odom.pose.pose.position.y));
    }

    // LaserScan → 雷达系点云（含无效距离过滤，对照原工程 TransformLaser）
    static void scanToCloud(const sensor_msgs::LaserScan &scan, unity::PointCloud &cloud)
    {
        cloud.clear();
        cloud.reserve(scan.ranges.size());
        float angle = scan.angle_min;
        for (const float range : scan.ranges)
        {
            if (range >= scan.range_min && range <= scan.range_max && range <= 50.0f)
                cloud.push_back(unity::PointType(std::cos(angle) * range,
                                                 std::sin(angle) * range));
            angle += scan.angle_increment;
        }
    }

    std::deque<std::pair<ros::Time, unity::Rigid2f>> odoms_;
    static constexpr size_t kOdomHistory = 10;
    static constexpr double kSyncTolerance = 0.02; // s
};

} // namespace slam2d_nodelet
