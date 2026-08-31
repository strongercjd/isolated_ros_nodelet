#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "slam/front_end_process.h"
#include "slam_input.h"

namespace slam2d_nodelet
{

class Slam2dNodelet : public nodelet::Nodelet
{
public:
    Slam2dNodelet() = default;
    ~Slam2dNodelet() override;

private:
    virtual void onInit();

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void resetCallback(const std_msgs::Empty::ConstPtr &msg);
    void workerLoop();

    void publishResult(const InputFrame &frame);
    void publishMap(const ros::Time &stamp);
    void fillCloud(const ros::Publisher &pub, const unity::PointCloud &cloud,
                   const ros::Time &stamp);

    // 输入与算法
    SlamInput input_;
    std::unique_ptr<slam::FrontEndProcess> front_end_; // 首帧到达时创建；reset 时重建

    // 回调线程 → worker 线程
    std::deque<InputFrame> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic_bool stop_{false};
    bool reset_request_ = false; // 受 mtx_ 保护
    uint64_t dropped_ = 0;       // 队列满丢弃计数（诊断用）
    static constexpr size_t kQueueMax = 3;

    // ROS 接口（私有句柄 → /slam2d/...，输入话题名经 remap 指向 flat_sim）
    ros::Subscriber scan_sub_, odom_sub_, reset_sub_;
    ros::Publisher map_pub_, input_cloud_pub_, mapping_cloud_pub_, pose_pub_;
    nav_msgs::OccupancyGrid map_msg_; // 复用 4MB 缓冲

    // 参数与状态
    int scan_decimate_ = 1;         // ~scan_decimate：每 N 帧处理 1 帧
    double map_publish_period_ = 1.0; // ~map_publish_period：地图发布周期（s）
    bool publish_clouds_ = true;    // ~publish_clouds
    int frame_counter_ = 0;
    ros::Time last_map_time_;
};

} // namespace slam2d_nodelet
