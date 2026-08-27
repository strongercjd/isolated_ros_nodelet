#include "slam2d_nodelet.h"

#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>

#include "unity/tic_toc.h"

namespace slam2d_nodelet
{

Slam2dNodelet::~Slam2dNodelet()
{
    stop_ = true;
    cv_.notify_all();
    if (worker_.joinable())
        worker_.join();
}

void Slam2dNodelet::onInit()
{
    ros::NodeHandle pnh = getPrivateNodeHandle();

    pnh.param("scan_decimate", scan_decimate_, 1);
    pnh.param("map_publish_period", map_publish_period_, 1.0);
    pnh.param("publish_clouds", publish_clouds_, true);

    // 话题写死全局名（不再依赖 plugins.json remap）：输入 scan=/mycar/base_scan,
    //   odom=/mycar/odom, reset=/slam2d/reset；输出 map/pose 等挂在 /slam2d 下
    scan_sub_ = pnh.subscribe("/mycar/base_scan", 5, &Slam2dNodelet::scanCallback, this);
    odom_sub_ = pnh.subscribe("/mycar/odom", 10, &Slam2dNodelet::odomCallback, this);
    reset_sub_ = pnh.subscribe("/slam2d/reset", 1, &Slam2dNodelet::resetCallback, this);

    map_pub_ = pnh.advertise<nav_msgs::OccupancyGrid>("/slam2d/map", 1, true /*latch*/);
    pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("/slam2d/pose", 1);
    if (publish_clouds_)
    {
        input_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("/slam2d/input_cloud", 1);
        mapping_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("/slam2d/mapping_cloud", 1);
    }

    worker_ = std::thread(&Slam2dNodelet::workerLoop, this);
    NODELET_INFO("Slam2dNodelet initialized: scan_decimate=%d map_period=%.1fs publish_clouds=%d",
                 scan_decimate_, map_publish_period_, publish_clouds_ ? 1 : 0);
}

void Slam2dNodelet::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    input_.pushOdom(msg);
}

void Slam2dNodelet::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (scan_decimate_ > 1 && (frame_counter_++ % scan_decimate_) != 0)
        return;

    InputFrame frame;
    if (!input_.makeFrame(msg, frame))
        return; // odom 未配对上，丢帧

    std::lock_guard<std::mutex> lk(mtx_);
    if (queue_.size() >= kQueueMax)
    {
        queue_.pop_front(); // 算不过来时丢最旧、取最新（降级不积压）
        if (++dropped_ % 100 == 1)
            NODELET_WARN("queue full, dropped %lu old frames", static_cast<unsigned long>(dropped_));
    }
    queue_.push_back(std::move(frame));
    cv_.notify_one();
}

void Slam2dNodelet::resetCallback(const std_msgs::Empty::ConstPtr &)
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        reset_request_ = true;
    }
    cv_.notify_one();
    NODELET_INFO("reset requested");
}

void Slam2dNodelet::workerLoop()
{
    while (!stop_)
    {
        InputFrame frame;
        bool have = false;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return stop_.load() || !queue_.empty() || reset_request_; });
            if (stop_)
                break;
            if (reset_request_)
            {
                reset_request_ = false;
                queue_.clear();
                front_end_.reset(); // 下一帧以其 odom 位姿为初值重建，地图系重新对齐世界系
                NODELET_INFO("slam2d reset: front end will be rebuilt on next frame");
            }
            if (!queue_.empty())
            {
                frame = std::move(queue_.front());
                queue_.pop_front();
                have = true;
            }
        }
        if (!have)
            continue;

        try
        {
            if (!front_end_)
            {
                // 首帧 odom 位姿作 SLAM 初值 → 地图系与 flat_sim 世界系对齐
                front_end_.reset(new slam::FrontEndProcess(frame.odom_pose));
                NODELET_INFO("front end created at odom pose (%.2f, %.2f, %.1f deg)",
                             frame.odom_pose.pose().x(), frame.odom_pose.pose().y(),
                             frame.odom_pose.angle() * 57.2958f);
            }

            unity::TicToc tt;
            tt.tic();
            front_end_->Process(frame.odom_pose, frame.cloud);
            const double cost_ms = tt.toc();

            publishResult(frame);
            if ((frame.stamp - last_map_time_).toSec() >= map_publish_period_)
            {
                last_map_time_ = frame.stamp;
                publishMap(frame.stamp);
            }
            NODELET_DEBUG("frame processed in %.1f ms", cost_ms);
            static uint32_t report = 0;
            if (++report % 100 == 1)
                NODELET_INFO("frame #%u pose(%.2f, %.2f, %.1f deg) cost %.1f ms",
                             report, front_end_->GetLastPose().pose().x(),
                             front_end_->GetLastPose().pose().y(),
                             front_end_->GetLastPose().angle() * 57.2958f, cost_ms);
        }
        catch (const std::exception &e)
        {
            NODELET_ERROR("slam2d process error: %s", e.what());
        }
        catch (...)
        {
            NODELET_ERROR("slam2d process unknown error");
        }
    }
}

void Slam2dNodelet::publishResult(const InputFrame &frame)
{
    if (!front_end_)
        return;
    const unity::Rigid2f pose = front_end_->GetLastPose();

    geometry_msgs::PoseStamped pm;
    pm.header.stamp = frame.stamp;
    pm.header.frame_id = "map";
    pm.pose.position.x = pose.pose().x();
    pm.pose.position.y = pose.pose().y();
    pm.pose.orientation.w = std::cos(pose.angle() / 2.0f);
    pm.pose.orientation.z = std::sin(pose.angle() / 2.0f);
    pose_pub_.publish(pm);

    if (!publish_clouds_)
        return;
    fillCloud(input_cloud_pub_, front_end_->GetLastInputCloud(), frame.stamp);
    fillCloud(mapping_cloud_pub_, front_end_->GetLastMappingCloud(), frame.stamp);
}

void Slam2dNodelet::fillCloud(const ros::Publisher &pub, const unity::PointCloud &cloud,
                              const ros::Time &stamp)
{
    if (!pub || cloud.empty())
        return;

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier mod(msg);
    // 此版本的 setPointCloud2FieldsByString 只认 xyz/rgb/rgba（无 xy），用显式字段：
    mod.setPointCloud2Fields(2, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32); // point_step = 8
    mod.resize(cloud.size()); // 此版本 resize 单参；height==1 时设 width=n

    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    for (const unity::PointType &p : cloud)
    {
        *it_x = p.x();
        *it_y = p.y();
        ++it_x;
        ++it_y;
    }
    pub.publish(msg);
}

void Slam2dNodelet::publishMap(const ros::Time &stamp)
{
    if (!front_end_)
        return;
    const unity::GridMap &map = front_end_->GetGlobalMap();
    const int W = map.width(), H = map.height();
    if (W <= 0 || H <= 0)
        return;

    map_msg_.header.stamp = stamp;
    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = map.resolution();
    map_msg_.info.width = W;
    map_msg_.info.height = H;
    // GridMap 构造时世界原点居中 → 左下角 = (-W/2*res, -H/2*res)
    map_msg_.info.origin.position.x = -0.5 * W * map.resolution();
    map_msg_.info.origin.position.y = -0.5 * H * map.resolution();
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.assign(static_cast<size_t>(W) * H, 0);

    // GridMap: data_[row*W+col]，row0 = y 最大侧（图像式）；
    // OccupancyGrid: row0 = y 最小侧 → 行翻转。
    // 值域映射：+127 自由 → 0，-127 占用 → 100，0 未观测 → 50（线性保留证据强度）。
    const std::vector<int8_t> &cells = map.data();
    for (int row = 0; row < H; ++row)
    {
        const int msg_row = H - 1 - row;
        int8_t *dst = &map_msg_.data[static_cast<size_t>(msg_row) * W];
        const int8_t *src = &cells[static_cast<size_t>(row) * W];
        for (int col = 0; col < W; ++col)
        {
            const int occ = ((127 - static_cast<int>(src[col])) * 100 + 127) / 254;
            dst[col] = static_cast<int8_t>(occ);
        }
    }
    map_pub_.publish(map_msg_);
}

} // namespace slam2d_nodelet

PLUGINLIB_EXPORT_CLASS(slam2d_nodelet::Slam2dNodelet, nodelet::Nodelet)
