#pragma once

#include <custom_ros_nodelet/custom_ros_nodelet.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace map_log_nodelet
{

/**
 * @brief SLAM 建图日志记录插件：把 /slam2d/* 话题写入 rosbag（LZ4），
 *        由 /mycar/cmd_vel 门控——速度非零即记录，全零即停。
 *
 * 门控状态机（仅回调线程）：
 *   IDLE      --cmd_vel 非零上升沿--> RECORDING：首次开门 + 补写最近状态
 *   RECORDING --cmd_vel 全零下降沿--> IDLE    ：暂停写入（不关文件）
 * 一次会话只写一个文件（map_log_<时间戳>.bag）：静止仅暂停写入，
 * 重新运动续写同一文件，进程退出时统一 close 落索引。
 *
 * 线程模型：manager 是单线程 spin，落盘在独立 worker 线程完成
 * （仿 slam2d_nodelet 的有界队列模式）；rosbag::Bag 仅 worker 线程触碰。
 */
class MapLogNodelet : public nodelet::Nodelet
{
public:
    MapLogNodelet() = default;
    ~MapLogNodelet() override;

private:
    virtual void onInit();

    // ---- ROS 回调（manager 单 spin 线程：只缓存与入队，不做 IO）----
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void inputCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void mappingCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    // ---- 门控（仅回调线程调用）----
    void startRecording();        // 首次开门 + 补写最近状态；重开只恢复写入
    void stopRecording();         // 暂停写入（不关文件）
    void enqueueLatestSnapshot(); // 门控重开时补写 map/pose/双点云最近值
    static bool isMoving(const geometry_msgs::Twist &v)
    {
        // 精确比较、无阈值：保持式 cmd_vel 静止时持续发字面零值
        return v.linear.x != 0.0 || v.angular.z != 0.0;
    }

    // ---- 落盘线程（唯一触碰 bag_ 的线程）----
    void workerLoop();
    void openSegment(const std::string &path);
    void closeSegment();

    // ---- 队列元素：开门命令与数据统一排序，保证 bag 时间戳单调 ----
    struct Item
    {
        enum Kind : uint8_t
        {
            kOpen,
            kMap,
            kPose,
            kInput,
            kMapping
        } kind;
        ros::Time stamp;     // 入队时刻（bag 记录时间戳，非 header.stamp）
        std::string path;    // kOpen：段文件绝对路径
        nav_msgs::OccupancyGrid::ConstPtr map;      // kMap，引用计数浅持有
        geometry_msgs::PoseStamped::ConstPtr pose;  // kPose
        sensor_msgs::PointCloud2::ConstPtr input;   // kInput
        sensor_msgs::PointCloud2::ConstPtr mapping; // kMapping
    };
    void pushItem(Item &&item); // 有界入队：满时丢最旧数据项（kOpen 不丢）

    // ---- 最近状态缓存（门控重开时补写；state_mtx_ 保护）----
    mutable std::mutex state_mtx_;
    nav_msgs::OccupancyGrid::ConstPtr last_map_;
    geometry_msgs::PoseStamped::ConstPtr last_pose_;
    sensor_msgs::PointCloud2::ConstPtr last_input_;
    sensor_msgs::PointCloud2::ConstPtr last_mapping_;

    // ---- 回调线程 → worker 线程 ----
    std::deque<Item> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic_bool stop_{false};
    bool recording_ = false; // 门控状态，仅回调线程读写
    bool opened_ = false;    // 已入队过 kOpen（一次会话只开一个文件），仅回调线程读写
    size_t dropped_ = 0;     // 队列满丢弃计数（诊断用）
    size_t queue_max_ = 64;  // ~queue_max

    // ---- worker 私有 ----
    // 按需构造：rosbag::Bag 构造即创建 encryptor 的 pluginlib::ClassLoader，
    // 该构造在 rospack 找不到 rosbag_storage 包时会抛异常（依赖 ROS_PACKAGE_PATH）。
    // 用 unique_ptr 把构造推迟到 openSegment 的 try/catch 内，环境缺失时
    // 降级为"记录不可用 + 错误日志"，不会导致 nodelet 加载失败拖垮 manager。
    std::unique_ptr<rosbag::Bag> bag_; // 仅 worker 线程触碰（Bag 非线程安全）

    // ---- 路径与参数 ----
    std::string resolveLogDir() const; // 参数 > dladdr > /proc/self/exe > CWD
    std::string bagPath() const;       // <log_dir>/map_log_<会话戳>.bag
    std::string log_dir_;
    std::string session_stamp_; // onInit 时刻，一次启动内恒定

    // ROS 接口（私有句柄，数据话题写死全局名，同 slam2d）
    ros::Subscriber cmd_vel_sub_, map_sub_, pose_sub_, input_sub_, mapping_sub_;
    std::string cmd_vel_topic_ = "/mycar/cmd_vel"; // ~cmd_vel_topic
};

} // namespace map_log_nodelet
