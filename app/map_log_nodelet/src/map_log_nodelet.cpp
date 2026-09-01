#include "map_log_nodelet.h"

#include <dlfcn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <ctime>
#include <vector>

namespace map_log_nodelet
{

    // 记录的话题名（与 slam2d_nodelet 发布的绝对话题名一致）
    static const char *const kMapTopic = "/slam2d/map";
    static const char *const kPoseTopic = "/slam2d/pose";
    static const char *const kInputTopic = "/slam2d/input_cloud";
    static const char *const kMappingTopic = "/slam2d/mapping_cloud";

    /** 本 .so 内的静态变量地址，供 dladdr 反推插件加载路径 */
    static const int kDlMarker = 0;

    static std::string dirNameOf(const std::string &path)
    {
        const size_t pos = path.find_last_of('/');
        if (pos == std::string::npos)
            return ".";
        if (pos == 0)
            return "/";
        return path.substr(0, pos);
    }

    /** 递归创建目录（仿 custom_ros_nodelet 的 makeDirs；C++14 无 std::filesystem） */
    static bool makeDirs(const std::string &path)
    {
        if (path.empty() || path == "." || path == "/")
            return true;
        std::string cur;
        for (size_t i = 0; i < path.size(); ++i)
        {
            cur += path[i];
            if (path[i] == '/' && i > 0)
                ::mkdir(cur.c_str(), 0755); // 已存在则忽略
        }
        ::mkdir(path.c_str(), 0755);
        struct stat st;
        return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
    }

    /** 读 /proc/self/exe 的真实路径（manager 可执行文件位置） */
    static std::string selfExePath()
    {
        char buf[4096];
        const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (n <= 0)
            return std::string();
        buf[n] = '\0';
        return std::string(buf);
    }

    /** 取路径的真实绝对路径（路径不存在时返回空） */
    static std::string realPathOf(const std::string &path)
    {
        char buf[4096];
        if (!::realpath(path.c_str(), buf))
            return std::string();
        return std::string(buf);
    }

    MapLogNodelet::~MapLogNodelet()
    {
        stop_ = true;
        cv_.notify_all();
        if (worker_.joinable())
            worker_.join();
    }

    void MapLogNodelet::onInit()
    {
        ros::NodeHandle &pnh = getPrivateNodeHandle();
        pnh.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
        int queue_max = static_cast<int>(queue_max_);
        pnh.param("queue_max", queue_max, static_cast<int>(queue_max_));
        queue_max_ = static_cast<size_t>(queue_max > 0 ? queue_max : 64);

        // 会话时间戳（一次启动内恒定，用作所有段文件的公共前缀）
        {
            std::time_t t = std::time(nullptr);
            std::tm tm_buf;
            localtime_r(&t, &tm_buf);
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
            session_stamp_ = buf;
        }

        log_dir_ = resolveLogDir();
        NODELET_INFO("MapLogNodelet initialized: cmd_vel_topic=%s log_dir=%s",
                     cmd_vel_topic_.c_str(), log_dir_.c_str());

        // 队列深度给足：回调只做浅拷贝入队，正常远不会触顶
        cmd_vel_sub_ = pnh.subscribe(cmd_vel_topic_, 10, &MapLogNodelet::cmdVelCallback, this);
        map_sub_ = pnh.subscribe(kMapTopic, 10, &MapLogNodelet::mapCallback, this);
        pose_sub_ = pnh.subscribe(kPoseTopic, 10, &MapLogNodelet::poseCallback, this);
        input_sub_ = pnh.subscribe(kInputTopic, 10, &MapLogNodelet::inputCloudCallback, this);
        mapping_sub_ = pnh.subscribe(kMappingTopic, 10, &MapLogNodelet::mappingCloudCallback, this);

        worker_ = std::thread(&MapLogNodelet::workerLoop, this);
    }

    // ---------------------------------------------------------------- 路径

    std::string MapLogNodelet::resolveLogDir() const
    {
        // 1) ~log_dir 参数显式指定
        std::string param_dir;
        getPrivateNodeHandle().param<std::string>("log_dir", param_dir, "");
        if (!param_dir.empty())
            return param_dir;

        // 2) dladdr：从本 .so 的加载路径推导 <app_runtime>/data/log
        //    manager 以绝对路径加载插件 → dli_fname = <app_runtime>/lib/libmap_log_nodelet.so
        Dl_info info;
        if (::dladdr(reinterpret_cast<const void *>(&kDlMarker), &info) && info.dli_fname)
        {
            const std::string so = realPathOf(info.dli_fname);
            if (!so.empty())
            {
                // libmap_log_nodelet.so → lib → <app_runtime> → data/log
                const std::string dir = dirNameOf(dirNameOf(so)) + "/data/log";
                NODELET_INFO("log dir derived from plugin path: %s", dir.c_str());
                return dir;
            }
        }

        // 3) /proc/self/exe：manager 可执行 → bin → <app_runtime> → data/log
        const std::string exe = selfExePath();
        if (!exe.empty())
        {
            const std::string dir = dirNameOf(dirNameOf(exe)) + "/data/log";
            NODELET_WARN("dladdr failed, log dir derived from /proc/self/exe: %s", dir.c_str());
            return dir;
        }

        // 4) 兜底（不应发生）
        NODELET_WARN("cannot locate app_runtime, fallback to CWD/map_log/");
        return "map_log";
    }

    std::string MapLogNodelet::nextSegmentPath()
    {
        char seg[8];
        std::snprintf(seg, sizeof(seg), "%03u", segment_++);
        return log_dir_ + "/map_log_" + session_stamp_ + "_seg" + seg + ".bag";
    }

    // ---------------------------------------------------------------- 门控

    void MapLogNodelet::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        const bool moving = isMoving(*msg);
        if (moving && !recording_)
            startRecording();
        else if (!moving && recording_)
            stopRecording();
    }

    void MapLogNodelet::startRecording()
    {
        recording_ = true;
        Item open;
        open.kind = Item::kOpen;
        open.stamp = ros::Time::now();
        open.path = nextSegmentPath();
        pushItem(std::move(open));
        enqueueLatestSnapshot(); // 回放第一帧即有完整地图+位姿+双点云
    }

    void MapLogNodelet::stopRecording()
    {
        recording_ = false;
        Item close;
        close.kind = Item::kClose;
        close.stamp = ros::Time::now();
        pushItem(std::move(close));
    }

    void MapLogNodelet::enqueueLatestSnapshot()
    {
        // 同一时刻补写四类最近值（空缺跳过），保证 bag 时间轴单调不回跳
        const ros::Time stamp = ros::Time::now();
        nav_msgs::OccupancyGrid::ConstPtr map;
        geometry_msgs::PoseStamped::ConstPtr pose;
        sensor_msgs::PointCloud2::ConstPtr input, mapping;
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            map = last_map_;
            pose = last_pose_;
            input = last_input_;
            mapping = last_mapping_;
        }
        if (map)
        {
            Item it;
            it.kind = Item::kMap;
            it.stamp = stamp;
            it.map = map;
            pushItem(std::move(it));
        }
        if (pose)
        {
            Item it;
            it.kind = Item::kPose;
            it.stamp = stamp;
            it.pose = pose;
            pushItem(std::move(it));
        }
        if (input)
        {
            Item it;
            it.kind = Item::kInput;
            it.stamp = stamp;
            it.input = input;
            pushItem(std::move(it));
        }
        if (mapping)
        {
            Item it;
            it.kind = Item::kMapping;
            it.stamp = stamp;
            it.mapping = mapping;
            pushItem(std::move(it));
        }
    }

    // ---------------------------------------------------------------- 数据回调

    void MapLogNodelet::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            last_map_ = msg;
        }
        if (!recording_)
            return;
        Item it;
        it.kind = Item::kMap;
        it.stamp = ros::Time::now();
        it.map = msg;
        pushItem(std::move(it));
    }

    void MapLogNodelet::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            last_pose_ = msg;
        }
        if (!recording_)
            return;
        Item it;
        it.kind = Item::kPose;
        it.stamp = ros::Time::now();
        it.pose = msg;
        pushItem(std::move(it));
    }

    void MapLogNodelet::inputCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            last_input_ = msg;
        }
        if (!recording_)
            return;
        Item it;
        it.kind = Item::kInput;
        it.stamp = ros::Time::now();
        it.input = msg;
        pushItem(std::move(it));
    }

    void MapLogNodelet::mappingCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            last_mapping_ = msg;
        }
        if (!recording_)
            return;
        Item it;
        it.kind = Item::kMapping;
        it.stamp = ros::Time::now();
        it.mapping = msg;
        pushItem(std::move(it));
    }

    // ---------------------------------------------------------------- 队列

    void MapLogNodelet::pushItem(Item &&item)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.size() >= queue_max_)
            {
                // 丢最旧的数据项（kOpen/kClose 控制项不丢）
                bool dropped_old = false;
                for (auto it = queue_.begin(); it != queue_.end(); ++it)
                {
                    if (it->kind != Item::kOpen && it->kind != Item::kClose)
                    {
                        queue_.erase(it);
                        dropped_old = true;
                        break;
                    }
                }
                if (dropped_old && ++dropped_ % 100 == 1)
                    NODELET_WARN("queue full, dropped %zu oldest data items", dropped_);
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
    }

    // ---------------------------------------------------------------- worker

    void MapLogNodelet::openSegment(const std::string &path)
    {
        const std::string dir = dirNameOf(path);
        if (!makeDirs(dir))
        {
            NODELET_ERROR("cannot create log dir %s, recording disabled", dir.c_str());
            return;
        }
        try
        {
            bag_.reset(new rosbag::Bag()); // 构造可能因 rospack 找不到包而抛（见头文件注释）
            bag_->open(path, rosbag::bagmode::Write);
            bag_->setCompression(rosbag::compression::LZ4);
            NODELET_INFO("recording started: %s", path.c_str());
        }
        catch (const std::exception &e)
        {
            bag_.reset();
            NODELET_ERROR("open bag failed: %s (%s)", path.c_str(), e.what());
        }
    }

    void MapLogNodelet::closeSegment()
    {
        if (!bag_)
            return;
        bag_->close(); // chunk 索引只在 close 时落盘，不 close 的 bag 不可读
        bag_.reset();
        NODELET_INFO("recording stopped");
    }

    void MapLogNodelet::workerLoop()
    {
        while (true)
        {
            Item item;
            {
                std::unique_lock<std::mutex> lock(mtx_);
                cv_.wait(lock, [this]
                         { return stop_.load() || !queue_.empty(); });
                if (queue_.empty())
                    break; // stop_ 且队列已排空
                item = std::move(queue_.front());
                queue_.pop_front();
            }

            try
            {
                switch (item.kind)
                {
                case Item::kOpen:
                    if (bag_) // 防御：上一个段未正常关闭
                        closeSegment();
                    openSegment(item.path);
                    break;
                case Item::kClose:
                    closeSegment();
                    break;
                case Item::kMap:
                    if (bag_)
                        bag_->write(kMapTopic, item.stamp, item.map);
                    break;
                case Item::kPose:
                    if (bag_)
                        bag_->write(kPoseTopic, item.stamp, item.pose);
                    break;
                case Item::kInput:
                    if (bag_)
                        bag_->write(kInputTopic, item.stamp, item.input);
                    break;
                case Item::kMapping:
                    if (bag_)
                        bag_->write(kMappingTopic, item.stamp, item.mapping);
                    break;
                }
            }
            catch (const std::exception &e)
            {
                NODELET_ERROR("bag write error: %s", e.what());
                closeSegment(); // 写失败即收段，避免继续写坏
            }
        }
        closeSegment(); // 正常退出路径：保证 bag 可读
    }

} // namespace map_log_nodelet

PLUGINLIB_EXPORT_CLASS(map_log_nodelet::MapLogNodelet, nodelet::Nodelet)
