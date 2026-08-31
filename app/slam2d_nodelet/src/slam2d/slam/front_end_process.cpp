#include "front_end_process.h"
#include "laser_mapping.h"
#include "icp2d.h"
#include "laser_odometry.h"
#include "unity/voxel_filter.h"
#include "unity/tic_toc.h"

namespace slam
{

    void TransformPointCloud(const unity::PointCloud &source, const unity::Rigid2f &pose,
                             unity::PointCloud &result)
    {
        result.clear();
        result.reserve(source.size());
        for (const unity::PointType &p : source)
        {
            result.push_back(pose * p);
        }
    }

    FrontEndProcess::FrontEndProcess(const unity::Rigid2f &init_pose)
        : frame_id_(-1),
          global_map_(std::make_shared<unity::GridMap>(500, 500)), // 创建2000*2000的地图
          lastest_pose_(init_pose),                                  // SLAM 位姿初值（传首帧 odom 位姿可对齐世界系）
          last_pose_(init_pose),
          last_predict_pose_(init_pose)
    {
    }

    void FrontEndProcess::Process(const unity::Rigid2f &odom_pose, const unity::PointCloud &cloud)
    {
        if (frame_id_ > -1) // 初始是-1，所以第1帧数据不处理
        {
            const unity::Rigid2f odom_delta((last_odom_.inverse() * odom_pose));
            last_odom_ = odom_pose;
            // lastest_pose_ = lastest_pose_ * odom_delta;
            if (!cloud.empty())
            {
                // filter
                unity::PointCloud filted_cloud;
                unity::VoxelFilter::AdaptiveFilter(cloud, 180, filted_cloud); // 点云滤波,找到使用180个点的点云

                // scan to scan
                unity::TicToc tt;
                unity::Rigid2f current_init_pose;
                if (0 == frame_id_)                                 // 第一帧数据
                    current_init_pose = lastest_pose_ * odom_delta; // 初始姿态
                else
                    current_init_pose = lastest_pose_ * LaserOdometry::Process(prev_cloud_, filted_cloud, odom_delta.inverse()); // 点云匹配

                // std::cout << "Scan to scan use time\t" << tt.toc() << std::endl; // 计算点云之间匹配时间

                tt.tic();
                unity::Rigid2f mapping_pose;
                if (frame_id_ < 10)
                {
                    mapping_pose = current_init_pose;
                }
                else
                {
                    mapping_pose = current_init_pose;
                    unity::PointCloud map_cloud;
                    global_map_->GetSurroundCloud(current_init_pose.pose(), 50.0f, -70, map_cloud);
                    Icp2D::Icp2DParams icp_params(current_init_pose);
                    icp_params.max_match_distance = 0.05f;
                    if (!Icp2D::AligenPointCloud(filted_cloud, map_cloud, icp_params, mapping_pose))
                    {
                        std::cout << "Warning: Icp failed" << std::endl;
                        mapping_pose = current_init_pose;
                    }
                }

                // std::cout << "Scan to map use time\t" << tt.toc() << std::endl; // 计算点云匹配到地图的时间

                // mapping
                tt.tic();
                unity::PointCloud mapping_cloud;

                // mapping_pose = odom_pose;

                TransformPointCloud(cloud, mapping_pose, mapping_cloud); // 将cloud经过mapping_pose变换成为mapping_cloud

                LaserMapping::DoMapping(*global_map_, mapping_cloud, mapping_pose); // 更新map

                // std::cout << "mapping use time\t" << tt.toc() << std::endl;

                // iteration
                lastest_pose_ = mapping_pose;
                prev_cloud_ = filted_cloud;

                // 导出本帧结果，供 nodelet 层发布可视化话题
                last_pose_ = mapping_pose;
                last_predict_pose_ = current_init_pose;
                last_mapping_cloud_ = mapping_cloud;
                TransformPointCloud(cloud, current_init_pose, last_input_cloud_);
            }
        }
        else
        {
            last_odom_ = odom_pose;
        }

        frame_id_++;
    }

} // namespace slam
