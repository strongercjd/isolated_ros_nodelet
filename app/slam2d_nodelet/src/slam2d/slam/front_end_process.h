#pragma once

#include "unity/grid_map.h"
#include "unity/point_cloud.h"
#include "unity/transform.h"
#include <memory>

namespace slam
{
    /**
     * @brief 前端处理类
     *
     * 移植自 lidarslam_2d（剥离 cv_ui/OpenCV 显示，结果改为 getter 导出，
     * 由 nodelet 层发布 ROS 可视化话题）。
     */
    class FrontEndProcess
    {
    public:
        // init_pose：SLAM 位姿初值。传入首帧 odom 位姿可让地图系与 odom 世界系对齐。
        explicit FrontEndProcess(const unity::Rigid2f &init_pose = unity::Rigid2f(0, 0, 0));
        void Process(const unity::Rigid2f &odom_pose, const unity::PointCloud &cloud);

        unity::Rigid2f GetLatestPose() const
        {
            return lastest_pose_;
        }

        const unity::GridMap &GetGlobalMap() const
        {
            return *global_map_;
        }

        // ---- 最近一帧结果（供可视化发布）----
        unity::Rigid2f GetLastPose() const { return last_pose_; }             // 配准后位姿（蓝箭头）
        unity::Rigid2f GetLastPredictPose() const { return last_predict_pose_; } // 配准前预测位姿
        const unity::PointCloud &GetLastMappingCloud() const { return last_mapping_cloud_; } // 配准后全局点云（蓝）
        const unity::PointCloud &GetLastInputCloud() const { return last_input_cloud_; }     // 预测位姿下的全局点云（红）

    private:
        int frame_id_;
        unity::PointCloud prev_cloud_; // 上一帧点云
        unity::Rigid2f last_odom_;
        unity::Rigid2f lastest_pose_;
        std::shared_ptr<unity::GridMap> global_map_;
        unity::Rigid2f last_pose_;             // 最近一帧配准后位姿
        unity::Rigid2f last_predict_pose_;     // 最近一帧配准前预测位姿
        unity::PointCloud last_mapping_cloud_; // 配准后全局点云
        unity::PointCloud last_input_cloud_;   // 预测位姿下的全局点云
    };

} // namespace slam
