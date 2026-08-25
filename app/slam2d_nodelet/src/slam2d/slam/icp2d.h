#pragma once

#include "unity/point_cloud.h"
#include "unity/transform.h"

namespace slam
{
    /*二维点云间的配准方法，是ICP（Iterative Closest Point，迭代最近点）算法的一种应用。
    ICP是一种常见的点云配准方法，其基本思想是通过迭代寻找两个点云之间的最佳转换矩阵，使两个点云达到最佳的配准。
    */
    class Icp2D
    {
    public:
        struct Icp2DParams
        {
            const unity::Rigid2f init_pose_;  // 初始化pose，ICP算法开始时点的云的状态
            int max_icp_times_ = 40;          // ICP算法的最大迭代次数。这是为了防止ICP算法在无法得到满足结果的情况下持续进行无意义的迭代
            float max_match_distance = 0.1f;  // 寻找最近点时可以接受的最大距离。这是为了防止误匹配点导致算法出错
            float max_icp_esp_ = 0.001f;      // ICP算法中可以接受的最大误差。当ICP算法计算出的转换矩阵的误差小于这个值时，就可以认为找到了满足结果
            int max_optimization_times_ = 20; // 得到初始匹配后，优化算法的最大迭代次数。这是为了防止优化算法在无法得到更好结果的情况下持续进行无意义的迭代

            Icp2DParams(const unity::Rigid2f &init_pose);
        };

        static bool AligenPointCloud(const unity::PointCloud &source,
                                     const unity::PointCloud &dest, const Icp2DParams &params, unity::Rigid2f &result);
    };

}
