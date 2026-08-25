#include "icp2d.h"
#include "point_to_point.h"

namespace slam
{
    /**
     * @brief 源点云通过变换后存储在目标点云中
     *
     * @param origin 源点云
     * @param pose 姿态
     * @param result 目标点云
     */
    static void TransformPointCloud(const unity::PointCloud &origin,
                                    const unity::Rigid2f &pose, unity::PointCloud &result)
    {
        result.clear();
        result.reserve(origin.size());
        for (const unity::PointType &p : origin)
        {
            result.push_back(pose * p);
        }
    }
    /**
     * @brief 遍历点云，找到距离p点最近的点
     *
     * @param p 定点
     * @param cloud 匹配的点云
     * @return int 距离最小点的idx
     */
    static int FindNearestPoint(const unity::PointType &p, const unity::PointCloud &cloud)
    {
        float min_dist(1e9); // 无穷大的点
        int nearest_idx(-1);
        for (int idx(0); idx < cloud.size(); idx++)
        {
            const unity::PointType &t(cloud[idx]);
            // p.norm() 表示计算向量p的长度
            const float dist((p - t).norm());
            if (min_dist <= dist)
                continue;
            min_dist = dist;
            nearest_idx = idx;
        }
        return nearest_idx;
    }

    Icp2D::Icp2DParams::Icp2DParams(const unity::Rigid2f &init_pose)
        : init_pose_(init_pose)
    {
    }

    /*
    遍历变换后的点云中的每个点，找到其最近点，并判断最近点的距离是否满足最大匹配距离的要求。
    如果满足要求，将这对点加入到点对集合 p_to_p 中。
    根据点对集合 p_to_p 求解出转换矩阵 delta。
    根据转换矩阵 delta 和最大误差要求判断是否满足结束条件，如果满足则跳出循环。
    更新结果姿态。
    如果达到最大迭代次数仍未跳出循环，则返回 false 表示配准失败。
    如果未达到最大迭代次数且成功配准，则返回 true 表示配准成功。
    */

    /**
     * @brief 二维点云配准,将源点云与目标点云进行配准，使得源点云的姿态与目标点云的姿态尽可能接近
     *
     * @param source 源点云，即需要进行配准的点云，上一帧点云
     * @param dest 目标点云，即与源点云进行配准的目标点云，当前点云
     * @param params ICP2D参数，包括初始化姿态、最大迭代次数、最大匹配距离、最大优化迭代次数等。
     * @param result 配准结果，即经过配准后的源点云的姿态。
     * @return true  配准成功
     * @return false 配准失败
     */
    bool Icp2D::AligenPointCloud(const unity::PointCloud &source,
                                 const unity::PointCloud &dest, const Icp2DParams &params, unity::Rigid2f &result)
    {
        int icp_iter(0);
        result = params.init_pose_; // 设置为初始姿态，初始pose就是之前粗匹配的pose

        unity::PointCloud transformed_cloud;     // 变换后的点云
        while (icp_iter < params.max_icp_times_) // 进行最大迭代次数
        {
            TransformPointCloud(source, result, transformed_cloud); // 源点云通过变换后存储在目标点云中，第一次使用粗匹配的pose

            PointToPoint p_to_p(params.max_optimization_times_); // 初始化最大迭代次数
            for (const unity::PointType &p : transformed_cloud)  // 遍历变换后点云中每个点
            {
                const int nearst_idx(FindNearestPoint(p, dest)); // 找到P点距离点云最近的点，nearst_idx在点云中的idx。这里使用的变换的上一帧点云每个点和当前帧点云对比
                unity::PointType nearest(dest[nearst_idx]);      // 取出这个点
                if (nearst_idx < 0)                              // 负数，直接放弃
                    continue;
                /* p.squaredNorm() 是计算向量 p 的平方范数。
                平方范数（Squared Norm）是向量元素平方的和，然后再开平方。
                它常用于优化算法，如梯度下降法，因为计算平方范数比计算范数更快。 */
                if ((nearest - p).squaredNorm() > params.max_match_distance)//大于最大距离，直接放弃
                    continue;
                p_to_p.AddPair(p, nearest);//把p点和nearest点存储到p_to_p
            }
            /* 上面的for结束， 得到了p_to_p，存储了上一帧点云经过变换后的点云中每个点与当前帧点云中每个点的最短距离，以及最短距离对应的点云点idx。 */

            unity::Rigid2f delta;
            p_to_p.Solve(delta);
            if (delta.pose().norm() < params.max_icp_esp_ && delta.angle() < params.max_icp_esp_)//平方范数和角度都小于max_icp_esp_，满足条件，退出迭代
                break;

            result = delta * result;
            icp_iter++;
        }
        // std::cout<<"Icp times: "<<icp_iter<<std::endl;
        if (icp_iter >= params.max_icp_times_)//大于最大迭代数
            return false;
        return true;
    }

}
