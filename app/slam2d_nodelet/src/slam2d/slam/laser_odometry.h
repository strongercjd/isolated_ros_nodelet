#pragma once

#include "unity/point_cloud.h"
#include "unity/transform.h"

namespace slam
{
/**
 * @brief 雷达里程计
 *
 */
class LaserOdometry
{
public:
    static unity::Rigid2f Process(
        const unity::PointCloud& prev_cloud,
        const unity::PointCloud& curr_cloud, const unity::Rigid2f& odom);
};

} // namespace slam
