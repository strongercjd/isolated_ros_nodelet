#pragma once

#include "unity/grid_map.h"
#include "unity/point_cloud.h"
#include "unity/transform.h"

namespace slam
{
    
class LaserMapping
{
public:
    static void DoMapping(unity::GridMap& grid_map, const unity::PointCloud& aligned_cloud, const unity::Rigid2f& pose);
};

} // namespace slam
