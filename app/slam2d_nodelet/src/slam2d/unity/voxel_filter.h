#pragma once

#include "point_cloud.h"

namespace unity
{
    /**
     * @brief Voxel:三维像素
     *
     */
    class VoxelFilter
    {
    public:
        static void Filter(const PointCloud &input, float grid_size, PointCloud &output);
        static void AdaptiveFilter(const PointCloud &input, int min_num, PointCloud &output);
    };

} // namespace unity
