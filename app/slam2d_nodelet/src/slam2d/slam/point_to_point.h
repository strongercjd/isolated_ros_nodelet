#pragma once

#include "unity/transform.h"
#include "unity/point_cloud.h"

namespace slam
{

    class PointToPoint
    {
    public:
        PointToPoint(int max_iter_times);//初始化最大迭代次数
        void AddPair(const unity::PointType &source, const unity::PointType &target);//这个方法允许用户添加一对点，即源点（source）和目标点（target）。这对点将被用于配准过程。
        bool Solve(unity::Rigid2f &result);//匹配过程

    private:
        const int max_iter_times_;//最大迭代次数
        std::vector<std::pair<unity::PointType, unity::PointType>> pairs_;//存储点的对（pair）的向量（vector）。每一个pair包含一个源点和一个目标点。
    };

}
