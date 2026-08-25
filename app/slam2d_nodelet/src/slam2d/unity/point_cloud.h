#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

namespace unity
{
    /*
    Vector2f是一种特殊的2维浮点向量类型
    Vector2f表示一个具有两个浮点数的向量。这两个浮点数可以看作是x和y坐标，形成一个二维向量。在Eigen库中，这个二维向量通常用于表示2D空间中的点或者方向
    声明和初始化
    Eigen::Vector2f v; // 声明一个Vector2f对象
    v << 1, 2; // 初始化x和y坐标
    设置和获取坐标
    float x = v.x(); // 获取x坐标
    float y = v.y(); // 获取y坐标

    v.setX(3); // 设置x坐标
    v.setY(4); // 设置y坐标
    */
    typedef Eigen::Vector2f PointType;
    typedef std::vector<PointType> PointCloud;

};