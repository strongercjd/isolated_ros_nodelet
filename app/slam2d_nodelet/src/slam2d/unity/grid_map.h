#pragma once

#include "unity/point_cloud.h"

namespace unity
{

/*  Y
 *  | 0  1  2  3  4  5  6  7  8  9
 *  | 10 11 12 13 14 15 16 17 18 19
 *  |
 *  |
 *  |
 *  |
 *  |
 *  |
 *  |
 *  |-------------------------------> X
 */
class GridMap
{
public:
    GridMap(int w, int h, const Eigen::Vector2f& origin_pos = Eigen::Vector2f(0, 0), float resolution = 0.05f);
    GridMap(const GridMap& right);

    Eigen::Vector2f GetCellCenter(const Eigen::Array2i& index) const;
    Eigen::Array2i GetCellIndex(const Eigen::Vector2f& pos) const;

    int8_t& value(const Eigen::Array2i& index);
    int8_t& value(const Eigen::Vector2f& pos);
    int8_t value(const Eigen::Array2i& index) const;
    int8_t value(const Eigen::Vector2f& pos) const;

    void GetSurroundCloud(const Eigen::Vector2f& pos, 
        float distance, int8_t max_value, unity::PointCloud& cloud) const;

    bool IndexValid(const Eigen::Array2i& index) const;

    int width() const
    {
        return width_;
    }

    int height() const
    {
        return height_;
    }
    
    float resolution() const
    {
        return resolution_;//地图的分辨率，也就是每个格子占据的大小
    }
    
    const std::vector<int8_t>& data() const
    {
        return data_;
    }

    static int8_t min_value()
    {
        return -127;
    }

    static int8_t max_value()
    {
        return 127;
    }
private:
    const int width_;//地图的宽度
    const int height_;//地图的高度
    const Eigen::Vector2f origin_pos_;//原点，这个单位已经乘以地图分辨率了
    const float resolution_;//地图的分辨率，也就是每个格子占据的大小
    std::vector<int8_t> data_;//地图数据
    int8_t invalid_pos_data_;
};

}