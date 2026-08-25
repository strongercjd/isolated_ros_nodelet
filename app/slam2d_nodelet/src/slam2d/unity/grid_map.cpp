#include "grid_map.h"
#include <iostream>

namespace unity
{

    GridMap::GridMap(int w, int h, const Eigen::Vector2f &origin_pos, float resolution)
        : width_(w), height_(h),
          origin_pos_(-0.5f * width_ * resolution + origin_pos(0),
                      -0.5f * height_ * resolution + origin_pos(1)),
          resolution_(resolution), data_(width_ * height_, 0)
    {
    }

    GridMap::GridMap(const GridMap &right)
        : width_(right.width_), height_(right.height_),
          origin_pos_(right.origin_pos_), resolution_(right.resolution_),
          data_(right.data_)
    {
    }

    /**
     * @brief 获取指定索引的网格单元中心点坐标
     * 
     * @param index 网格单元的索引
     * @return Eigen::Vector2f 网格单元中心点的坐标
     */
    Eigen::Vector2f GridMap::GetCellCenter(const Eigen::Array2i &index) const
    {
        return Eigen::Vector2f(
            (static_cast<float>(index(0)) + 0.5f) * resolution_ + origin_pos_(0),
            (static_cast<float>(height_ - index(1)) - 0.5f) * resolution_ + origin_pos_(1));
    }

    /**
     * @brief 根据给定的2D位置（在Eigen::Vector2f类型中）返回相应的单元格索引
     * 
     * @param pos 输入的位置
     * @return Eigen::Array2i 
     */
    Eigen::Array2i GridMap::GetCellIndex(const Eigen::Vector2f &pos) const
    {
        return Eigen::Array2i(
            static_cast<int>((pos(0) - origin_pos_(0)) / resolution_),//X轴的索引
            //ISSUE：height_减去有可能是将坐标系原点从左上角转移到左下角
            static_cast<int>(static_cast<float>(height_) - (pos(1) - origin_pos_(1)) / resolution_));//Y轴的索引
    }

    /**
     * @brief 从给定的位置开始，获取其周围一定范围内的点云数据
     * 
     * @param pos 2D向量，表示在GridMap中的位置
     * @param distance 想要获取的周围点的最大距离
     * @param max_value 地图中某个点的值的最大阈值。如果点的值大于此阈值，该点将不会被添加到点云中。
     * @param cloud PointCloud对象，它将被填充从给定位置获取的周围点。
     */
    void GridMap::GetSurroundCloud(const Eigen::Vector2f &pos,
                                   float distance, int8_t max_value, unity::PointCloud &cloud) const
    {
        const Eigen::Vector2f lt_pos(pos(0) - distance, pos(1) + distance);//left_top_pos 需要获取点云范围的左上角坐标
        const Eigen::Vector2f rb_pos(pos(0) + distance, pos(1) - distance);//right_bottom_pos  需要获取点云范围的右上角坐标
        const Eigen::Array2i lt_index(GetCellIndex(lt_pos));// 左上角在地图中的索引
        const Eigen::Array2i rb_index(GetCellIndex(rb_pos));// 右下角在地图中的索引
        cloud.clear();
        for (int x(std::max(0, lt_index(0))); x < std::min(width_, rb_index(0)); x++)
            for (int y(std::max(0, lt_index(1))); y < std::min(height_, rb_index(1)); y++)
            {
                const Eigen::Array2i index(x, y);
                if (value(index) > max_value)
                    continue;
                cloud.push_back(GetCellCenter(index));//转换成在地图中的坐标
            }
    }

    int8_t &GridMap::value(const Eigen::Array2i &index)
    {
        if (index(0) < 0 || index(0) >= width_ || index(1) < 0 || index(1) >= height_)
        {
            invalid_pos_data_ = 0;
            return invalid_pos_data_;
        }
        return data_[index(1) * width_ + index(0)];
    }

    int8_t &GridMap::value(const Eigen::Vector2f &pos)
    {
        return value(GetCellIndex(pos));
    }

    /**
     * @brief 返回对应坐标下地图中的值
     * 
     * @param index 对应点的坐标
     * @return int8_t 
     */
    int8_t GridMap::value(const Eigen::Array2i &index) const
    {
        if (index(0) < 0 || index(0) >= width_ || index(1) < 0 || index(1) >= height_)
        {
            return 0;//不在地图范围内，返回0
        }
        return data_[index(1) * width_ + index(0)];
    }

    int8_t GridMap::value(const Eigen::Vector2f &pos) const
    {
        return value(GetCellIndex(pos));
    }

    bool GridMap::IndexValid(const Eigen::Array2i &index) const
    {
        if (index(0) < 0 || index(0) >= width_ || index(1) < 0 || index(1) >= height_)
        {
            return false;
        }
        return true;
    }

}