#include "laser_mapping.h"
#include <set>

namespace slam
{

    const int dt_free_value(4);
    const int dt_free_after_value(4);
    const int dt_occ_value(8);

    /**
     * @brief 将整数数组转换为 uint64_t 类型的值
     *
     * @param p 点坐标
     * @return uint64_t
     */
    uint64_t Array2iToCode(const Eigen::Array2i &p)
    {
        union Int2ToU64
        {
            uint64_t u64;
            int int2[2];
        };

        Int2ToU64 coverter;
        coverter.int2[0] = p(0);
        coverter.int2[1] = p(1);
        return coverter.u64;
    }

    void swap_int(int *a, int *b)
    {
        *a ^= *b;
        *b ^= *a;
        *a ^= *b;
    }

    /**
     * @brief 增加一个特定位置（由x和y坐标确定）在grid_map地图上的值
     *
     * @param x 坐标x
     * @param y 坐标y
     * @param grid_map 地图
     * @param inc 指定增加值的数量
     */

    void IncreaseValue(int x, int y, unity::GridMap &grid_map, int8_t inc)
    {
        int8_t &value(grid_map.value(Eigen::Array2i(x, y))); // 取出对应坐标下map中的数据
        if (value < unity::GridMap::min_value() + 10)        // 检查当前单元格的值是否小于地图的最小值加上10
            value += std::max(1, inc / 2);                   // 增加的数量是inc的一半（向下取整）和1中的较大值。这样做是为了确保增加的值至少为1
        else if (value <= unity::GridMap::max_value() - inc) // 当前单元格的值小于或等于地图的最大值减去inc
            value += inc;                                    // 直接增加inc的数量到当前单元格的值
        else
            value = unity::GridMap::max_value(); // 当前单元格的值设置为地图的最大值
    }

    /**
     * @brief 在特定的坐标位置，减去制定的值
     *
     * @param x
     * @param y
     * @param grid_map
     * @param inc
     */
    void DecreaseValue(int x, int y, unity::GridMap &grid_map, int8_t inc)
    {
        int8_t &value(grid_map.value(Eigen::Array2i(x, y)));
        if (value > unity::GridMap::max_value() - 10)
            value -= std::max(1, inc / 2);
        else if (value >= unity::GridMap::min_value() + inc)
            value -= inc;
        else
            value = unity::GridMap::min_value();
    }

    void RadioLaser(int x1, int y1, int x2, int y2, unity::GridMap &grid_map)
    {
        int dx = std::abs(x2 - x1),
            dy = std::abs(y2 - y1),
            yy = 0;

        if (dx < dy)
        {
            yy = 1;
            swap_int(&x1, &y1);
            swap_int(&x2, &y2);
            swap_int(&dx, &dy);
        }

        int ix = (x2 - x1) > 0 ? 1 : -1,
            iy = (y2 - y1) > 0 ? 1 : -1,
            cx = x1,
            cy = y1,
            n2dy = dy * 2,
            n2dydx = (dy - dx) * 2,
            d = dy * 2 - dx;

        if (yy)
        {
            while (cx != x2)
            {
                if (d < 0)
                    d += n2dy;
                else
                {
                    cy += iy;
                    d += n2dydx;
                }
                IncreaseValue(cy, cx, grid_map, 2);
                cx += ix;
            }
        }
        else
        {
            while (cx != x2)
            {
                if (d < 0)
                    d += n2dy;
                else
                {
                    cy += iy;
                    d += n2dydx;
                }
                IncreaseValue(cx, cy, grid_map, 2);
                cx += ix;
            }
        }
    }

    /**
     * @brief 给定的两个点p0和p之间进行激光雷达扫描，并对网格地图grid_map进行更新
     *
     * @param p0 原点(在map中的索引)
     * @param p  点云中的点(在map中的索引)
     * @param grid_map 地图
     * @param occ_points 点云中所有点在地图中的索引
     */
    void RadioLaser(const Eigen::Array2i &p0, const Eigen::Array2i &p, unity::GridMap &grid_map, std::set<uint64_t> &occ_points)
    {
        if (p0(0) == p(0) && p0(1) == p(1))
            return;

        const Eigen::Array2i &delta_p(p - p0);            // p到p0的向量
        if (std::abs(delta_p(0)) >= std::abs(delta_p(1))) // x-major delta_p的x分量绝对值大于或等于y分量绝对值（即，扫描线斜率小于或等于1），则进入x-major模式
        {
            const float k(static_cast<float>(delta_p(1)) / static_cast<float>(delta_p(0))); // 计算斜率
            const int delta_x(delta_p(0) > 0 ? 1 : -1);                                     // delta_x增量 ，增量为正，delta_x为1
            for (int x(p0(0)); x != (p(0)); x += delta_x)                                   // 在X轴遍历
            {
                const float y(static_cast<float>(x - p0(0)) * k + p0(1));                   // 计算每个点对应的值
                if (occ_points.count(Array2iToCode(Eigen::Array2i(x, std::round(y)))) != 0) // 检查这个点是是否被占用，这里理解为：过滤重叠的点
                    return;
                // std::round 四舍五入
                IncreaseValue(x, std::round(y), grid_map, dt_free_value); // 在地图中增加它的值
            }

            /* 对于从p(0) + delta_x到p(0) + 4 * delta_x的每个x值，执行类似的操作，但在这里它会检查地图上该点的当前值是否小于-120。
            如果是，则增加该点的自由值并退出循环。否则，它会适当地增加或减少该点的值（根据它是正还是负）。 */
            for (int x(p(0) + delta_x); x != (p(0) + 4 * delta_x); x += delta_x)
            {
                const float y(static_cast<float>(x - p0(0)) * k + p0(1));        // 计算y值
                int8_t &value(grid_map.value(Eigen::Array2i(x, std::round(y)))); // 获取当前点在map中的值
                if (value < -120)
                {
                    IncreaseValue(x, std::round(y), grid_map, dt_free_after_value); // 直接在地图中增加它的值
                    break;
                }
                const int8_t dt_value(std::min(dt_free_after_value, std::abs(value))); // 选择一个最小的
                if (value > 0)
                    DecreaseValue(x, std::round(y), grid_map, dt_value); // 减去特定的值
                else
                    IncreaseValue(x, std::round(y), grid_map, dt_value); // 直接在地图中增加它的值
            }
        }
        else // y-major
        {
            const float k(static_cast<float>(delta_p(0)) / static_cast<float>(delta_p(1)));
            const int delta_y(delta_p(1) > 0 ? 1 : -1);
            for (int y(p0(1)); y != p(1); y += delta_y)
            {
                const float x(static_cast<float>(y - p0(1)) * k + p0(0));
                if (occ_points.count(Array2iToCode(Eigen::Array2i(std::round(x), y))) != 0)
                    return;
                IncreaseValue(std::round(x), y, grid_map, dt_free_value);
            }
            for (int y(p(1) + delta_y); y != (p(1) + 4 * delta_y); y += delta_y)
            {
                const float x(static_cast<float>(y - p0(1)) * k + p0(0));
                int8_t &value(grid_map.value(Eigen::Array2i(std::round(x), y)));
                if (value < -120)
                {
                    IncreaseValue(std::round(x), y, grid_map, dt_free_after_value);
                    break;
                }
                const int8_t dt_value(std::min(dt_free_after_value, std::abs(value)));
                if (value > 0)
                    DecreaseValue(std::round(x), y, grid_map, dt_value);
                else
                    IncreaseValue(std::round(x), y, grid_map, dt_value);
            }
        }
    }

    /**
     * @brief 对输入的点云数据进行处理，并映射到一个网格地图(grid_map)上
     *
     * @param grid_map
     * @param aligned_cloud
     * @param pose
     */
    void LaserMapping::DoMapping(unity::GridMap &grid_map, const unity::PointCloud &aligned_cloud, const unity::Rigid2f &pose)
    {
        std::set<uint64_t> occ_codes;                                          // 用于存储点云在map中的索引
        const Eigen::Array2i origin_index(grid_map.GetCellIndex(pose.pose())); // 获取原点在地图中的索引
        unity::PointCloud cloud;
        cloud.reserve(aligned_cloud.size());
        for (const unity::PointType &p : aligned_cloud)
        {
            /* 对输入的点云数据(aligned_cloud)进行遍历，对于每一个点，计算它到当前位姿(pose)的距离。如果距离小于0.2f或者大于30.0f，则跳过该点，否则将其添加到新的点云数据(cloud)中。 */
            const float distance((p - pose.pose()).norm());
            if (distance < 0.2f || distance > 30.0f)
                continue;
            cloud.push_back(p);
        }
        for (const unity::PointType &p : cloud)
        {
            /* 历新的点云数据(cloud)，对于其中的每个点，计算其在网格地图中的索引，并将对应的占用情况代码插入到occ_codes集合中 */
            const Eigen::Array2i index(grid_map.GetCellIndex(p));
            occ_codes.insert(Array2iToCode(index));
        }
        for (const unity::PointType &p : cloud)
        {
            const Eigen::Array2i index(grid_map.GetCellIndex(p));      // 地图中的索引
            RadioLaser(origin_index, index, grid_map, occ_codes);      // 更新原点到index地图
            //ISSUE：为什么要再次减去dt_occ_value呢？
            DecreaseValue(index(0), index(1), grid_map, dt_occ_value); // 减去特定的值
        }
    }

} // namespace slam
