#include "voxel_filter.h"
#include <unordered_set>
#include <iostream>

namespace unity
{
    /**
     * @brief 这个函数实现了一个简单的VoxelGrid滤波器。
     * 它将输入的点云中的每个点分配到一个网格中，并只保留那些其网格中没有其他点的点的数据。
     * 这样可以有效地减少数据点的数量
     * 
     * 这里计算会除以 grid_size，所以grid_size必须大于0。grid_size数值越大，那么输出的点云数量就会减少
     *
     * @param input 输入点云
     * @param grid_size 表示网格大小
     * @param output 输入点云
     */
    void VoxelFilter::Filter(const PointCloud &input, float grid_size, PointCloud &output)
    {
        if (grid_size <= 0.0f)
            return;

        if (output.capacity() < input.size())
            output.reserve(input.size());
        output.clear();
        /*
        std::unordered_set: 这是C++标准库中的一个容器，它是一个无序集合，也就是说它不保证元素的排序。
        unordered_set 通常用于存储唯一的元素，它的主要特性是元素在集合中的顺序是不确定的，但每个元素只能出现一次。
        */
        std::unordered_set<int> hashtab;
        for (const PointType &pi : input)
        {
            const int xi(static_cast<int>(pi(0) / grid_size) + 50000);
            const int yi(static_cast<int>(pi(1) / grid_size) + 50000);
            const int value((xi << 16) | yi); // 使用了位运算，将点云的坐标转换为整数。
            if (hashtab.find(value) != hashtab.end())
                continue;
            output.push_back(pi);
            hashtab.insert(value);
        }
    }
    /**
     * @brief 自适应滤波器 经过滤波后的点云数量大于等于min_num。
     * 这种自适应过滤方法在处理大规模点云数据时非常有用，它可以有效地减少数据点的数量，同时尽可能地保留数据点的信息。
     *
     * @param input 输入点云
     * @param min_num 最小点云个数
     * @param output 输出点云
     * @note  这个函数有个缺陷，如果点数小于min_num，函数没有返回错误
     */
    void VoxelFilter::AdaptiveFilter(const PointCloud &input, int min_num, PointCloud &output)
    {
        if (input.size() <= min_num) // 点云个数小于min_num，直接返回原点云
        {
            output = input;
            return;
        }

        const float max_grid_size = 1.0f; // 网格大小

        VoxelFilter::Filter(input, max_grid_size, output); // 过滤重复的点,网格滤波器

        // std::cout << "First Filter:" << input.size() << "->" << output.size() << "\n";

        if (output.size() >= min_num)
            return;
        /*输出点云的数量仍然小于min_num，那么执行自适应调整过程*/
        PointCloud candidate; // candidate:求职候选人,这里表示候选点云
        for (float high_length = max_grid_size;
             high_length > 0.01 * max_grid_size; high_length /= 2.0f)
        {
            float low_length = high_length / 2.0f;
            VoxelFilter::Filter(input, low_length, output); // 过滤重复的点,降低网格大小,输出更多的有用的点

            if (output.size() < min_num)
                continue;
            /*自适应调整过程是一个二分查找过程，它以max_grid_size为初始调整步长，不断将步长减半，直到步长小于0.01 * max_grid_size。
            找到点数大于min_num的点*/
            // std::cout<<"high_length:" << high_length << " low_length:" << low_length << "\n";
            /* 下面是找到最合适的栅格大小，点云个数大于等于min_num 假设high_length是0.25,那么最合适的栅格大小是0.125到0.25之间*/
            while ((high_length - low_length) / low_length > 0.1f)//0.1表示二分法最多进行到low_length*0.1
            {
                const float mid_length = (low_length + high_length) / 2.0f;
                VoxelFilter::Filter(input, mid_length, candidate);
                if (candidate.size() >= min_num)
                {
                    low_length = mid_length;
                    output = candidate;
                }
                else
                    high_length = mid_length;
            }
            return;
        }
    }

}