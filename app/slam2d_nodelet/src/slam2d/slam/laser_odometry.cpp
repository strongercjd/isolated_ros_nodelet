#include "laser_odometry.h"
#include "icp2d.h"
#include "unity/voxel_filter.h"
#include "unity/tic_toc.h"

namespace slam
{
    /* 候选姿态 */
    struct CandidatePose
    {
        unity::Rotation rotation;
        unity::PointCloud cloud;

        CandidatePose(float angle)
            : rotation(angle)
        {
        }
        /**
         * @brief 将输入的向量 translate 添加到 cloud 中的每个点上，并将结果存储在 output 中。这在几何学中通常被称为平移变换
         *
         * @param translate 二维向量
         * @param output
         */
        void GenerateTransformedCloud(const Eigen::Vector2f &translate, unity::PointCloud &output) const
        {
            if (output.size() < cloud.size()) // 检查 output 的大小是否小于 cloud 的大小
                output.resize(cloud.size());  // 调整 output 的大小以匹配 cloud 的大小
            for (uint32_t idx = 0; idx < cloud.size(); idx++)
            {
                output[idx] = cloud[idx] + translate; // 将 translate 向量加到 cloud 中索引为 idx 的点上
            }
        }
    };

    /**
     * @brief 基于点云和给定的角度范围以及步长生成一系列可能的姿态或位置  Generate:生成 Candidate:候选 Poses:位姿
     *
     * 这个函数通过在给定的角度范围内进行均匀的旋转，为输入的点云数据生成了一系列的候选姿态。
     * 每一个候选姿态都包含了一个旋转后的点云，这些点云是通过对原始点云中的每个点应用旋转操作得到的。
     *
     * @param max_angular 最大角度
     * @param angular_step 角度步长
     * @param source 输入的点云数据
     * @param candidates 存储候选姿态的向量
     * @param base_angle 基础角度base_angle（默认为0.0）
     */
    void GenerateCandidatePoses(float max_angular, float angular_step, const unity::PointCloud &source,
                                std::vector<CandidatePose> &candidates, float base_angle = .0f)
    {
        candidates.clear();                                                      // 清空之前存储的候选姿态。
        candidates.reserve(static_cast<int>(2.0f * max_angular / angular_step)); // 预留足够的空间以避免在循环中频繁的内存分配。这是根据给定的最大角度和角度步长来计算需要的空间。

        // 遍历从-max_angular到max_angular的所有角度，每次增加angular_step
        for (float angle = -max_angular; angle <= max_angular; angle += angular_step)
        {
            unity::Rotation rot(base_angle + angle); // 创建一个旋转对象，该对象表示相对于基础角度增加angle的旋转。
            // CandidatePose(angle)获取该角度下的旋转方程
            // 候选必须存 base_angle+angle：点云按 base+angle 旋转，pose 输出用 candidate.rotation.angle()
            // 构造，只存 angle 会丢基准角（原工程从未传非 0 的 base_angle，未暴露）
            candidates.push_back(CandidatePose(base_angle + angle)); // 将一个新的候选姿态添加到向量中，这个姿态以当前的角度为参数创建。
            for (const unity::PointType &p : source)    // 遍历输入的点云数据。
            {
                /* v.back() 访问v的最后一个元素 因为上面
                candidates.push_back(CandidatePose(angle))
                刚刚放进去一个姿态,所以获取最新的姿态就好 */
                candidates.back().cloud.push_back(rot * p); // 对每个点应用旋转，并将结果添加到当前候选姿态的点云中。
            }
        }
    }
    /**
     * @brief 计算点云 cloud 中与给定点 p 的最近点的距离
     *
     * @param p 定点
     * @param cloud 点云
     * @return float 最近距离
     */
    static float NearestDistance(const unity::PointType &p, const unity::PointCloud &cloud)
    {
        float min_dist(1e9); // 初始化一个浮点数 min_dist，并设置其初值为 1e9，这是一个相对较大的数，通常用作无穷大的近似值。
        for (int idx(0); idx < cloud.size(); idx++)
        {
            const unity::PointType &t(cloud[idx]);
            const float dist((p - t).norm()); // 计算点 p 到点 t 的欧几里得距离（即两点之间的直线距离），并得到其长度
            if (min_dist <= dist)
                continue;
            min_dist = dist;
        }
        return min_dist;
    }
    /**
     * @brief 计算的是源点云中那些距离目标点云中最近点的距离小于或等于 max_distance 的点的平均距离
     * 也就是计算经过平移变换后的上一帧点云中每个点距离当前点云最小距离,然后求这些距离的平均值(距离必须小于max_distance,大于的直接舍弃)
     *
     * @param source 源点云,也就是上一帧点云经过了旋转,再经过平移后的点云
     * @param target 目标点云,也就是当前帧点云
     * @param max_distance 可接受的最大距离
     * @return float
     */
    static float CalculateDistance(
        const unity::PointCloud &source, const unity::PointCloud &target, float max_distance)
    {
        int count(0);        // 记录符合条件的点的数量
        float sum_dist(.0f); // 累计这些点的距离
        /* 经过这个for循环之后，count表示的是符合条件的点的数量，sum_dist表示的是符合条件的点的总距离， */
        for (const unity::PointType &s : source)
        {
            const float min_dist(NearestDistance(s, target)); // 返回 s 到 target(当前点云) 中最近点的距离,返回这个点和点云中每个点差值的最小值
            if (min_dist > max_distance)                      // 如果最近点的距离大于 max_distance，则跳过当前循环，处理下一个点
                continue;
            sum_dist += min_dist;
            count++;
        }
        if (count <= 0)
            return max_distance;
        return std::sqrt(sum_dist / static_cast<float>(count)); // 计算平均距离，std::sqrt（平方根函数）来确保结果是一个正数
    }

    static void TransformPointCloud(const unity::PointCloud &source, const unity::Rigid2f &pose,
                                    unity::PointCloud &result)
    {
        result.clear();
        result.reserve(source.size());
        for (const unity::PointType &p : source)
        {
            result.push_back(pose * p);
        }
    }
    /**
     * @brief Correlative 相关 Scan 扫描 Match 匹配.点云匹配，以确定它们之间的相对姿态（旋转和平移）
     * 函数通过生成一系列可能的姿态（称为“CandidatePoses”），然后对每个姿态应用一系列平移变换，以找到与目标点云最匹配的姿态，并将该姿态赋值给参考参数pose
     *
     * @param source 上一帧点云
     * @param target 当前点云
     * @param pose 计算得到的两帧点云之间的pose
     */
    // uint8_t flg_cjd = 0;
    void CorrelativeScanMatch(
        const unity::PointCloud &source, const unity::PointCloud &target, unity::Rigid2f &pose,
        const unity::Rigid2f &guess)
    {
        std::vector<CandidatePose> candidates;
        /* 以 guess（odom 帧间变换，与 CSM 输出同语义：把上一帧点映到当前帧系）
         * 为中心的小窗搜索（±10°，步长 2°）。
         * 原实现以零位姿为基准全范围搜索 ±30°（步长 5°）且不用 odom 初值：
         * 贴墙原地旋转时，错位候选只有少数点碰巧贴上墙，而评分只对"匹配上的
         * 点"取均值（未匹配点直接丢弃），错位候选反而得分更高——实测 90° 原地
         * 转向累计跟丢 ~74°、位置发散 3.9 m。 */
        GenerateCandidatePoses(10.0f / 57.3f, 2.0f / 57.3f, source, candidates,
                               guess.angle());

        const float max_linear(0.06f);  // 最大线性偏移（相对 guess 平移）
        const float linear_step(0.02f); // 线性偏移步长

        const float max_distance(0.5f); // 最大距离
        const Eigen::Vector2f base_t(guess.pose()); // 平移基准 = guess 平移

        float min_distance(1e6);             // 用于存储与目标点云之间的最小距离
        unity::PointCloud transformed_cloud; // 用于存储经过变换的源点云
        for (const CandidatePose &candidate : candidates)
        {
            for (float offsetX = -max_linear; offsetX <= max_linear; offsetX += linear_step)
            {
                for (float offsetY = -max_linear / 2.0f; offsetY <= max_linear / 2.0f; offsetY += linear_step)
                {
                    // 在 guess 平移基准上叠加候选偏移（原实现只搜零平移附近）
                    candidate.GenerateTransformedCloud(base_t + Eigen::Vector2f(offsetX, offsetY), transformed_cloud);
                    /* 取出平移后的点云中每个点距离当前点云每个点距离最小的值(小于max_distance)，然后取这些距离的平均数 */
                    float distance(CalculateDistance(transformed_cloud, target, max_distance));

                    if (min_distance <= distance) // 如果当前平移下计算的distance大于min_distance，则跳过当前循环。
                        continue;
                    pose = unity::Rigid2f(candidate.rotation.angle(), base_t(0) + offsetX, base_t(1) + offsetY); // 最匹配的相对姿态赋值给引用参数pose
                    min_distance = distance;                                             // 存储最短距离
                }
            }
        }
    }

    float GetResiualOfTransform(
        const unity::PointCloud &source, const unity::PointCloud &target, const unity::Rigid2f &trans)
    {
        unity::PointCloud transformed_cloud;
        TransformPointCloud(source, trans, transformed_cloud);
        float dist_sum(.0f);
        int dist_num(0);
        for (const unity::PointType &p : transformed_cloud)
        {
            const float dist(NearestDistance(p, target));
            if (dist > 0.1f)
                continue;
            dist_sum += dist;
            dist_num++;
        }
        if (dist_num <= 0)
            return 1e6;
        return dist_sum / static_cast<float>(dist_num);
    }
    /**
     * @brief
     *
     * @param prev_cloud 上一帧点云
     * @param curr_cloud 当前帧点云
     * @param odom 计算两帧点云之间的pose
     * @return unity::Rigid2f
     */
    unity::Rigid2f LaserOdometry::Process(
        const unity::PointCloud &prev_cloud,
        const unity::PointCloud &curr_cloud, const unity::Rigid2f &odom)
    {
        unity::TicToc tt;
        // CSM
        // 以 odom 为兜底初值：CSM 全候选都超 max_distance 时至少不会引入错误跳变
        unity::Rigid2f initial_delta(odom.angle(), odom.pose().x(), odom.pose().y());
        tt.tic();
        /* 点云粗匹配后得到了两帧点云之间的pose */
        CorrelativeScanMatch(prev_cloud, curr_cloud, initial_delta, odom); // 点云粗匹配（以 odom 为中心的小窗搜索）
        // std::cout<<"csm\t"<<tt.toc()<<std::endl;

        // unity::Rigid2f laser_delta(initial_delta);
        Icp2D::Icp2DParams icp_params(initial_delta); // 通过粗匹配计算的pose转给icp参数
        icp_params.max_match_distance = 0.05f;        // 寻找最近点时可以接受的最大距离
        unity::Rigid2f laser_delta;
        tt.tic();
        if (!Icp2D::AligenPointCloud(prev_cloud, curr_cloud, icp_params, laser_delta))//经过此步骤，得到了两帧点云之间的pose也就是laser_delta
        {
            std::cout << "Warning: L-L icp failed" << std::endl;
            laser_delta = initial_delta;//匹配失败了，就用粗匹配的pose
        }
        else
        {
            // const float laser_res(GetResiualOfTransform(curr_cloud, prev_cloud, laser_delta));
            // const float odom_res(GetResiualOfTransform(curr_cloud, prev_cloud, odom));
            // if (laser_res > odom_res)
            //     laser_delta = odom;
        }

        // std::cout<<"icp use time\t"<<tt.toc()<<std::endl;
        return laser_delta.inverse();
    }

} // namespace slam
