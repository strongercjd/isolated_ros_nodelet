#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <vector>

namespace fastbuild_task_nodelet
{

// frontier 检测参数
struct FrontierParams
{
    int occ_free_th = 25;       // occ <= 阈值视为自由（保守，要求多次 free 证据）
    int occ_occ_th = 65;        // occ >= 阈值视为占用
    int pass_half_window = 2;   // 通行性检查窗口半径（格）；2 → 5x5 = 0.25 m @res 0.05
    int min_cluster = 3;        // 小于该格数的簇丢弃
    size_t max_clusters = 64;   // 候选目标上限（按簇大小截断）
};

struct FrontierGoal
{
    double wx = 0.0, wy = 0.0;  // 世界坐标（map 系，栅格中心）
    int col = 0, row = 0;       // 质心所在栅格（取整）
    size_t size = 0;            // 簇大小（候选排序用）
    uint8_t via = 0;            // 0=直线可达 1=绕行（pickAndSendGoal 决策时填，决策日志用）
};

namespace frontier_detail
{

inline bool isFree(int v, int free_th)
{
    return v >= 0 && v <= free_th;
}

inline bool isOccupied(int v, int occ_th)
{
    return v >= occ_th && v <= 100;
}

} // namespace frontier_detail

// 从占用栅格提取 frontier 质心候选：
//   frontier 格 = free 且 8 邻域存在 unknown（既非 free 也非 occupied）
//   通行性过滤：候选格 (2*half+1)^2 邻域内无 occupied
//   4 连通聚类，簇 >= min_cluster 取质心；按簇大小降序，截断 max_clusters
inline std::vector<FrontierGoal> detectFrontiers(const nav_msgs::OccupancyGrid &map, const FrontierParams &p)
{
    using namespace frontier_detail;
    std::vector<FrontierGoal> goals;
    const int w = static_cast<int>(map.info.width);
    const int h = static_cast<int>(map.info.height);
    if (w <= 2 || h <= 2 || map.data.size() != static_cast<size_t>(w) * h)
        return goals;

    const int8_t *d = map.data.data();
    const double res = map.info.resolution;

    // 标记通过通行性检查的 frontier 格
    std::vector<uint8_t> front(static_cast<size_t>(w) * h, 0);
    for (int r = 1; r < h - 1; ++r)
    {
        for (int c = 1; c < w - 1; ++c)
        {
            const int v = d[r * w + c];
            if (!isFree(v, p.occ_free_th))
                continue;
            bool touch_unknown = false;
            for (int dr = -1; dr <= 1 && !touch_unknown; ++dr)
                for (int dc = -1; dc <= 1; ++dc)
                {
                    if (dr == 0 && dc == 0)
                        continue;
                    const int nv = d[(r + dr) * w + (c + dc)];
                    if (!isFree(nv, p.occ_free_th) && !isOccupied(nv, p.occ_occ_th))
                    {
                        touch_unknown = true;
                        break;
                    }
                }
            if (!touch_unknown)
                continue;

            bool blocked = false; // 通行性：邻域窗口内无 occupied
            for (int dr = -p.pass_half_window; dr <= p.pass_half_window && !blocked; ++dr)
                for (int dc = -p.pass_half_window; dc <= p.pass_half_window; ++dc)
                {
                    const int rr = r + dr, cc = c + dc;
                    if (rr < 0 || cc < 0 || rr >= h || cc >= w)
                        continue;
                    if (isOccupied(d[rr * w + cc], p.occ_occ_th))
                    {
                        blocked = true;
                        break;
                    }
                }
            if (!blocked)
                front[static_cast<size_t>(r) * w + c] = 1;
        }
    }

    // 4 连通聚类（BFS），簇 >= min_cluster 取质心
    const int dr4[4] = {-1, 1, 0, 0};
    const int dc4[4] = {0, 0, -1, 1};
    std::queue<int> pending;
    for (int r = 1; r < h - 1; ++r)
    {
        for (int c = 1; c < w - 1; ++c)
        {
            const size_t idx = static_cast<size_t>(r) * w + c;
            if (front[idx] != 1)
                continue;
            // 收集整个簇
            long rsum = r, csum = c;
            size_t count = 1;
            front[idx] = 2; // 已入簇
            pending.push(static_cast<int>(idx));
            while (!pending.empty())
            {
                const int cur = pending.front();
                pending.pop();
                const int cr = cur / w, cc = cur % w;
                for (int k = 0; k < 4; ++k)
                {
                    const int nr = cr + dr4[k], nc = cc + dc4[k];
                    if (nr < 1 || nc < 1 || nr >= h - 1 || nc >= w - 1)
                        continue;
                    const size_t nidx = static_cast<size_t>(nr) * w + nc;
                    if (front[nidx] != 1)
                        continue;
                    front[nidx] = 2;
                    rsum += nr;
                    csum += nc;
                    ++count;
                    pending.push(static_cast<int>(nidx));
                }
            }
            if (count < static_cast<size_t>(p.min_cluster))
                continue;
            FrontierGoal g;
            g.row = static_cast<int>(rsum / static_cast<long>(count));
            g.col = static_cast<int>(csum / static_cast<long>(count));
            g.wx = map.info.origin.position.x + (g.col + 0.5) * res;
            g.wy = map.info.origin.position.y + (g.row + 0.5) * res;
            g.size = count;
            goals.push_back(g);
        }
    }

    std::sort(goals.begin(), goals.end(),
              [](const FrontierGoal &a, const FrontierGoal &b) { return a.size > b.size; });
    if (goals.size() > p.max_clusters)
        goals.resize(p.max_clusters);
    return goals;
}

// 已探索自由面积（m^2）：free 格计数 × 单格面积
inline double freeAreaM2(const nav_msgs::OccupancyGrid &map, int occ_free_th)
{
    if (map.info.resolution <= 0.0)
        return 0.0;
    size_t n = 0;
    for (const auto v : map.data)
    {
        const int iv = v;
        if (iv >= 0 && iv <= occ_free_th)
            ++n;
    }
    return static_cast<double>(n) * map.info.resolution * map.info.resolution;
}

/**
 * @brief 检查从 (x0,y0) 到 (x1,y1) 是否直线可达
 * @param map 地图
 * @param x0 起始点 x
 * @param y0 起始点 y
 * @param x1 目标点 x
 * @param y1 目标点 y
 * @param p 直线可达性参数
 * @return true 可达
 * @return false 不可达
 */
inline bool lineReachable(const nav_msgs::OccupancyGrid &map, double x0, double y0,
                          double x1, double y1, const FrontierParams &p)
{
    using namespace frontier_detail;
    const int w = static_cast<int>(map.info.width);
    const int h = static_cast<int>(map.info.height);
    const double res = map.info.resolution;
    if (w <= 2 || h <= 2 || res <= 0.0 || map.data.size() != static_cast<size_t>(w) * h)
        return true; // 地图异常时不拦截

    const double dx = x1 - x0, dy = y1 - y0;
    const double dist = std::hypot(dx, dy);
    const int steps = std::max(1, static_cast<int>(dist / (res * 0.5)));
    for (int s = 0; s <= steps; ++s)
    {
        const double t = static_cast<double>(s) / steps;
        const int c = static_cast<int>((x0 + dx * t - map.info.origin.position.x) / res);
        const int r = static_cast<int>((y0 + dy * t - map.info.origin.position.y) / res);
        for (int dr = -p.pass_half_window; dr <= p.pass_half_window; ++dr)
        {
            for (int dc = -p.pass_half_window; dc <= p.pass_half_window; ++dc)
            {
                const int rr = r + dr, cc = c + dc;
                if (rr < 0 || cc < 0 || rr >= h || cc >= w)
                    continue; // 越界 = 未观测，不拦
                if (isOccupied(map.data[static_cast<size_t>(rr) * w + cc], p.occ_occ_th))
                    return false;
            }
        }
    }
    return true;
}

} // namespace fastbuild_task_nodelet
