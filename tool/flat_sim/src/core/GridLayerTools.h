// flat_sim —— 编辑墙格层与世界障碍的工具函数（栅格化 / 权威判断）
// 不依赖 ROS / GUI：core 层算法，SimRunner、SimView、测试共用。
#pragma once

#include "core/World.h"

namespace flat_sim {

// 权威占用判断：格层已初始化且 active（active 时碰撞/显示以格层为准）。
inline bool gridAuthoritative(const GridLayer* g) {
  return g && g->inited() && g->active;
}

// 把世界里的几何障碍（wall/box 矩形、circle 圆）栅格化进格层 g（置占用，不清空）。
// 要求 g 已 configure（GUI 进入编辑或运行时装入 legacy editwall 时调用）。
// 对每个格子用「矩形 vs 格子 SAT 相交」「圆心到格子最近点 ≤ 半径」精确判定，
// 薄墙/斜墙不会因只采样格心而漏掉。
void bakeWorldObstacles(const WorldDesc& w, GridLayer& g);

}  // namespace flat_sim
