// flat_sim —— .fworld / .frobot 加载器
//
// 语义（详见 tool/flat_sim/README.md 与 tool/新需求.md 第 5、6 节）：
//   单位：长度米、角度度；备注 # 在 TextFormat 层已被丢弃。
//   .fworld：world / wall / box / circle / robot / robot_file 顶层块（可多个）
//   .frobot：单个 robot: 块
//   robot_file 路径相对 .fworld 所在目录解析。
#pragma once

#include <string>

#include "core/World.h"

namespace flat_sim {

// 解析 .fworld；失败抛 format::FormatError（文件:行号: 原因）
WorldDesc loadWorld(const std::string& path);

// 解析 .frobot（单个 robot: 块）
RobotDesc loadRobotFile(const std::string& path);

}  // namespace flat_sim
