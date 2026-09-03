// flat_sim —— .fworld 序列化（与 WorldLoader 读的是同一种自研格式，可往返）
//
// 用于 GUI「另存为新 .fworld」：把 原世界几何 + 编辑墙层(editwall) + 测量标注
// (annotation) 写成一个能被 loadWorld 重新读回的完整世界文件。
#pragma once

#include <string>

#include "core/World.h"

namespace flat_sim {

// 序列化为 .fworld 文本；失败（罕见）返回非空错误信息
std::string worldToText(const WorldDesc& w);

// 写入文件；成功返回空串，失败返回错误信息
std::string saveWorldFile(const WorldDesc& w, const std::string& path);

}  // namespace flat_sim
