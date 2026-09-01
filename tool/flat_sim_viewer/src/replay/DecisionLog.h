// flat_sim_viewer —— fastbuild 决策日志解析（无 Qt、无 ROS，便于 headless 测试）
//
// 数据来源：app_runtime/data/log/custom_ros_nodelet.log（manager 把全部节点输出
// 重定向进的共享日志文件）。fastbuild_task_nodelet 每次决策用 NODELET_INFO 输出一行
//   FASTBUILD_DECISION {"seq":..,"sec":..,"nsec":..,"task_state":..,"area":..,
//                       "has_selected":..,"sel_x":..,"sel_y":..,
//                       "candidates":[{"x":..,"y":..,"size":..,"via":..},..],
//                       "blacklist":[{"x":..,"y":..},..]}
// 完整行形如 "[INFO] [09-01 ..] [/fastbuild_task] [file:line]: FASTBUILD_DECISION {...}".
// 解析只认标记后的 JSON；sec/nsec 是 ros::Time 绝对时间戳（匹配 slam bag 的基准），
// 前缀的 %m-%d %H:%M:%S 无年份且为本地时，不使用。
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace flat_sim_viewer {

// 单条决策记录（字段对齐 fastbuild 的 FASTBUILD_DECISION JSON）
struct DecisionRecord {
  struct Cand {
    float x = 0, y = 0;
    uint32_t size = 0;  // 簇大小
    uint8_t via = 0;    // 0=直线可达 1=绕行
  };
  uint64_t sec = 0;
  uint32_t nsec = 0;  // 决策时刻 ros::Time(sec, nsec)
  uint32_t seq = 0;
  uint8_t task_state = 0;  // 1=RUNNING 4=DONE
  float area_m2 = 0;
  bool has_selected = false;
  float sel_x = 0, sel_y = 0;
  std::vector<Cand> candidates;
  std::vector<std::pair<float, float>> blacklist;
};

// 解析日志中的 FASTBUILD_DECISION 行。progress(0..1) 定期回调（可空）。
// 损坏/缺 sec 的行跳过并计数（不致命）。返回 false 且 err 非空 = 文件打不开 / 无记录。
bool parseDecisionLog(const std::string& path,
                      const std::function<void(double)>& progress,
                      std::vector<DecisionRecord>& out,
                      std::string& err);

}  // namespace flat_sim_viewer
