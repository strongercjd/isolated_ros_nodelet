// flat_sim_viewer —— fastbuild 决策回放控制器（slam 地图背景 + 决策记录步进）
//
// 两条独立数据源按时间戳匹配：
//   · 决策日志（custom_ros_nodelet.log）→ records_（决策记录 = 步进帧）
//   · slam bag（map_log_<ts>.bag）→ BagPlayer（地图背景；点云/位姿由 SlamView 抑制）
// 步进 = 上/下一条决策记录；播放走 slam bag 时间轴，决策覆盖层在记录边界切换。
// 轮询式：UI 线程每 tick 调 advance()（与 BagPlayer 同模式）。无 Qt。
#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <ros/time.h>

#include "replay/BagPlayer.h"
#include "replay/DecisionLog.h"
#include "ros/SlamSnapshot.h"

namespace flat_sim_viewer {

class FastbuildPlayer {
 public:
  FastbuildPlayer() = default;
  ~FastbuildPlayer() = default;
  FastbuildPlayer(const FastbuildPlayer&) = delete;
  FastbuildPlayer& operator=(const FastbuildPlayer&) = delete;

  // 决策日志 → records_；slam bag → 地图背景。成功后定位到第一条记录。
  bool openDecisionLog(const std::string& path,
                       const std::function<void(double)>& progress = nullptr);
  bool openSlamBag(const std::string& path,
                   const std::function<void(double)>& progress = nullptr);
  void close();

  bool ready() const { return !records_.empty() && slam_.isOpen(); }
  bool decisionLoaded() const { return !records_.empty(); }
  const std::string& slamFileName() const { return slam_.fileName(); }
  const std::string& decisionFileName() const { return decision_path_; }
  const std::string& error() const { return err_; }

  // ---- 时间轴（slam bag）----
  bool empty() const { return records_.empty(); }
  size_t recordCount() const { return records_.size(); }
  size_t recordIndex() const { return idx_; }
  ros::Time bagStart() const { return slam_.bagStart(); }
  ros::Time bagEnd() const { return slam_.bagEnd(); }
  double durationSec() const { return slam_.durationSec(); }
  ros::Time playhead() const { return head_; }

  // ---- 播放控制 ----
  void play();
  void pause() { playing_ = false; slam_.pause(); }
  bool isPlaying() const { return playing_; }
  void setSpeed(double s) { speed_ = s; slam_.setSpeed(s); }
  double speed() const { return speed_; }
  void advance(double wallDtSec);  // 播放推进：overlay 切到 ≤playhead 的最后一条记录
  void stepForward();              // 上/下一条决策记录（隐含暂停）
  void stepBackward();
  void seekToTime(const ros::Time& t);  // 拖动：overlay = 该时刻对应的最近记录
  void seekToRecord(size_t i);

  // 当前画面（slam 地图 + 当前决策覆盖层；点云/位姿仍在，由视图按模式抑制）
  const SlamSnapshot& snapshot() const { return snap_; }

 private:
  // slam seek 到 t + 重算 overlay（最近记录 ≤ t）
  void rebuildTo(const ros::Time& t);
  void applyRecord(const DecisionRecord& r);  // 填 snap_.decision
  size_t recordAtOrBefore(const ros::Time& t) const;
  // 首条记录时刻，但 clamp 到 slam bag 起点：首条决策（如 CMD_START 任务开始标记）
  // 可能早于 bag 首个 slam 事件，seek 到 bag 之前会走 BagPlayer 回退路径清空地图快照
  ros::Time firstRecordTime() const;

  BagPlayer slam_;
  std::vector<DecisionRecord> records_;
  size_t idx_ = 0;         // 当前 overlay 记录下标（records_.size() = 无匹配）
  ros::Time head_;         // 当前时间轴位置（slam 系）
  bool playing_ = false;
  double speed_ = 1.0;
  std::string err_;
  std::string decision_path_;
  SlamSnapshot snap_;      // 组装快照：slam 状态 + 决策覆盖层
};

}  // namespace flat_sim_viewer
