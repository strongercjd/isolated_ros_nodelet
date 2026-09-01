// flat_sim_viewer —— rosbag 回放引擎（无 Qt、无线程、无 Q_OBJECT）
//
// 轮询式：UI 线程每 tick 调 advance()，与 SlamListener 的 snapshot() 拉取模式一致。
// bag 内话题：/slam2d/map /slam2d/pose /slam2d/input_cloud /slam2d/mapping_cloud
//（map_log_nodelet 录制；也兼容手动 rosbag record 的同名话题）。
//
// 帧 = pose 消息：slam2d 每 scan 帧发 pose+双云（同 stamp），map 1Hz 是背景刷新，
// 以 pose 为帧对齐"一次视觉变化"。
//
// 时间轴与索引：
//   open() 全量扫描一次建事件索引（只记 time+topic，不反序列化消息体）；
//   播放走持久 View 迭代器顺序推进（同 chunk 不重解压）；
//   回退/大跳用检查点（每 ~60 事件存一份快照）恢复后增量重放。
// 所有 bag 访问都在调用线程（UI 线程）——rosbag::Bag 非线程安全。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "ros/SlamSnapshot.h"

namespace flat_sim_viewer {

class BagPlayer {
 public:
  BagPlayer() = default;
  ~BagPlayer();
  BagPlayer(const BagPlayer&) = delete;
  BagPlayer& operator=(const BagPlayer&) = delete;

  // 打开并建索引。progress(0..1) 定期回调（加载大文件时刷进度用），可空。
  // 成功后定位在第一帧（已应用、暂停状态）。
  bool open(const std::string& path,
            const std::function<void(double)>& progress = nullptr);
  void close();
  bool isOpen() const { return bag_.isOpen(); }
  const std::string& error() const { return err_; }
  const std::string& fileName() const { return path_; }

  // ---- 时间轴 ----
  bool empty() const { return events_.empty(); }
  ros::Time bagStart() const;  // 首事件时间；空 bag 返回 ros::Time(0)
  ros::Time bagEnd() const;    // 末事件时间
  double durationSec() const;
  size_t frameCount() const { return poseFrames_.size(); }  // pose 帧数
  size_t frameIndex() const;  // 当前帧序号（0 起；无 pose 时 0）
  ros::Time playhead() const { return head_; }

  // ---- 播放控制 ----
  void play();           // 已到末尾则从头重播
  void pause() { playing_ = false; }
  bool isPlaying() const { return playing_; }
  bool atEnd() const { return idx_ >= events_.size(); }
  void setSpeed(double s) { speed_ = s; }
  double speed() const { return speed_; }

  // 播放中每 tick 调：吸收 (head_, head_ + dt*speed] 窗口内的事件；到尾自动暂停
  void advance(double wallDtSec);
  // 单步 = 下/上一 pose 帧（隐含暂停）；已在端点则不动
  void stepForward();
  void stepBackward();
  // 拖动进度条：应用到 time <= t 的所有事件（保持当前播放状态）
  void seekToTime(const ros::Time& t);

  // 当前画面（引用，调用方立即消费或拷贝）
  const SlamSnapshot& snapshot() const { return snap_; }

 private:
  struct Event {
    ros::Time t;
    uint8_t topic;  // 0=map 1=pose 2=input 3=mapping
  };

  void applyEvent(const rosbag::MessageInstance& m);  // instantiate → SnapshotBuilder
  void applyTo(size_t targetIdx);                     // 前进：应用 events_[idx_, targetIdx)
  void rebuildTo(size_t targetIdx);                   // 回退：检查点恢复 + 增量应用
  void resetIterTo(size_t k);                         // 重建持久迭代器并快进到 events_[k)
  size_t curFrameSlot() const;  // poseFrames_ 中最后已应用 pose 的位置（无则 size()）

  rosbag::Bag bag_;
  std::string path_, err_;
  std::vector<std::string> topics_;  // 4 个目标话题

  std::vector<Event> events_;       // 全时间线（View 迭代序，即时间序）
  std::vector<size_t> poseFrames_;  // pose 事件在 events_ 中的下标
  std::vector<std::pair<size_t, SlamSnapshot>> cps_;  // 检查点 <idx, 应用后快照>
  size_t nextCpAt_ = 0;             // 下一个检查点的事件下标

  // 持久顺序迭代器（播放快进不重扫历史）
  std::unique_ptr<rosbag::View> liveView_;
  rosbag::View::iterator liveIt_;

  SlamSnapshot snap_;
  uint32_t mapSeqOut_ = 0;  // seq 高水位：跨回退仍单调递增（SlamView 据此重建 QImage）
  size_t idx_ = 0;          // 已应用到 events_[idx_)
  ros::Time head_;          // 当前时间轴位置 = events_[idx_-1].t（idx_==0 为首事件时间）
  bool playing_ = false;
  double speed_ = 1.0;

  static constexpr size_t kCpInterval = 60;  // 检查点间隔（≈2s bag 时间）
};

}  // namespace flat_sim_viewer
