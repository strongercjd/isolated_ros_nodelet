// flat_sim_viewer —— BagPlayer 实现
#include "replay/BagPlayer.h"

#include <algorithm>
#include <stdexcept>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <rosbag/message_instance.h>
#include <rosbag/query.h>

#include "ros/SnapshotBuilder.h"

namespace flat_sim_viewer {

namespace {

constexpr const char* kMapTopic = "/slam2d/map";
constexpr const char* kPoseTopic = "/slam2d/pose";
constexpr const char* kInputTopic = "/slam2d/input_cloud";
constexpr const char* kMappingTopic = "/slam2d/mapping_cloud";

// 事件 topic 编码（与 Event::topic 对应）
uint8_t topicId(const std::string& topic) {
  if (topic == kMapTopic) return 0;
  if (topic == kPoseTopic) return 1;
  if (topic == kInputTopic) return 2;
  if (topic == kMappingTopic) return 3;
  return 255;
}

}  // namespace

BagPlayer::~BagPlayer() { close(); }

// ---------------------------------------------------------------- 打开 / 关闭

bool BagPlayer::open(const std::string& path,
                     const std::function<void(double)>& progress) {
  close();
  try {
    bag_.open(path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    err_ = e.what();
    return false;
  }
  path_ = path;
  topics_ = {kMapTopic, kPoseTopic, kInputTopic, kMappingTopic};

  // 全量建索引：只记 (time, topicId)，不 instantiate 消息体
  try {
    rosbag::View index;
    index.addQuery(bag_, rosbag::TopicQuery(topics_));
    const uint32_t total = index.size();
    uint32_t n = 0;
    for (const rosbag::MessageInstance& m : index) {
      const uint8_t tid = topicId(m.getTopic());
      if (tid == 255) continue;
      Event ev;
      ev.t = m.getTime();
      ev.topic = tid;
      events_.push_back(ev);
      if (tid == 1) poseFrames_.push_back(events_.size() - 1);
      if (progress && total > 0 && ++n % 500 == 0) progress((double)n / (double)total);
    }
  } catch (const std::exception& e) {
    err_ = e.what();
    close();
    return false;
  }

  if (events_.empty()) {
    err_ = "bag 中没有 /slam2d/* 话题消息";
    close();
    return false;
  }

  // 初始状态：定位在第一帧（已应用、暂停）
  resetIterTo(0);
  head_ = events_.front().t;
  nextCpAt_ = kCpInterval;
  stepForward();
  if (progress) progress(1.0);
  return true;
}

void BagPlayer::close() {
  playing_ = false;
  liveView_.reset();
  if (bag_.isOpen()) bag_.close();
  events_.clear();
  poseFrames_.clear();
  cps_.clear();
  snap_ = SlamSnapshot{};
  idx_ = 0;
  nextCpAt_ = 0;
  head_ = ros::Time(0);
  path_.clear();
  err_.clear();
}

// ---------------------------------------------------------------- 时间轴

ros::Time BagPlayer::bagStart() const {
  return events_.empty() ? ros::Time(0) : events_.front().t;
}

ros::Time BagPlayer::bagEnd() const {
  return events_.empty() ? ros::Time(0) : events_.back().t;
}

double BagPlayer::durationSec() const {
  return (bagEnd() - bagStart()).toSec();
}

size_t BagPlayer::curFrameSlot() const {
  // poseFrames_ 中最后已应用（下标 < idx_）的位置；无已应用 pose 返回 size()
  size_t lo = 0, hi = poseFrames_.size();
  while (lo < hi) {
    const size_t mid = (lo + hi) / 2;
    if (poseFrames_[mid] < idx_) lo = mid + 1;
    else hi = mid;
  }
  return lo;  // lo == 已应用 pose 的个数，也是"下一帧"的槽位
}

size_t BagPlayer::frameIndex() const {
  const size_t slot = curFrameSlot();
  return slot == 0 ? 0 : slot - 1;
}

// ---------------------------------------------------------------- 播放控制

void BagPlayer::play() {
  if (events_.empty()) return;
  if (atEnd()) rebuildTo(0);  // 到尾重播：从头重建
  playing_ = true;
}

void BagPlayer::advance(double wallDtSec) {
  if (!playing_ || events_.empty()) return;
  const ros::Time boundary = head_ + ros::Duration(wallDtSec * speed_);
  // 应用所有 time <= boundary 的事件
  size_t target = std::upper_bound(events_.begin(), events_.end(), boundary,
                                   [](const ros::Time& t, const Event& ev) { return t < ev.t; }) -
                  events_.begin();
  if (target > idx_) applyTo(target);
  if (target >= events_.size()) playing_ = false;  // 到尾自动暂停
}

void BagPlayer::stepForward() {
  playing_ = false;
  if (events_.empty()) return;
  const size_t slot = curFrameSlot();
  if (slot >= poseFrames_.size()) {
    // 没有下一 pose 帧：直接到末尾
    applyTo(events_.size());
    return;
  }
  applyTo(poseFrames_[slot] + 1);  // 含该 pose 及其之前的全部事件
}

void BagPlayer::stepBackward() {
  playing_ = false;
  if (events_.empty()) return;
  const size_t slot = curFrameSlot();  // = 已应用 pose 数；当前帧 = slot-1（0 基）
  if (slot < 2) return;                // 已在第 0 帧或无 pose，不能再退
  rebuildTo(poseFrames_[slot - 2] + 1);  // 退到上一帧（含该帧全部事件）
}

void BagPlayer::seekToTime(const ros::Time& t) {
  if (events_.empty()) return;
  size_t target = std::upper_bound(events_.begin(), events_.end(), t,
                                   [](const ros::Time& tt, const Event& ev) { return tt < ev.t; }) -
                  events_.begin();
  if (target == idx_) return;
  if (target > idx_) applyTo(target);
  else rebuildTo(target);
}

// ---------------------------------------------------------------- 内部：应用

void BagPlayer::applyEvent(const rosbag::MessageInstance& m) {
  const uint8_t tid = topicId(m.getTopic());
  if (tid == 0) {
    auto msg = m.instantiate<nav_msgs::OccupancyGrid>();
    if (msg) snapshot_builder::applyMap(snap_, *msg, ++mapSeqOut_);
  } else if (tid == 1) {
    auto msg = m.instantiate<geometry_msgs::PoseStamped>();
    if (msg) snapshot_builder::applyPose(snap_, *msg);
  } else if (tid == 2) {
    auto msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg) snapshot_builder::applyInputCloud(snap_, *msg);
  } else if (tid == 3) {
    auto msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg) snapshot_builder::applyMappingCloud(snap_, *msg);
  }
}

void BagPlayer::resetIterTo(size_t k) {
  liveView_ = std::make_unique<rosbag::View>();
  liveView_->addQuery(bag_, rosbag::TopicQuery(topics_));
  liveIt_ = liveView_->begin();
  if (k > 0) {
    // 快进过已应用的 events_[0, k)：跳过 time <= events_[k-1].t 的全部事件。
    // （map_log 录制的 bag 时间戳唯一；手动 bag 的同戳尾部偏差对"最新值"语义无感）
    const ros::Time t = events_[k - 1].t;
    while (liveIt_ != liveView_->end() && liveIt_->getTime() <= t) ++liveIt_;
  }
}

void BagPlayer::applyTo(size_t targetIdx) {
  if (targetIdx <= idx_ || targetIdx > events_.size()) return;
  const ros::Time boundary = events_[targetIdx - 1].t;
  while (liveIt_ != liveView_->end() && liveIt_->getTime() <= boundary) {
    applyEvent(*liveIt_);
    ++liveIt_;
  }
  idx_ = targetIdx;
  head_ = boundary;

  // 检查点（仅前进路径保存；快照拷贝 = map shared_ptr + 几 KB 点云）
  if (idx_ >= nextCpAt_) {
    cps_.emplace_back(idx_, snap_);
    nextCpAt_ = idx_ + kCpInterval;
  }
}

void BagPlayer::rebuildTo(size_t targetIdx) {
  targetIdx = std::min(targetIdx, events_.size());
  if (targetIdx >= idx_) {
    applyTo(targetIdx);
    return;
  }

  // 找最近检查点：cps_ 按 first 升序，取最后一个 first <= targetIdx
  size_t cpPos = 0;
  {
    size_t lo = 0, hi = cps_.size();
    while (lo < hi) {
      const size_t mid = (lo + hi) / 2;
      if (cps_[mid].first <= targetIdx) lo = mid + 1;
      else hi = mid;
    }
    cpPos = lo;  // 命中的检查点下标 + 1（0 = 无命中，从头重放）
  }

  // 恢复状态。mapSeqOut_ 是高水位不回退 —— SlamView 靠 seq 变化重建 QImage
  if (cpPos > 0) {
    snap_ = cps_[cpPos - 1].second;
    idx_ = cps_[cpPos - 1].first;
  } else {
    snap_ = SlamSnapshot{};
    idx_ = 0;
  }
  resetIterTo(idx_);
  applyTo(targetIdx);
  // applyTo 在 targetIdx == 已应用下标时是 no-op，head_ 仍需对齐（如到尾重播回 0）
  head_ = targetIdx == 0 ? events_.front().t : events_[targetIdx - 1].t;
}

}  // namespace flat_sim_viewer
