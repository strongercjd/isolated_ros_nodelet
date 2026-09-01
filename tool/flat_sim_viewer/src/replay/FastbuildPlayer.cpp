#include "replay/FastbuildPlayer.h"

#include <algorithm>

namespace flat_sim_viewer {

bool FastbuildPlayer::openDecisionLog(const std::string& path,
                                      const std::function<void(double)>& progress) {
  err_.clear();
  std::string err;
  std::vector<DecisionRecord> records;
  if (!parseDecisionLog(path, progress, records, err)) {
    err_ = err;
    return false;
  }
  decision_path_ = path;
  // 记录按日志顺序（时间单调）；保险起见按时间排序
  std::sort(records.begin(), records.end(), [](const DecisionRecord& a, const DecisionRecord& b) {
    return a.sec != b.sec ? a.sec < b.sec : a.nsec < b.nsec;
  });
  records_ = std::move(records);
  idx_ = 0;
  if (slam_.isOpen() && !records_.empty())
    rebuildTo(firstRecordTime());  // 已有背景：定位到首条记录
  return true;
}

bool FastbuildPlayer::openSlamBag(const std::string& path,
                                  const std::function<void(double)>& progress) {
  err_.clear();
  if (!slam_.open(path, progress)) {
    err_ = slam_.error();
    return false;
  }
  if (!records_.empty()) rebuildTo(firstRecordTime());
  return true;
}

void FastbuildPlayer::close() {
  slam_.close();
  records_.clear();
  decision_path_.clear();
  idx_ = 0;
  head_ = ros::Time(0);
  playing_ = false;
  snap_ = SlamSnapshot{};
  err_.clear();
}

// 播放控制：时间推进完全交给 BagPlayer（slam_ 持有自己的 pending_ / speed_ / atEnd）
void FastbuildPlayer::play() {
  if (records_.empty() || !slam_.isOpen()) return;
  if (slam_.atEnd()) {
    // 到尾重播：从头定位（overlay 回第一条）
    slam_.pause();
    rebuildTo(ros::Time(records_[0].sec, records_[0].nsec));
  }
  slam_.setSpeed(speed_);
  slam_.play();
  playing_ = true;
}

void FastbuildPlayer::advance(double wallDtSec) {
  if (!playing_ || records_.empty()) return;
  slam_.advance(wallDtSec);
  head_ = slam_.playhead();
  if (!slam_.isPlaying()) playing_ = false;  // 到尾自动暂停
  // 每 tick 重拷 slam 快照（地图/点云/位姿随播放推进；否则 snap_ 停留在最后一次
  // rebuildTo，播放时地图背景不刷新甚至缺失），再叠 ≤playhead 的决策覆盖层
  snap_ = slam_.snapshot();
  snap_.decision = SlamSnapshot::DecisionOverlay{};
  idx_ = recordAtOrBefore(head_);
  if (idx_ < records_.size())
    applyRecord(records_[idx_]);
}

void FastbuildPlayer::stepForward() {
  playing_ = false;
  slam_.pause();
  if (records_.empty()) return;
  if (idx_ >= records_.size()) {
    idx_ = 0;  // 时间轴在首条决策之前（idx_==size 哨兵）→ 前进到首条
  } else if (idx_ >= records_.size() - 1) {
    return;  // 已在末条
  } else {
    ++idx_;
  }
  rebuildTo(ros::Time(records_[idx_].sec, records_[idx_].nsec));
}

void FastbuildPlayer::stepBackward() {
  playing_ = false;
  slam_.pause();
  if (records_.empty() || idx_ == 0 || idx_ == records_.size()) return;
  --idx_;
  rebuildTo(ros::Time(records_[idx_].sec, records_[idx_].nsec));
}

void FastbuildPlayer::seekToRecord(size_t i) {
  if (records_.empty() || i >= records_.size()) return;
  idx_ = i;
  rebuildTo(ros::Time(records_[i].sec, records_[i].nsec));
}

void FastbuildPlayer::seekToTime(const ros::Time& t) {
  if (records_.empty() || !slam_.isOpen()) return;
  const size_t target = recordAtOrBefore(t);
  idx_ = target;
  rebuildTo(t);  // slam seek 到 t；overlay 取 recordAtOrBefore(t)
}

// slam seek 到 t（应用全部 ≤t 事件 → 地图/位姿状态），再填决策覆盖层
void FastbuildPlayer::rebuildTo(const ros::Time& t) {
  playing_ = false;
  slam_.pause();
  slam_.seekToTime(t);
  head_ = t;
  snap_ = slam_.snapshot();  // 拷贝：map shared_ptr + 点云 + 位姿
  snap_.decision.has = false;
  if (idx_ < records_.size()) applyRecord(records_[idx_]);
}

void FastbuildPlayer::applyRecord(const DecisionRecord& r) {
  auto& d = snap_.decision;
  d.has = true;
  d.candidates.clear();
  d.candidates.reserve(r.candidates.size());
  for (const auto& c : r.candidates) {
    SlamSnapshot::DecisionOverlay::Cand cand;
    cand.x = c.x;
    cand.y = c.y;
    cand.size = c.size;
    cand.via = c.via;
    d.candidates.push_back(cand);
  }
  d.hasSelected = r.has_selected;
  d.selX = r.sel_x;
  d.selY = r.sel_y;
  d.blacklist = r.blacklist;
  d.areaM2 = r.area_m2;
  d.taskState = r.task_state;
}

ros::Time FastbuildPlayer::firstRecordTime() const {
  if (records_.empty()) return ros::Time(0);
  const ros::Time t0(records_[0].sec, records_[0].nsec);
  const ros::Time start = slam_.bagStart();
  return start.isZero() ? t0 : (start < t0 ? t0 : start);  // 取两者较晚者
}

// 最后一条 time ≤ t 的记录下标；无则 records_.size()
size_t FastbuildPlayer::recordAtOrBefore(const ros::Time& t) const {
  size_t lo = 0, hi = records_.size();
  while (lo < hi) {
    const size_t mid = (lo + hi) / 2;
    const auto& r = records_[mid];
    const ros::Time rt(r.sec, r.nsec);
    if (rt <= t) lo = mid + 1;
    else hi = mid;
  }
  return lo == 0 ? records_.size() : lo - 1;
}

}  // namespace flat_sim_viewer
