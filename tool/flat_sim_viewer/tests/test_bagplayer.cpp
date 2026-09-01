// flat_sim_viewer —— BagPlayer headless 回归测试（无 Qt，直接驱动回放引擎）
//
// 用真实录制的 bag 验证：加载索引、首帧补写快照、单步、seq 高水位（回退重建）、
// 播放推进、到尾自动暂停、到尾重播。用法：
//   test_bagplayer <file.bag>
#include <cmath>
#include <cstdio>
#include <string>

#include "replay/BagPlayer.h"

using flat_sim_viewer::BagPlayer;

static int failures = 0;

#define CHECK(cond, msg)                                            \
  do {                                                              \
    if (cond) {                                                     \
      std::printf("  [PASS] %s\n", msg);                            \
    } else {                                                        \
      std::printf("  [FAIL] %s  (line %d)\n", msg, __LINE__);       \
      ++failures;                                                   \
    }                                                               \
  } while (0)

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "用法: %s <file.bag>\n", argv[0]);
    return 2;
  }
  const std::string path = argv[1];

  std::printf("== 1. open 与索引 ==\n");
  BagPlayer p;
  CHECK(p.open(path), "open 成功");
  CHECK(p.frameCount() > 10, "pose 帧数 > 10");
  CHECK(p.durationSec() > 1.0, "时长 > 1s");
  std::printf("  frameCount=%zu duration=%.2fs\n", p.frameCount(), p.durationSec());

  std::printf("== 2. 首帧补写快照（门控重开时写入的完整状态）==\n");
  {
    const auto& s = p.snapshot();
    CHECK(s.hasPose, "首帧有位姿");
    CHECK(s.hasInput, "首帧有 input 点云");
    CHECK(s.hasMapping, "首帧有 mapping 点云");
    CHECK(s.map.seq > 0 && s.map.data && !s.map.data->empty(), "首帧有地图数据");
    CHECK(!p.isPlaying(), "open 后处于暂停");
  }

  std::printf("== 3. 单步前进 ==\n");
  {
    const size_t f0 = p.frameIndex();
    const auto t0 = p.playhead();
    const uint32_t seq0 = p.snapshot().map.seq;
    p.stepForward();
    p.stepForward();
    p.stepForward();
    CHECK(p.frameIndex() == f0 + 3, "frameIndex +3");
    CHECK(p.playhead() > t0, "playhead 前进");
    CHECK(p.snapshot().map.seq >= seq0, "map seq 单调不减");
    CHECK(!p.isPlaying(), "单步隐含暂停");
  }

  std::printf("== 4. seek 回退与 seq 高水位（SlamView 靠 seq 重建 QImage）==\n");
  {
    const uint32_t seqBefore = p.snapshot().map.seq;
    p.seekToTime(p.bagStart());  // 拖回开头：rebuildTo(0) 路径
    CHECK(p.snapshot().map.seq > seqBefore, "回退后 seq 仍递增（高水位）");
    CHECK(p.snapshot().hasPose, "回退后首帧状态完整");
    const size_t fi = p.frameIndex();
    CHECK(fi <= 1, "回退后位于开头帧");
  }

  std::printf("== 5. 播放推进与倍速 ==\n");
  {
    p.play();
    CHECK(p.isPlaying(), "play 后 isPlaying");
    const auto t0 = p.playhead();
    for (int i = 0; i < 10; ++i) p.advance(0.1);  // 1.0s 墙钟
    CHECK(p.playhead() > t0, "advance 推进 playhead");
    p.setSpeed(2.0);
    const auto t1 = p.playhead();
    p.advance(0.5);  // 墙钟 0.5s → bag 时间 1.0s
    CHECK((p.playhead() - t1).toSec() > 0.9, "2× 倍速生效");
  }

  std::printf("== 6. 播到末尾自动暂停 ==\n");
  {
    p.play();
    p.advance(p.durationSec() + 5.0);  // 一次跳过末尾
    CHECK(!p.isPlaying(), "到尾自动暂停");
    CHECK(p.atEnd(), "atEnd");
  }

  std::printf("== 7. 到尾重播 ==\n");
  {
    p.play();
    CHECK(p.isPlaying(), "到尾后再 play 进入播放");
    CHECK(!p.atEnd(), "playhead 回到开头");
    CHECK((p.playhead() - p.bagStart()).toSec() < 0.5, "重播从头开始");
    p.pause();
  }

  std::printf("== 8. 单步后退（检查点重建路径）==\n");
  {
    p.seekToTime(p.bagStart() + ros::Duration(p.durationSec() * 0.7));
    const size_t f = p.frameIndex();
    p.stepBackward();
    CHECK(p.frameIndex() == f - 1, "stepBackward 帧号 -1");
    p.stepBackward();
    p.stepBackward();
    CHECK(p.frameIndex() == f - 3, "连续后退 -3");
    CHECK(p.snapshot().hasPose, "后退后状态完整");
  }

  std::printf("== 9. 关闭与错误路径 ==\n");
  {
    p.close();
    CHECK(!p.isOpen(), "close 后 isOpen=false");
    CHECK(!p.open("/nonexistent/x.bag"), "打开不存在文件失败");
    CHECK(!p.error().empty(), "error() 有信息");
  }

  std::printf(failures == 0 ? "\n全部通过\n" : "\n失败 %d 项\n", failures);
  return failures == 0 ? 0 : 1;
}
