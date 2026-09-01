// flat_sim_viewer —— DecisionLog 解析 headless 测试（无 Qt / ROS，仅 nlohmann）
//
// 构造含 FASTBUILD_DECISION 标记行的样例日志 → parseDecisionLog →
// 校验记录数、时间戳、候选/选中/拉黑/面积字段、坏行容错、错误路径。
//
// 编译（在 tool/flat_sim_viewer 下）：
//   g++ -std=c++17 -I src -I src/third_party \
//       tests/test_decision_log.cpp src/replay/DecisionLog.cpp \
//       -o test_decision_log && ./test_decision_log
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "replay/DecisionLog.h"

using flat_sim_viewer::DecisionRecord;
using flat_sim_viewer::parseDecisionLog;

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

static bool near(float a, float b) { return std::fabs(a - b) < 1e-4f; }

int main() {
  const std::string path = "/tmp/flat_viewer_test_decision_" +
                           std::to_string((long)getpid()) + ".log";

  std::printf("== 1. 构造样例日志并解析 ==\n");
  {
    std::ofstream out(path);
    out << "[INFO] [09-01 23:09:45] [/fastbuild_task] [fastbuild_task_nodelet.cpp:52]: "
           "FASTBUILD_DECISION {\"seq\":1,\"sec\":1730000000,\"nsec\":100000000,"
           "\"task_state\":1,\"area\":4.20,\"has_selected\":true,\"sel_x\":1.20,\"sel_y\":3.40,"
           "\"candidates\":[{\"x\":1.20,\"y\":3.40,\"size\":8,\"via\":0},"
           "{\"x\":-2.50,\"y\":1.00,\"size\":3,\"via\":1}],"
           "\"blacklist\":[{\"x\":-1.0,\"y\":-2.0}]}\n";
    out << "[INFO] [09-01 23:10:01] [/fastbuild_task] [fastbuild_task_nodelet.cpp:52]: "
           "FASTBUILD_DECISION {\"seq\":2,\"sec\":1730000040,\"nsec\":500000000,"
           "\"task_state\":1,\"area\":5.60,\"has_selected\":false,"
           "\"candidates\":[{\"x\":0.30,\"y\":0.10,\"size\":2,\"via\":0}],"
           "\"blacklist\":[]}\n";
    // 坏行：标记后不是合法 JSON → 跳过
    out << "[INFO] [09-01 23:10:02] [/fastbuild_task] [...]: FASTBUILD_DECISION {broken json\n";
    // 无标记行 → 忽略
    out << "[INFO] [09-01 23:10:03] [/base_navi] [x.cpp:1]: 普通日志，无标记\n";
    // 标记但缺 sec → 跳过（协议要求 sec 为权威时间戳）
    out << "[INFO] [...] : FASTBUILD_DECISION {\"seq\":9,\"has_selected\":true}\n";
    out << "[INFO] [09-01 23:11:00] [/fastbuild_task] [...]: "
           "FASTBUILD_DECISION {\"seq\":3,\"sec\":1730000060,\"nsec\":0,"
           "\"task_state\":4,\"area\":7.80,\"has_selected\":true,\"sel_x\":5.00,\"sel_y\":-1.00,"
           "\"candidates\":[],\"blacklist\":[{\"x\":1.0,\"y\":2.0},{\"x\":3.0,\"y\":4.0}]}\n";
    out.close();

    std::vector<DecisionRecord> recs;
    std::string err;
    int progressCalls = 0;
    double lastProgress = 0;
    const bool ok = parseDecisionLog(
        path, [&](double p) { ++progressCalls; lastProgress = p; }, recs, err);
    CHECK(ok, "解析成功");
    CHECK(recs.size() == 3, "记录数 = 3（跳过 2 条坏行）");
    CHECK(progressCalls > 0 && lastProgress > 0.99, "进度回调推进到 1.0");

    const DecisionRecord& r0 = recs[0];
    CHECK(r0.sec == 1730000000ull && r0.nsec == 100000000u, "r0 时间戳 sec/nsec");
    CHECK(r0.seq == 1, "r0 seq = 1");
    CHECK(r0.task_state == 1, "r0 task_state = RUNNING(1)");
    CHECK(near(r0.area_m2, 4.20f), "r0 面积 = 4.20");
    CHECK(r0.has_selected && near(r0.sel_x, 1.20f) && near(r0.sel_y, 3.40f),
          "r0 选中点 (1.20, 3.40)");
    CHECK(r0.candidates.size() == 2, "r0 候选数 = 2");
    CHECK(near(r0.candidates[0].x, 1.20f) && near(r0.candidates[0].y, 3.40f) &&
              r0.candidates[0].size == 8 && r0.candidates[0].via == 0,
          "r0 候选[0] 直线可达(x=1.2,y=3.4,size=8,via=0)");
    CHECK(r0.candidates[1].via == 1, "r0 候选[1] 绕行(via=1)");
    CHECK(r0.blacklist.size() == 1 && near(r0.blacklist[0].first, -1.0f) &&
              near(r0.blacklist[0].second, -2.0f),
          "r0 拉黑点 1 个 (-1.0, -2.0)");

    const DecisionRecord& r1 = recs[1];
    CHECK(!r1.has_selected, "r1 未选出目标");
    CHECK(r1.candidates.size() == 1 && r1.blacklist.empty(), "r1 候选 1 / 拉黑空");

    const DecisionRecord& r2 = recs[2];
    CHECK(r2.sec == 1730000060ull && r2.task_state == 4, "r2 时间戳 / task_state = DONE(4)");
    CHECK(r2.has_selected && near(r2.sel_x, 5.00f) && near(r2.sel_y, -1.00f),
          "r2 选中点 (5.00, -1.00)");
    CHECK(r2.blacklist.size() == 2, "r2 拉黑点 2 个");
  }

  std::printf("== 2. 错误路径 ==\n");
  {
    std::vector<DecisionRecord> recs;
    std::string err;
    CHECK(!parseDecisionLog("/nonexistent/x.log", nullptr, recs, err), "文件不存在 → false");
    CHECK(!err.empty(), "err 有信息");
    err.clear();

    std::ofstream empty("/tmp/flat_viewer_test_empty.log");
    empty << "只有普通日志，没有 FASTBUILD_DECISION 标记\n";
    empty.close();
    CHECK(!parseDecisionLog("/tmp/flat_viewer_test_empty.log", nullptr, recs, err),
          "无记录 → false");
    CHECK(!err.empty(), "err 提示无记录");
    std::remove("/tmp/flat_viewer_test_empty.log");
  }

  std::remove(path.c_str());
  std::printf(failures == 0 ? "\n全部通过\n" : "\n失败 %d 项\n", failures);
  return failures == 0 ? 0 : 1;
}
