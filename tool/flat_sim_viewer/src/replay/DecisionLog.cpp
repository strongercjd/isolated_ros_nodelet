#include "replay/DecisionLog.h"

#include <cstring>
#include <exception>
#include <fstream>

#include <nlohmann/json.hpp>

namespace flat_sim_viewer {

namespace {

const char* const kMarker = "FASTBUILD_DECISION";

}  // namespace

bool parseDecisionLog(const std::string& path,
                      const std::function<void(double)>& progress,
                      std::vector<DecisionRecord>& out,
                      std::string& err) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    err = "cannot open decision log: " + path;
    return false;
  }
  in.seekg(0, std::ios::end);
  const std::streamoff total = in.tellg();
  in.seekg(0, std::ios::beg);

  out.clear();
  size_t dropped = 0;
  std::string line;
  while (std::getline(in, line)) {
    const size_t pos = line.find(kMarker);
    if (pos == std::string::npos) continue;
    const std::string payload = line.substr(pos + std::strlen(kMarker));
    try {
      const nlohmann::json j = nlohmann::json::parse(payload);
      if (!j.is_object() || !j.contains("sec")) {
        ++dropped;
        continue;
      }
      DecisionRecord r;
      r.sec = j.value("sec", (uint64_t)0);
      r.nsec = j.value("nsec", (uint32_t)0);
      r.seq = j.value("seq", (uint32_t)0);
      r.task_state = j.value("task_state", (uint8_t)0);
      r.area_m2 = j.value("area", 0.0f);
      r.has_selected = j.value("has_selected", false);
      r.sel_x = j.value("sel_x", 0.0f);
      r.sel_y = j.value("sel_y", 0.0f);
      if (j.contains("candidates") && j["candidates"].is_array()) {
        for (const auto& c : j["candidates"]) {
          DecisionRecord::Cand cand;
          cand.x = c.value("x", 0.0f);
          cand.y = c.value("y", 0.0f);
          cand.size = c.value("size", (uint32_t)0);
          cand.via = c.value("via", (uint8_t)0);
          r.candidates.push_back(cand);
        }
      }
      if (j.contains("blacklist") && j["blacklist"].is_array()) {
        for (const auto& b : j["blacklist"])
          r.blacklist.emplace_back(b.value("x", 0.0f), b.value("y", 0.0f));
      }
      out.push_back(std::move(r));
    } catch (const std::exception&) {
      ++dropped;
    }
    if (progress && total > 0) progress((double)in.tellg() / (double)total);
  }

  if (out.empty()) {
    err = "no FASTBUILD_DECISION records in " + path;
    return false;
  }
  if (progress) progress(1.0);
  return true;
}

}  // namespace flat_sim_viewer
