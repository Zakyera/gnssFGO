#pragma once
// CP70 diagnostic delivery only. Never reads/reconstructs a belief payload.
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cpv70 {
using Id = uint64_t;
using Edge = std::pair<Id, Id>;
struct Entry {
  Id update, from, to;
};
struct Receipt {
  Id update, from, to, first_seen;
};
template <class Belief> struct Selection {
  std::vector<Belief> released, held;
  std::vector<Receipt> receipts;
};
class RecordedUpdateGate {
  bool enabled_ = false, begun_ = false;
  Id last_update_ = 0;
  std::vector<Entry> schedule_;
  std::map<Edge, size_t> indices_;
  std::map<Edge, Id> first_seen_;
  std::set<Edge> released_;
  static void require(bool ok, const std::string& why) {
    if (!ok) throw std::runtime_error("CPV70_DELIVERY_FAILURE: " + why);
  }
 public:
  RecordedUpdateGate() = default;
  explicit RecordedUpdateGate(std::vector<Entry> schedule)
      : enabled_(true), schedule_(std::move(schedule)) {
    require(!schedule_.empty(), "empty schedule");
    for (size_t i = 0; i < schedule_.size(); ++i) {
      const auto& r = schedule_[i];
      require(r.from < r.to, "invalid sender edge");
      require(i == 0 || schedule_[i-1].update <= r.update,
              "nonmonotonic schedule");
      require(indices_.emplace(Edge{r.from, r.to}, i).second,
              "duplicate schedule edge");
    }
  }
  static RecordedUpdateGate fromEnvironment() {
    const char* path = std::getenv("CPV70_RECORDED_UPDATE_SCHEDULE");
    if (!path || !*path) return RecordedUpdateGate();
    std::ifstream f(path);
    require(f.good(), "schedule open failed");
    std::vector<Entry> rows;
    std::string line;
    while (std::getline(f, line)) {
      require(!line.empty() && line.find_first_not_of("0123456789 ")==std::string::npos,
              "invalid schedule characters");
      std::istringstream s(line);
      Entry r{}; std::string extra;
      require(bool(s >> r.update >> r.from >> r.to) && !(s >> extra),
              "invalid schedule row");
      rows.push_back(r);
    }
    require(f.eof(), "schedule read failure");
    return RecordedUpdateGate(std::move(rows));
  }
  bool enabled() const { return enabled_; }
  size_t releasedCount() const { return released_.size(); }
  // CP73 read-only preflight: only a currently due missing input may wait.
  // All structural, duplicate, late and past-missing guards still fail now.
  template <class Belief>
  bool ready(Id update, const std::vector<Belief>& arrived) const {
    auto trial = *this;
    if (!enabled_) return true;
    trial.selectImpl(update, arrived, false);
    for (const auto& r : schedule_) {
      if (r.update > update) break;
      if (!trial.released_.count(Edge{r.from, r.to})) return false;
    }
    return true;
  }
  // Atomic scheduler state: failure cannot partially mark a release successful.
  template <class Belief>
  Selection<Belief> select(Id update, const std::vector<Belief>& arrived) {
    if (!enabled_) return {arrived, {}, {}};
    auto trial = *this;
    auto result = trial.selectImpl(update, arrived);
    *this = std::move(trial);
    return result;
  }
 private:
  template <class Belief>
  Selection<Belief> selectImpl(Id update, const std::vector<Belief>& arrived,
                              bool require_complete = true) {
    require(!begun_ || update > last_update_, "repeated/reversed backend update");
    begun_ = true; last_update_ = update;
    Selection<Belief> out;
    std::set<Edge> seen_this_update;
    std::vector<std::pair<size_t, Belief>> due;
    for (const auto& b : arrived) {
      const Edge key{b.sender_from_pose_index, b.sender_to_pose_index};
      require(b.source_agent == static_cast<uint8_t>('g'), "unexpected source");
      const auto it = indices_.find(key);
      require(it != indices_.end(), "unlisted sender edge");
      require(seen_this_update.insert(key).second, "duplicate pending edge");
      require(!released_.count(key), "duplicate already-released edge");
      const auto& r = schedule_[it->second];
      require(update <= r.update, "late sender edge " + std::to_string(r.from));
      first_seen_.emplace(key, update);
      if (update < r.update) out.held.push_back(b);
      else due.emplace_back(it->second, b);
    }
    // Preserve recorded order if several callbacks enter the same backend update.
    std::sort(due.begin(), due.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });
    for (const auto& p : due) {
      const auto& r = schedule_[p.first]; const Edge key{r.from, r.to};
      released_.insert(key); out.released.push_back(p.second);
      out.receipts.push_back({update, r.from, r.to, first_seen_.at(key)});
    }
    for (const auto& r : schedule_) {
      if (r.update > update) break;
      require((!require_complete && r.update == update) ||
                  released_.count(Edge{r.from, r.to}),
              "missing at target update " + std::to_string(r.update) +
              " sender " + std::to_string(r.from) + "->" + std::to_string(r.to));
    }
    return out;
  }
};
inline RecordedUpdateGate& deliveryGate() {
  static RecordedUpdateGate gate = RecordedUpdateGate::fromEnvironment();
  return gate;
}
}  // namespace cpv70
