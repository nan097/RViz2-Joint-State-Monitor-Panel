#include "joint_state_rviz_panel/joint_data_manager.hpp"
#include <cmath>
#include <unordered_set>
namespace jsm {
void JointDataManager::update(const sensor_msgs::msg::JointState &msg,
                              Clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto warn = [&](const std::string &text) {
    ++warnings_;
    diagnostic_ = text;
  };
  if (msg.position.empty() || msg.position.size() != msg.name.size()) {
    warn("Empty or mismatched position array");
    return;
  }
  std::unordered_set<std::string> seen;
  bool valid = false;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    const auto &name = msg.name[i];
    if (name.empty() || !seen.insert(name).second) {
      warn("Empty/duplicate joint name (first occurrence wins)");
      continue;
    }
    if (!std::isfinite(msg.position[i])) {
      warn("Non-finite position: " + name);
      continue;
    }
    auto it = index_.find(name);
    if (it == index_.end()) {
      index_[name] = joints_.size();
      joints_.push_back({name, msg.position[i], now});
    } else {
      auto &j = joints_[it->second];
      j.position = msg.position[i];
      j.received = now;
    }
    valid = true;
  }
  if (valid) {
    last_ = now;
    times_.push_back(now);
    while (!times_.empty() && now - times_.front() > std::chrono::seconds(3))
      times_.pop_front();
  }
}
Snapshot JointDataManager::snapshot(Clock::time_point now) const {
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot s;
  s.joints = joints_;
  s.last_valid = last_;
  s.warnings = warnings_;
  s.diagnostic = diagnostic_;
  auto first = times_.begin();
  while (first != times_.end() && now - *first > std::chrono::seconds(3))
    ++first;
  auto n = std::distance(first, times_.end());
  if (n >= 2) {
    double span = std::chrono::duration<double>(times_.back() - *first).count();
    if (span >= 0.2)
      s.hz = (n - 1) / span;
  }
  return s;
}
void JointDataManager::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  joints_.clear();
  index_.clear();
  times_.clear();
  last_.reset();
  warnings_ = 0;
  diagnostic_.clear();
}
} // jsm 命名空间结束
