#pragma once
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <unordered_map>
#include <vector>
namespace jsm {
using Clock = std::chrono::steady_clock;
struct Joint {
  std::string name;
  double position = 0;
  std::optional<Clock::time_point> received;
};
struct Snapshot {
  std::vector<Joint> joints;
  double hz = 0;
  std::optional<Clock::time_point> last_valid;
  size_t warnings = 0;
  std::string diagnostic;
};
class JointDataManager {
  mutable std::mutex mutex_;
  std::vector<Joint> joints_;
  std::unordered_map<std::string, size_t> index_;
  std::deque<Clock::time_point> times_;
  std::optional<Clock::time_point> last_;
  size_t warnings_ = 0;
  std::string diagnostic_;

public:
  void update(const sensor_msgs::msg::JointState &,
              Clock::time_point now = Clock::now());
  Snapshot snapshot(Clock::time_point now = Clock::now()) const;
  void clear();
};
} // jsm 命名空间结束
