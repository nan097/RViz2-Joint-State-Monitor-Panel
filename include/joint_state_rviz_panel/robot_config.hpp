#pragma once
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace jsm {
struct JointSpec {
  std::string type = "unknown";
  std::optional<double> lower, upper;
};
struct Profile {
  std::string display_name;
  std::vector<std::string> names;
  std::map<std::string, JointSpec> joints;
};
struct RobotConfig {
  std::string topic = "/joint_states", reliability = "best_effort",
              unit = "deg", mode = "auto", profile = "generic";
  double refresh_hz = 20, warn = 1, timeout = 2;
  bool assume = false;
  std::map<std::string, Profile> profiles{
      {"generic", Profile{"Generic Robot", {}, {}}}};
  static RobotConfig load(const std::string &path);
  static RobotConfig fromYaml(const std::string &text);
  std::string toYaml() const;
  void validate() const;
};
} // jsm 命名空间结束
