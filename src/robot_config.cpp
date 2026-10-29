#include "joint_state_rviz_panel/robot_config.hpp"
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace jsm {
void RobotConfig::validate() const {
  auto require = [](bool ok, const char *s) {
    if (!ok)
      throw std::runtime_error(s);
  };
  require(!topic.empty(), "topic cannot be empty");
  require(std::isfinite(refresh_hz) && refresh_hz >= 1 && refresh_hz <= 100,
          "refresh_hz must be 1..100");
  require(std::isfinite(warn) && std::isfinite(timeout) && warn > 0 &&
              timeout > warn,
          "Require 0 < warn_after_sec < timeout_after_sec");
  require(reliability == "best_effort" || reliability == "reliable",
          "Invalid QoS reliability");
  require(unit == "deg" || unit == "rad", "Invalid rotation unit");
  require(mode == "auto" || mode == "configured", "Invalid mode");
  require(profiles.count(profile), "Unknown robot_profile");
  for (const auto &entry : profiles) {
    std::set<std::string> names;
    for (const auto &name : entry.second.names)
      require(!name.empty() && names.insert(name).second,
              "Empty/duplicate configured joint name");
    for (const auto &item : entry.second.joints) {
      const auto &j = item.second;
      require(j.type == "unknown" || j.type == "revolute" ||
                  j.type == "continuous" || j.type == "prismatic" ||
                  j.type == "fixed",
              "Invalid joint type");
      require(j.lower.has_value() == j.upper.has_value(),
              "Limits require both lower and upper");
      if (j.lower)
        require(std::isfinite(*j.lower) && std::isfinite(*j.upper) &&
                    *j.lower < *j.upper,
                "Invalid limits");
    }
  }
}
RobotConfig RobotConfig::load(const std::string &path) {
  if (path.empty())
    return RobotConfig{};
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot open config: " + path);
  std::ostringstream text;
  text << stream.rdbuf();
  return fromYaml(text.str());
}
RobotConfig RobotConfig::fromYaml(const std::string &text) {
  RobotConfig c;
  auto root = YAML::Load(text);
  if (!root.IsMap())
    throw std::runtime_error("Configuration must be a YAML mapping");
  auto m = root["monitor"];
  if (m) {
#define READ(key, field, type)                                                 \
  if (m[key])                                                                  \
    c.field = m[key].as<type>();
    READ("topic", topic, std::string)
    READ("refresh_hz", refresh_hz, double)
    READ("qos_reliability", reliability, std::string)
    READ("warn_after_sec", warn, double)
    READ("timeout_after_sec", timeout, double)
    READ("default_rotation_unit", unit, std::string)
    READ("mode", mode, std::string)
    READ("robot_profile", profile, std::string)
    READ("assume_unknown_revolute", assume, bool)
#undef READ
  }
  if (auto robots = root["robots"])
    for (const auto &entry : robots) {
      Profile p;
      auto key = entry.first.as<std::string>();
      auto r = entry.second;
      p.display_name =
          r["display_name"] ? r["display_name"].as<std::string>() : key;
      if (r["joint_names"])
        p.names = r["joint_names"].as<std::vector<std::string>>();
      if (auto joints = r["joints"])
        for (const auto &item : joints) {
          JointSpec j;
          auto v = item.second;
          if (v["type"])
            j.type = v["type"].as<std::string>();
          if (v["lower"])
            j.lower = v["lower"].as<double>();
          if (v["upper"])
            j.upper = v["upper"].as<double>();
          p.joints[item.first.as<std::string>()] = j;
        }
      c.profiles[key] = p;
    }
  c.validate();
  return c;
}
std::string RobotConfig::toYaml() const {
  YAML::Node root;
  auto m = root["monitor"];
  m["topic"] = topic;
  m["refresh_hz"] = refresh_hz;
  m["qos_reliability"] = reliability;
  m["warn_after_sec"] = warn;
  m["timeout_after_sec"] = timeout;
  m["default_rotation_unit"] = unit;
  m["mode"] = mode;
  m["robot_profile"] = profile;
  m["assume_unknown_revolute"] = assume;
  for (const auto &entry : profiles) {
    auto p = root["robots"][entry.first];
    p["display_name"] = entry.second.display_name;
    p["joint_names"] = entry.second.names;
    for (const auto &item : entry.second.joints) {
      auto j = p["joints"][item.first];
      j["type"] = item.second.type;
      if (item.second.lower) {
        j["lower"] = *item.second.lower;
        j["upper"] = *item.second.upper;
      }
    }
  }
  YAML::Emitter out;
  out << root;
  return out.c_str();
}
} // jsm 命名空间结束
