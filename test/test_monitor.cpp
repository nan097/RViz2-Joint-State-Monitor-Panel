#include "joint_state_rviz_panel/joint_data_manager.hpp"
#include "joint_state_rviz_panel/robot_config.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace jsm;
static sensor_msgs::msg::JointState message(std::vector<std::string> names,
                                            std::vector<double> values) {
  sensor_msgs::msg::JointState m;
  m.name = std::move(names);
  m.position = std::move(values);
  return m;
}
TEST(Cache, StableNameMappingAndPartialUpdates) {
  JointDataManager d;
  auto t = Clock::now();
  d.update(message({"a", "b"}, {1, 2}), t);
  d.update(message({"b", "a"}, {3, 4}), t + std::chrono::seconds(1));
  d.update(message({"a"}, {5}), t + std::chrono::seconds(2));
  auto s = d.snapshot(t + std::chrono::seconds(2));
  ASSERT_EQ(s.joints.size(), 2u);
  EXPECT_EQ(s.joints[0].name, "a");
  EXPECT_DOUBLE_EQ(s.joints[0].position, 5);
  EXPECT_EQ(s.joints[1].name, "b");
  EXPECT_DOUBLE_EQ(s.joints[1].position, 3);
  EXPECT_EQ(s.joints[1].received, t + std::chrono::seconds(1));
}
TEST(Cache, InvalidDataDoesNotRefreshTime) {
  JointDataManager d;
  auto t = Clock::now();
  d.update(message({"a"}, {1}), t);
  d.update(message({"a"}, {}));
  d.update(message({"a"}, {1, 2}));
  d.update(message({"a"}, {std::numeric_limits<double>::quiet_NaN()}));
  d.update(message({"a"}, {std::numeric_limits<double>::infinity()}));
  auto s = d.snapshot();
  EXPECT_EQ(s.last_valid, t);
  EXPECT_EQ(s.warnings, 4u);
  EXPECT_DOUBLE_EQ(s.joints[0].position, 1);
}
TEST(Cache, DuplicateFirstWinsAndEmptyNameSkipped) {
  JointDataManager d;
  d.update(message({"a", "a", ""}, {1, 2, 3}));
  auto s = d.snapshot();
  ASSERT_EQ(s.joints.size(), 1u);
  EXPECT_DOUBLE_EQ(s.joints[0].position, 1);
  EXPECT_EQ(s.warnings, 2u);
}
TEST(Cache, SnapshotIsIndependentAndClearResets) {
  JointDataManager d;
  d.update(message({"a"}, {1}));
  auto s = d.snapshot();
  d.clear();
  EXPECT_EQ(s.joints.size(), 1u);
  auto next = d.snapshot();
  EXPECT_TRUE(next.joints.empty());
  EXPECT_FALSE(next.last_valid);
  EXPECT_EQ(next.hz, 0);
}
TEST(Cache, RateAndIdleDecay) {
  JointDataManager d;
  auto t = Clock::now();
  for (int i = 0; i <= 100; ++i)
    d.update(message({"a"}, {1}), t + std::chrono::milliseconds(i * 20));
  EXPECT_NEAR(d.snapshot(t + std::chrono::seconds(2)).hz, 50, 0.01);
  EXPECT_DOUBLE_EQ(d.snapshot(t + std::chrono::seconds(6)).hz, 0);
}
TEST(Config, RejectInvalidThresholdsAndLimits) {
  RobotConfig c;
  c.timeout = 0;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = RobotConfig{};
  c.profiles["generic"].joints["a"] = {"prismatic", 2, 1};
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = RobotConfig{};
  c.profiles["generic"].names = {"a", "a"};
  EXPECT_THROW(c.validate(), std::runtime_error);
  c = RobotConfig{};
  EXPECT_NO_THROW(c.validate());
}
