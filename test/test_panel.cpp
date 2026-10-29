#include "joint_state_rviz_panel/joint_state_panel.hpp"
#include "test_context.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/yaml_config_writer.hpp>
#include <thread>
using joint_state_rviz_panel::JointStatePanel;
using namespace jsm;
static void pump(int ms = 70) {
  auto end = Clock::now() + std::chrono::milliseconds(ms);
  while (Clock::now() < end) {
    QApplication::processEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
}
static bool textContains(QWidget &w, const QString &text) {
  for (auto label : w.findChildren<QLabel *>())
    if (label->text().contains(text))
      return true;
  return false;
}
static std::vector<JointRowWidget *> rows(QWidget &w) {
  std::vector<JointRowWidget *> result;
  for (auto widget : w.findChildren<QWidget *>())
    if (auto row = dynamic_cast<JointRowWidget *>(widget))
      result.push_back(row);
  return result;
}
struct HostExecutor {
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::atomic<bool> running{true};
  explicit HostExecutor(const rclcpp::Node::SharedPtr &node) {
    executor.add_node(node);
    // 模拟 RViz 对宿主执行器的周期调用。
    // Humble 中持续阻塞的 spin() 可能在新回调就绪前保留上一次执行对象。
    thread = std::thread([this] {
      while (running)
        executor.spin_once(std::chrono::milliseconds(10));
    });
  }
  ~HostExecutor() {
    running = false;
    executor.cancel();
    thread.join();
  }
};
static rviz_common::Config settings(const QString &topic) {
  rviz_common::Config config;
  config.mapSetValue("Topic", topic);
  config.mapSetValue("Unit", "deg");
  return config;
}
TEST(Plugin, DiscoverAndLoadByExportedClass) {
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common",
                                                    "rviz_common::Panel");
  ASSERT_TRUE(
      loader.isClassAvailable("joint_state_rviz_panel/JointStatePanel"));
  auto panel =
      loader.createSharedInstance("joint_state_rviz_panel/JointStatePanel");
  EXPECT_NE(dynamic_cast<JointStatePanel *>(panel.get()), nullptr);
}
TEST(Panel, EmptyNamesFromRvizYaml) {
  rviz_common::YamlConfigReader reader;
  rviz_common::Config config;
  reader.readString(config,
                    "Topic: /joint_states\nJointNames: []\nFilterMode: auto\n");
  ASSERT_FALSE(reader.error());
  JointStatePanel panel;
  panel.load(config);
  EXPECT_FALSE(textContains(panel, "错误"));
  rviz_common::Config saved;
  panel.save(saved);
  rviz_common::YamlConfigWriter writer;
  reader.readString(config, writer.writeString(saved));
  panel.load(config);
  EXPECT_FALSE(textContains(panel, "错误"));
}
TEST(Panel, SaveLoadRoundTripBeforeAndAfterInitialize) {
  TestContext context;
  auto node = context.abstraction->get_raw_node();
  HostExecutor executor(node);
  JointStatePanel first;
  first.setClassId("joint_state_rviz_panel/JointStatePanel");
  first.setName("Joint Monitor");
  auto config = settings("/panel_test/restore");
  config.mapSetValue("Unit", "rad");
  config.mapSetValue("FilterMode", "configured");
  config.mapSetValue("Reliability", "reliable");
  config.mapSetValue("AssumeUnknownRevolute", true);
  auto list = config.mapMakeChild("JointNames");
  list.setType(rviz_common::Config::List);
  for (auto name : {"missing", "b", "a"})
    list.listAppendNew().setValue(name);
  first.load(config);
  first.initialize(&context);
  pump();
  EXPECT_EQ(node->count_subscribers("/panel_test/restore"), 1u);
  rviz_common::Config saved;
  first.save(saved);
  rviz_common::YamlConfigWriter writer;
  auto yaml = writer.writeString(saved);
  ASSERT_FALSE(writer.error());
  rviz_common::YamlConfigReader reader;
  rviz_common::Config restored;
  reader.readString(restored, yaml);
  ASSERT_FALSE(reader.error());
  JointStatePanel second;
  second.initialize(&context);
  second.load(restored);
  second.resize(300, 750);
  second.show();
  pump();
  EXPECT_EQ(second.findChild<QLineEdit *>("Topic")->text(),
            "/panel_test/restore");
  EXPECT_EQ(second.findChild<QComboBox *>("Unit")->currentText(), "rad");
  EXPECT_EQ(second.findChild<QComboBox *>("QoS")->currentText(), "reliable");
  EXPECT_TRUE(
      second.findChild<QCheckBox *>("AssumeUnknownRevolute")->isChecked());
  auto displayed = rows(second);
  ASSERT_EQ(displayed.size(), 3u);
  EXPECT_TRUE(textContains(*displayed[0], "missing"));
  EXPECT_TRUE(textContains(*displayed[0], "--"));
  EXPECT_TRUE(textContains(*displayed[1], "b"));
  EXPECT_TRUE(textContains(*displayed[2], "a"));
  EXPECT_LE(second.minimumSizeHint().width(), 300);
  EXPECT_TRUE(second.grab().save("/tmp/joint-panel-configured.png"));
}
TEST(Panel, HostNodeDeliverySwitchTimeoutAndRepeatedRemoval) {
  TestContext context;
  auto node = context.abstraction->get_raw_node();
  HostExecutor executor(node);
  auto panel = std::make_unique<JointStatePanel>();
  auto config = settings("/panel_test/first");
  RobotConfig definitions;
  definitions.topic = "/panel_test/first";
  definitions.warn = .15;
  definitions.timeout = .3;
  definitions.profiles["generic"].joints["joint0"] = {"revolute", -3.14, 3.14};
  definitions.profiles["generic"].joints["joint1"] = {"prismatic", {}, {}};
  config.mapSetValue("RobotSettings",
                     QString::fromStdString(definitions.toYaml()));
  panel->load(config);
  panel->initialize(&context);
  panel->resize(300, 750);
  panel->show();
  pump();
  EXPECT_TRUE(textContains(*panel, "等待有效数据"));
  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(
      "/panel_test/first", rclcpp::SensorDataQoS());
  sensor_msgs::msg::JointState msg;
  auto send = [&](const auto &pub, const auto &message,
                  const QString &expected) {
    for (int i = 0; i < 80; ++i) {
      pub->publish(message);
      pump(20);
      if (textContains(*panel, expected))
        return true;
    }
    return false;
  };
  for (int count : {6, 7, 14}) {
    msg.name.clear();
    msg.position.clear();
    for (int i = 0; i < count; ++i) {
      msg.name.push_back("joint" + std::to_string(i));
      msg.position.push_back(i == 0 ? 1.5707963267948966 : i);
    }
    ASSERT_TRUE(send(publisher, msg, "joint" + QString::number(count - 1)));
    EXPECT_EQ(rows(*panel).size(), static_cast<size_t>(count));
  }
  pump(); // 添加最后几行后，等待 Qt 完成排队的布局更新。
  EXPECT_TRUE(textContains(*panel, "90.00 °"));
  EXPECT_TRUE(textContains(*panel, "1.00 m"));
  ASSERT_FALSE(rows(*panel).empty());
  EXPECT_GE(rows(*panel)[0]->height(), 60);
  auto visible_rows = rows(*panel);
  for (size_t i = 1; i < visible_rows.size(); ++i)
    EXPECT_GT(visible_rows[i]->y(), visible_rows[i - 1]->geometry().bottom());
  panel->grab().save("/tmp/joint-panel-live.png");
  std::reverse(msg.name.begin(), msg.name.end());
  std::reverse(msg.position.begin(), msg.position.end());
  ASSERT_TRUE(send(publisher, msg, "正常更新"));
  auto displayed = rows(*panel);
  EXPECT_TRUE(textContains(*displayed[0], "joint0"));
  EXPECT_TRUE(textContains(*displayed[0], "90.00 °"));
  pump(400);
  EXPECT_TRUE(textContains(*panel, "数据超时"));
  ASSERT_TRUE(send(publisher, msg, "正常更新"));
  panel->findChild<QLineEdit *>("Topic")->setText("/panel_test/second");
  panel->findChild<QPushButton *>("Apply")->click();
  EXPECT_TRUE(rows(*panel).empty());
  publisher->publish(msg);
  pump(100);
  EXPECT_TRUE(rows(*panel).empty());
  auto next = node->create_publisher<sensor_msgs::msg::JointState>(
      "/panel_test/second", rclcpp::SensorDataQoS());
  msg.name = {"new_joint"};
  msg.position = {2};
  ASSERT_TRUE(send(next, msg, "new_joint"));
  EXPECT_EQ(rows(*panel).size(), 1u);
  // 非法输入不能破坏当前订阅和最近有效值。
  panel->findChild<QLineEdit *>("Topic")->setText("/bad topic");
  panel->findChild<QPushButton *>("Apply")->click();
  EXPECT_TRUE(textContains(*panel, "错误"));
  EXPECT_TRUE(textContains(*panel, "new_joint"));
  panel.reset();
  pump();
  EXPECT_EQ(node->count_subscribers("/panel_test/second"), 0u);
  // 在持续发布期间反复销毁面板，验证排队回调的生命周期安全。
  std::atomic<bool> publishing{true};
  std::thread stream([&] {
    while (publishing) {
      next->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  for (int i = 0; i < 10; ++i) {
    auto temporary = std::make_unique<JointStatePanel>();
    temporary->load(settings("/panel_test/second"));
    temporary->initialize(&context);
    pump(20);
    temporary.reset();
  }
  publishing = false;
  stream.join();
  pump();
  EXPECT_EQ(node->count_subscribers("/panel_test/second"), 0u);
}
TEST(Widgets, UnitsAndNoFabricatedLimits) {
  JointRowWidget row("test");
  RobotConfig c;
  Joint joint{"test", 1.5707963267948966, Clock::now()};
  row.refresh(joint, {"revolute", {}, {}}, c, true, false);
  EXPECT_TRUE(textContains(row, "90.00 °"));
  row.refresh(joint, {"revolute", {}, {}}, c, false, false);
  EXPECT_TRUE(textContains(row, "1.57 rad"));
  row.refresh(joint, {"prismatic", {}, {}}, c, true, false);
  EXPECT_TRUE(textContains(row, "1.57 m"));
  row.refresh(joint, {}, c, true, false);
  EXPECT_TRUE(textContains(row, "1.57 raw"));
  joint.received.reset();
  row.refresh(joint, {}, c, true, false);
  EXPECT_TRUE(textContains(row, "--"));
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
