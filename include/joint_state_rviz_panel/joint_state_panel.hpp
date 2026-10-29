#pragma once
#include "joint_data_manager.hpp"
#include "joint_row_widget.hpp"
#include "robot_config.hpp"
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QLineEdit;
class QComboBox;
class QCheckBox;
class QLabel;
class QVBoxLayout;
class QTimer;

namespace joint_state_rviz_panel {
class JointStatePanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit JointStatePanel(QWidget *parent = nullptr);
  ~JointStatePanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

private:
  jsm::RobotConfig settings_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::shared_ptr<jsm::JointDataManager> data_;
  QLineEdit *topic_, *joint_names_;
  QComboBox *mode_, *profile_, *unit_, *qos_;
  QCheckBox *assume_;
  QLabel *status_, *statistics_, *diagnostics_, *active_topic_;
  QVBoxLayout *rows_layout_;
  QTimer *timer_;
  std::map<std::string, jsm::JointRowWidget *> rows_;
  QString error_;
  size_t publishers_ = 0;
  jsm::Clock::time_point graph_checked_{};
  void apply();
  void commit(const jsm::RobotConfig &settings);
  void updateControls();
  void refresh();
  void clearRows();
  void notifyHost() noexcept;
  void showError(const QString &text);
};
} // joint_state_rviz_panel 命名空间结束
