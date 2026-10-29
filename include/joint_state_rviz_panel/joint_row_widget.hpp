#pragma once
#include "joint_data_manager.hpp"
#include "robot_config.hpp"
#include <QWidget>
class QLabel;
class QProgressBar;
namespace jsm {
class JointRowWidget : public QWidget {
  QLabel *value_, *state_;
  QProgressBar *bar_;

public:
  explicit JointRowWidget(const std::string &name, QWidget *parent = nullptr);
  void refresh(const Joint &, const JointSpec &, const RobotConfig &,
               bool degrees, bool assume);
};
} // jsm 命名空间结束
