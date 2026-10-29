#include "joint_state_rviz_panel/joint_row_widget.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>
#include <algorithm>
namespace jsm {
JointRowWidget::JointRowWidget(const std::string &name, QWidget *parent)
    : QWidget(parent) {
  setMinimumHeight(70);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  auto layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(2);
  auto label = new QLabel(QString::fromStdString(name));
  label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setStyleSheet("font-weight:bold");
  layout->addWidget(label);
  auto values = new QHBoxLayout;
  value_ = new QLabel("--");
  value_->setWordWrap(true);
  value_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  state_ = new QLabel;
  state_->setWordWrap(true);
  values->addWidget(value_, 1);
  values->addWidget(state_, 1);
  layout->addLayout(values);
  bar_ = new QProgressBar;
  bar_->setRange(0, 1000);
  bar_->setTextVisible(false);
  bar_->setFixedHeight(5);
  layout->addWidget(bar_);
}
void JointRowWidget::refresh(const Joint &j, const JointSpec &spec,
                             const RobotConfig &c, bool degrees, bool assume) {
  const bool rotation = spec.type == "revolute" || spec.type == "continuous" ||
                        (spec.type == "unknown" && assume);
  QString unit = rotation ? (degrees ? "°" : "rad")
                          : (spec.type == "prismatic" ? "m" : "raw");
  bar_->setVisible(j.received && spec.lower && spec.upper &&
                   spec.type != "continuous");
  if (!j.received) {
    value_->setText("--");
    state_->setText("等待数据");
    state_->setStyleSheet("color: #677788");
    return;
  }
  double value =
      j.position * (rotation && degrees ? 180.0 / 3.14159265358979323846 : 1.0);
  value_->setText(QString::number(value, 'f', 2) + " " + unit);
  double age =
      std::chrono::duration<double>(Clock::now() - *j.received).count();
  QString status = age >= c.timeout ? "数据超时"
                   : age >= c.warn  ? "更新迟缓"
                                    : "正常";
  QString color = age >= c.timeout ? "#c0392b"
                  : age >= c.warn  ? "#aa7600"
                                   : "#16804a";
  if (spec.lower && spec.upper) {
    double fraction = (j.position - *spec.lower) / (*spec.upper - *spec.lower);
    bar_->setValue(static_cast<int>(std::clamp(fraction, 0.0, 1.0) * 1000));
    if (fraction < 0 || fraction > 1)
      status += " · 超出限位";
  }
  if (spec.type == "unknown" && assume)
    status += " · 假定旋转";
  state_->setText(status);
  state_->setStyleSheet("color: " + color);
  value_->setToolTip(QString("原始值: %1\n类型: %2\n距有效更新: %3 ms")
                         .arg(j.position, 0, 'g', 15)
                         .arg(QString::fromStdString(spec.type))
                         .arg(age * 1000, 0, 'f', 0));
}
} // jsm 命名空间结束
