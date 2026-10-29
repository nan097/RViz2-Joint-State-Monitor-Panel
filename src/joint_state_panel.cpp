#include "joint_state_rviz_panel/joint_state_panel.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <set>

namespace joint_state_rviz_panel {
namespace {
std::vector<std::string> namesFromText(const QString &text) {
  std::vector<std::string> names;
  std::set<std::string> seen;
  if (text.trimmed().isEmpty())
    return names;
  for (const auto &item : text.split(',')) {
    auto name = item.trimmed().toStdString();
    if (name.empty() || !seen.insert(name).second)
      throw std::runtime_error("关节名称不能为空或重复；请用逗号分隔。");
    names.push_back(name);
  }
  return names;
}
QString namesText(const std::vector<std::string> &names) {
  QStringList list;
  for (const auto &name : names)
    list.push_back(QString::fromStdString(name));
  return list.join(", ");
}
} // 匿名命名空间结束
JointStatePanel::JointStatePanel(QWidget *parent)
    : rviz_common::Panel(parent),
      data_(std::make_shared<jsm::JointDataManager>()) {
  setObjectName("JointStatePanel");
  auto layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(5);
  auto title = new QLabel("Joint State Monitor");
  title->setStyleSheet("font-size:16px;font-weight:bold");
  layout->addWidget(title);
  auto topic_row = new QHBoxLayout;
  topic_ = new QLineEdit;
  topic_->setObjectName("Topic");
  topic_->setMinimumWidth(80);
  topic_->setToolTip("sensor_msgs/msg/JointState 话题");
  auto apply_button = new QPushButton("应用");
  apply_button->setObjectName("Apply");
  topic_row->addWidget(topic_, 1);
  topic_row->addWidget(apply_button);
  layout->addLayout(topic_row);
  auto options = new QHBoxLayout;
  unit_ = new QComboBox;
  unit_->setObjectName("Unit");
  unit_->addItems({"deg", "rad"});
  qos_ = new QComboBox;
  qos_->setObjectName("QoS");
  qos_->addItems({"best_effort", "reliable"});
  qos_->setToolTip("订阅 Reliability");
  options->addWidget(new QLabel("旋转单位"));
  options->addWidget(unit_);
  options->addWidget(qos_, 1);
  layout->addLayout(options);
  auto filter_row = new QHBoxLayout;
  mode_ = new QComboBox;
  mode_->setObjectName("FilterMode");
  mode_->addItems({"自动全部", "指定过滤"});
  auto yaml = new QPushButton("加载 YAML");
  yaml->setObjectName("LoadYaml");
  filter_row->addWidget(mode_, 1);
  filter_row->addWidget(yaml);
  layout->addLayout(filter_row);
  profile_ = new QComboBox;
  profile_->setObjectName("Profile");
  layout->addWidget(profile_);
  joint_names_ = new QLineEdit;
  joint_names_->setObjectName("JointNames");
  joint_names_->setPlaceholderText("关节名称，逗号分隔；按此顺序显示");
  layout->addWidget(joint_names_);
  assume_ = new QCheckBox("未知类型按旋转关节显示（推断）");
  assume_->setObjectName("AssumeUnknownRevolute");
  layout->addWidget(assume_);
  active_topic_ = new QLabel;
  active_topic_->setWordWrap(true);
  active_topic_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(active_topic_);
  status_ = new QLabel;
  status_->setObjectName("DataStatus");
  layout->addWidget(status_);
  statistics_ = new QLabel;
  statistics_->setObjectName("Statistics");
  statistics_->setWordWrap(true);
  layout->addWidget(statistics_);
  auto scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setMinimumHeight(80);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto content = new QWidget;
  rows_layout_ = new QVBoxLayout(content);
  rows_layout_->setContentsMargins(0, 0, 0, 0);
  rows_layout_->setSpacing(3);
  rows_layout_->setAlignment(Qt::AlignTop);
  rows_layout_->setSizeConstraint(QLayout::SetMinAndMaxSize);
  scroll->setWidget(content);
  layout->addWidget(scroll, 1);
  diagnostics_ = new QLabel;
  diagnostics_->setObjectName("Diagnostics");
  diagnostics_->setWordWrap(true);
  layout->addWidget(diagnostics_);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &JointStatePanel::refresh);
  connect(apply_button, &QPushButton::clicked, this, &JointStatePanel::apply);
  connect(topic_, &QLineEdit::returnPressed, this, &JointStatePanel::apply);
  connect(joint_names_, &QLineEdit::returnPressed, this,
          &JointStatePanel::apply);
  connect(mode_, qOverload<int>(&QComboBox::activated), this,
          [this](int) { apply(); });
  connect(qos_, qOverload<int>(&QComboBox::activated), this,
          [this](int) { apply(); });
  connect(unit_, qOverload<int>(&QComboBox::activated), this, [this](int) {
    settings_.unit = unit_->currentText().toStdString();
    refresh();
    Q_EMIT configChanged();
  });
  connect(assume_, &QCheckBox::clicked, this, [this](bool checked) {
    settings_.assume = checked;
    refresh();
    Q_EMIT configChanged();
  });
  connect(profile_, qOverload<int>(&QComboBox::activated), this, [this](int) {
    joint_names_->setText(namesText(
        settings_.profiles.at(profile_->currentData().toString().toStdString())
            .names));
    apply();
  });
  connect(yaml, &QPushButton::clicked, this, [this] {
    auto file = QFileDialog::getOpenFileName(this, "选择关节配置", {},
                                             "YAML (*.yaml *.yml)");
    if (file.isEmpty())
      return;
    try {
      commit(jsm::RobotConfig::load(file.toStdString()));
      Q_EMIT configChanged();
    } catch (const std::exception &e) {
      showError(QString::fromUtf8(e.what()));
    }
  });
  updateControls();
  refresh();
  timer_->start(50);
}
JointStatePanel::~JointStatePanel() {
  timer_->stop();
  // 排队回调只持有自己的缓存，不持有面板或任何界面控件。
  subscription_.reset();
  notifyHost();
  data_.reset();
  node_.reset();
}
void JointStatePanel::notifyHost() noexcept {
  if (!node_)
    return;
  try {
    // Humble 执行器可能在阻塞的等待集合中暂时持有订阅。
    // 唤醒宿主执行器，使空闲话题上被移除的订阅也能及时释放。
    node_->get_node_base_interface()->get_notify_guard_condition().trigger();
  } catch (const std::exception &) {
    // RViz 上下文可能已经退出，析构过程中不能继续抛出异常。
  }
}
void JointStatePanel::onInitialize() {
  auto context = getDisplayContext();
  if (!context) {
    showError("RViz DisplayContext 不可用");
    return;
  }
  auto abstraction = context->getRosNodeAbstraction().lock();
  if (!abstraction) {
    showError("RViz ROS 节点不可用");
    return;
  }
  node_ = abstraction->get_raw_node();
  if (!node_) {
    showError("RViz ROS 节点为空");
    return;
  }
  try {
    commit(settings_);
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
void JointStatePanel::showError(const QString &text) {
  error_ = text;
  refresh();
}
void JointStatePanel::clearRows() {
  for (auto &entry : rows_)
    delete entry.second;
  rows_.clear();
}
void JointStatePanel::updateControls() {
  const QSignalBlocker b1(topic_), b2(mode_), b3(profile_), b4(unit_), b5(qos_),
      b6(assume_), b7(joint_names_);
  topic_->setText(QString::fromStdString(settings_.topic));
  mode_->setCurrentIndex(settings_.mode == "configured" ? 1 : 0);
  unit_->setCurrentText(QString::fromStdString(settings_.unit));
  qos_->setCurrentText(QString::fromStdString(settings_.reliability));
  assume_->setChecked(settings_.assume);
  profile_->clear();
  for (const auto &p : settings_.profiles)
    profile_->addItem(QString::fromStdString(p.second.display_name),
                      QString::fromStdString(p.first));
  profile_->setCurrentIndex(
      profile_->findData(QString::fromStdString(settings_.profile)));
  joint_names_->setText(
      namesText(settings_.profiles.at(settings_.profile).names));
  joint_names_->setVisible(settings_.mode == "configured");
}
void JointStatePanel::commit(const jsm::RobotConfig &settings) {
  settings.validate();
  auto cache = std::make_shared<jsm::JointDataManager>();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr replacement;
  if (node_) {
    auto qos = rclcpp::SensorDataQoS();
    if (settings.reliability == "reliable")
      qos.reliable();
    replacement = node_->create_subscription<sensor_msgs::msg::JointState>(
        settings.topic, qos,
        [cache](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          cache->update(*msg);
        });
  }
  // 先创建新订阅，话题非法时保留原订阅。
  // 旧回调只会写入已经被替换的独立缓存，不会污染新话题数据。
  subscription_ = std::move(replacement);
  notifyHost();
  data_ = std::move(cache);
  settings_ = settings;
  error_.clear();
  publishers_ = 0;
  graph_checked_ = {};
  clearRows();
  updateControls();
  timer_->setInterval(qRound(1000 / settings_.refresh_hz));
  refresh();
}
void JointStatePanel::apply() {
  try {
    auto next = settings_;
    next.topic = topic_->text().trimmed().toStdString();
    next.mode = mode_->currentIndex() == 1 ? "configured" : "auto";
    next.profile = profile_->currentData().toString().toStdString();
    next.unit = unit_->currentText().toStdString();
    next.reliability = qos_->currentText().toStdString();
    next.assume = assume_->isChecked();
    next.profiles.at(next.profile).names = namesFromText(joint_names_->text());
    commit(next);
    Q_EMIT configChanged();
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
void JointStatePanel::refresh() {
  if (!data_)
    return;
  const auto now = jsm::Clock::now();
  auto snapshot = data_->snapshot(now);
  const auto &profile = settings_.profiles.at(settings_.profile);
  std::map<std::string, jsm::Joint> lookup;
  for (const auto &j : snapshot.joints)
    lookup[j.name] = j;
  auto names = profile.names;
  if (settings_.mode == "auto") {
    names.clear();
    for (const auto &j : snapshot.joints)
      names.push_back(j.name);
  }
  for (const auto &name : names) {
    if (!rows_.count(name)) {
      auto row = new jsm::JointRowWidget(name);
      rows_layout_->addWidget(row);
      rows_[name] = row;
    }
    jsm::Joint joint;
    joint.name = name;
    if (lookup.count(name))
      joint = lookup.at(name);
    jsm::JointSpec spec;
    if (profile.joints.count(name))
      spec = profile.joints.at(name);
    rows_.at(name)->refresh(joint, spec, settings_, settings_.unit == "deg",
                            settings_.assume);
  }
  if (node_ && subscription_ &&
      now - graph_checked_ >= std::chrono::milliseconds(500)) {
    graph_checked_ = now;
    try {
      publishers_ = node_->count_publishers(subscription_->get_topic_name());
    } catch (const std::exception &e) {
      error_ = QString::fromUtf8(e.what());
    }
  }
  const double age =
      snapshot.last_valid
          ? std::chrono::duration<double>(now - *snapshot.last_valid).count()
          : -1;
  const QString state = !error_.isEmpty()          ? "订阅 / 配置错误"
                        : !node_                   ? "等待 RViz ROS 节点"
                        : age < 0                  ? "等待有效数据"
                        : age >= settings_.timeout ? "数据超时"
                        : age >= settings_.warn    ? "更新迟缓"
                                                   : "正常更新";
  status_->setText("● " + state);
  status_->setStyleSheet(!error_.isEmpty() || age >= settings_.timeout
                             ? "color:#c0392b"
                         : age >= settings_.warn ? "color:#aa7600"
                         : age < 0               ? "color:#778899"
                                                 : "color:#16804a");
  const QString topic =
      subscription_ ? QString::fromStdString(subscription_->get_topic_name())
                    : QString::fromStdString(settings_.topic);
  active_topic_->setText("当前话题: " + topic);
  statistics_->setText(
      QString("%1 Hz · 显示 %2 / 已识别 %3 关节\n发布者: %4 · 最近有效更新: %5")
          .arg(snapshot.hz, 0, 'f', 1)
          .arg(names.size())
          .arg(snapshot.joints.size())
          .arg(publishers_)
          .arg(age < 0 ? "--" : QString::number(age * 1000, 'f', 0) + " ms"));
  QString diagnostic = error_;
  if (node_ && publishers_ == 0)
    diagnostic += " 未发现发布者。";
  if (publishers_ > 1)
    diagnostic += " 多个发布者：请用命名空间隔离机器人。";
  if (settings_.mode == "configured" && names.empty())
    diagnostic += " 过滤列表为空。";
  if (snapshot.warnings)
    diagnostic += QString(" 数据警告 %1 次: %2")
                      .arg(snapshot.warnings)
                      .arg(QString::fromStdString(snapshot.diagnostic));
  diagnostics_->setText(diagnostic.trimmed());
}
void JointStatePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", QString::fromStdString(settings_.topic));
  config.mapSetValue("Unit", QString::fromStdString(settings_.unit));
  config.mapSetValue("FilterMode", QString::fromStdString(settings_.mode));
  config.mapSetValue("Reliability",
                     QString::fromStdString(settings_.reliability));
  config.mapSetValue("AssumeUnknownRevolute", settings_.assume);
  config.mapSetValue("Profile", QString::fromStdString(settings_.profile));
  auto list = config.mapMakeChild("JointNames");
  list.setType(rviz_common::Config::List);
  for (const auto &name : settings_.profiles.at(settings_.profile).names)
    list.listAppendNew().setValue(QString::fromStdString(name));
  // 将关节定义、限位和超时阈值内嵌到 RViz 配置，避免依赖外部 YAML 路径。
  config.mapSetValue("RobotSettings",
                     QString::fromStdString(settings_.toYaml()));
}
void JointStatePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  try {
    jsm::RobotConfig next;
    QString text;
    if (config.mapGetString("RobotSettings", &text))
      next = jsm::RobotConfig::fromYaml(text.toStdString());
    if (config.mapGetString("Topic", &text))
      next.topic = text.toStdString();
    if (config.mapGetString("Unit", &text))
      next.unit = text.toStdString();
    if (config.mapGetString("FilterMode", &text))
      next.mode = text.toStdString();
    if (config.mapGetString("Reliability", &text))
      next.reliability = text.toStdString();
    if (config.mapGetString("Profile", &text))
      next.profile = text.toStdString();
    bool assume;
    if (config.mapGetBool("AssumeUnknownRevolute", &assume))
      next.assume = assume;
    if (!next.profiles.count(next.profile))
      throw std::runtime_error("Unknown saved profile");
    auto list = config.mapGetChild("JointNames");
    if (list.isValid()) {
      if (list.getType() != rviz_common::Config::List &&
          list.getType() != rviz_common::Config::Empty)
        throw std::runtime_error("JointNames must be a list");
      auto &names = next.profiles.at(next.profile).names;
      names.clear();
      for (int i = 0; i < list.listLength(); ++i)
        names.push_back(
            list.listChildAt(i).getValue().toString().toStdString());
    }
    // RViz 可能先加载配置，也可能先初始化；两种顺序都需要兼容。
    commit(next);
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
} // joint_state_rviz_panel 命名空间结束
PLUGINLIB_EXPORT_CLASS(joint_state_rviz_panel::JointStatePanel,
                       rviz_common::Panel)
