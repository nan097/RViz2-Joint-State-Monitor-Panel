// 仅用于测试真实 RViz 窗口、渲染和宿主执行器。
// 此程序不安装为产品入口，也不链接进面板插件。
#include "joint_state_rviz_panel/joint_state_panel.hpp"
#include <QApplication>
#include <QDockWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPointer>
#include <QTimer>
#include <iostream>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>

using joint_state_rviz_panel::JointStatePanel;
static bool contains(QWidget *widget, const QString &text) {
  for (auto label : widget->findChildren<QLabel *>())
    if (label->text().contains(text))
      return true;
  return false;
}
int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: rviz_host_smoke path/to/example.rviz\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto abstraction =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
          "panel_real_rviz_host");
  auto node = abstraction->get_raw_node();
  int result = 1;
  {
    rviz_common::VisualizationFrame frame(abstraction);
    frame.setApp(&app);
    frame.setSplashPath("");
    frame.initialize(abstraction, QString::fromLocal8Bit(argv[1]));
    frame.getManager()->startUpdate();
    frame.resize(1200, 850);
    frame.show();
    auto panel = frame.findChild<JointStatePanel *>();
    if (!panel) {
      std::cerr << "Panel was not loaded by RViz\n";
      return 3;
    }
    QWidget *ancestor = panel->parentWidget();
    while (ancestor && !qobject_cast<QDockWidget *>(ancestor))
      ancestor = ancestor->parentWidget();
    auto dock = qobject_cast<QDockWidget *>(ancestor);
    if (!dock) {
      std::cerr << "Panel is not dockable\n";
      return 4;
    }
    dock->setFloating(false);
    frame.addDockWidget(Qt::RightDockWidgetArea, dock);
    frame.resizeDocks({dock}, {320}, Qt::Horizontal);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS());
    sensor_msgs::msg::JointState message;
    for (int i = 0; i < 14; ++i) {
      message.name.push_back("joint" + std::to_string(i + 1));
      message.position.push_back(i / 10.0);
    }
    QTimer publish;
    QObject::connect(&publish, &QTimer::timeout,
                     [&] { publisher->publish(message); });
    publish.start(20);
    QTimer ready;
    QObject::connect(&ready, &QTimer::timeout, &app, [&] {
      auto current = frame.findChild<JointStatePanel *>();
      if (!current || !contains(current, "正常更新") ||
          !contains(current, "joint14") ||
          !current->findChild<QLabel *>("Statistics")
               ->text()
               .contains("发布者: 1"))
        return;
      ready.stop();
      std::vector<jsm::JointRowWidget *> rows;
      for (auto widget : current->findChildren<QWidget *>())
        if (auto row = dynamic_cast<jsm::JointRowWidget *>(widget))
          rows.push_back(row);
      if (rows.size() != 14) {
        std::cerr << "Wrong row count\n";
        app.quit();
        return;
      }
      for (size_t i = 1; i < rows.size(); ++i)
        if (rows[i]->y() <= rows[i - 1]->geometry().bottom()) {
          std::cerr << "Rows overlap\n";
          app.quit();
          return;
        }
      if (frame.dockWidgetArea(dock) != Qt::RightDockWidgetArea) {
        std::cerr << "Wrong dock area\n";
        app.quit();
        return;
      }
      if (!frame.saveDisplayConfig("/tmp/joint-panel-runtime.rviz")) {
        std::cerr << "Save failed\n";
        app.quit();
        return;
      }
      if (!frame.grab().save("/tmp/joint-panel-rviz-internal.png")) {
        app.quit();
        return;
      }
      frame.loadDisplayConfig("/tmp/joint-panel-runtime.rviz");
      QTimer::singleShot(500, &app, [&] {
        auto restored = frame.findChildren<JointStatePanel *>();
        if (restored.size() != 1 || !contains(restored[0], "正常更新") ||
            !contains(restored[0], "joint14")) {
          std::cerr << "Reload failed\n";
          app.quit();
          return;
        }
        if (frame.getManager()->getRootDisplayGroup()->numDisplays() != 1) {
          std::cerr << "Grid was lost\n";
          app.quit();
          return;
        }
        std::cout << "PASS: real RViz host delivers 14 joints, right dock, "
                     "non-overlap, save/reload, Grid preserved\n";
        result = 0;
        app.quit();
      });
    });
    ready.start(300);
    QTimer::singleShot(30000, &app, &QApplication::quit);
    app.exec();
    publish.stop();
  }
  rclcpp::shutdown();
  return result;
}
