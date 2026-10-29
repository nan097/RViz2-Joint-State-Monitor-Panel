#pragma once
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
// 只提供模拟的 RViz ROS 节点，面板测试不应依赖三维渲染器。
class TestContext : public rviz_common::DisplayContext {
public:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> abstraction;
  TestContext()
      : abstraction(
            std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
                "panel_test_host")) {}
  Ogre::SceneManager *getSceneManager() const override { return nullptr; }
  rviz_common::WindowManagerInterface *getWindowManager() const override {
    return nullptr;
  }
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const override {
    return {};
  }
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const override {
    return {};
  }
  std::shared_ptr<rviz_common::interaction::ViewPickerIface>
  getViewPicker() const override {
    return {};
  }
  rviz_common::FrameManagerIface *getFrameManager() const override {
    return nullptr;
  }
  QString getFixedFrame() const override { return "world"; }
  uint64_t getFrameCount() const override { return 0; }
  rviz_common::DisplayFactory *getDisplayFactory() const override {
    return nullptr;
  }
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const override {
    return abstraction;
  }
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}
  rviz_common::ToolManager *getToolManager() const override { return nullptr; }
  rviz_common::ViewManager *getViewManager() const override { return nullptr; }
  rviz_common::transformation::TransformationManager *
  getTransformationManager() override {
    return nullptr;
  }
  rviz_common::DisplayGroup *getRootDisplayGroup() const override {
    return nullptr;
  }
  uint32_t getDefaultVisibilityBit() const override { return 0; }
  rviz_common::BitAllocator *visibilityBits() override { return nullptr; }
  void setStatus(const QString &) override {}
  QString getHelpPath() const override { return {}; }
  std::shared_ptr<rclcpp::Clock> getClock() override {
    return abstraction->get_raw_node()->get_clock();
  }
  void lockRender() override {}
  void unlockRender() override {}
  void queueRender() override {}
};
