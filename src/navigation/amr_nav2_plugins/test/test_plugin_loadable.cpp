#include <gtest/gtest.h>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"

// configure()는 호출하지 않는다(실제 LifecycleNode·Costmap2DROS가 필요해서 무겁다).
// 여기서는 plugins.xml 내보내기 + CMake 라이브러리 이름이 맞아서 pluginlib이
// 클래스를 찾고 기본 생성자로 만들 수 있는지만 확인한다.
TEST(DwaControllerPluginTest, LoadableAsNav2CoreController)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");

  ASSERT_TRUE(loader.isClassAvailable("amr_nav2_plugins::DwaController"));

  std::shared_ptr<nav2_core::Controller> controller;
  ASSERT_NO_THROW(controller = loader.createSharedInstance("amr_nav2_plugins::DwaController"));
  EXPECT_NE(controller, nullptr);
}
