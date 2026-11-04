#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "amr_nav2_plugins/plan_path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_nav2_plugins
{
namespace
{

geometry_msgs::msg::PoseStamped goalPose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.orientation.w = std::cos(yaw / 2.0);
  return goal;
}

}  // namespace

TEST(PlanPathTest, EmptyPointsGiveEmptyPathWithHeader)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  const nav_msgs::msg::Path path = toPlanPath({}, 0.0, 0.0, 0.05, "map", stamp, goalPose(1, 1, 0));
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.header.stamp.sec, 7);
}

TEST(PlanPathTest, CellCentersMapToWorldAndHeadingFollowsThePath)
{
  // 원점 (-30.1, -21.6), 0.025 m (지도 v2 0.025 m와 같은 꼴). 셀 (0,0)의 중심은 원점 + 반 셀
  const std::vector<amr_planning_core::Point2D> points = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}};
  const nav_msgs::msg::Path path =
    toPlanPath(points, -30.1, -21.6, 0.025, "map", builtin_interfaces::msg::Time(), goalPose(-29.99, -21.4, 1.0));

  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_NEAR(path.poses[0].pose.position.x, -30.1 + 0.0125, 1e-9);
  EXPECT_NEAR(path.poses[0].pose.position.y, -21.6 + 0.0125, 1e-9);
  EXPECT_NEAR(path.poses[1].pose.position.x, -30.1 + 4.5 * 0.025, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses[0].pose.orientation), 0.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses[1].pose.orientation), M_PI / 2.0, 1e-9);
  EXPECT_EQ(path.poses[0].header.frame_id, "map");
}

TEST(PlanPathTest, LastPoseIsTheRequestedGoal)
{
  const std::vector<amr_planning_core::Point2D> points = {{0.0, 0.0}, {1.0, 1.0}};
  const geometry_msgs::msg::PoseStamped goal = goalPose(0.0612, 0.0588, -2.0);
  const nav_msgs::msg::Path path =
    toPlanPath(points, 0.0, 0.0, 0.05, "map", builtin_interfaces::msg::Time(), goal);

  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 0.0612);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, 0.0588);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), -2.0, 1e-9);
}

}  // namespace amr_nav2_plugins
