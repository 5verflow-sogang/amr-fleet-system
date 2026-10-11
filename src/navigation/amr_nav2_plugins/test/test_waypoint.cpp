#include <gtest/gtest.h>

#include "amr_nav2_plugins/waypoint.hpp"

namespace amr_nav2_plugins
{
namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

nav_msgs::msg::Path straightPath()
{
  nav_msgs::msg::Path path;
  for (double x = 0.0; x <= 10.0 + 1e-9; x += 0.5) {
    path.poses.push_back(makePose(x, 0.0));
  }
  return path;
}

}  // namespace

TEST(WaypointTest, EmptyPathReturnsRobotPose)
{
  const nav_msgs::msg::Path empty_path;
  const geometry_msgs::msg::PoseStamped robot = makePose(1.0, 2.0);
  const geometry_msgs::msg::PoseStamped waypoint = selectLookaheadWaypoint(empty_path, robot, 1.5);
  EXPECT_EQ(waypoint.pose.position.x, robot.pose.position.x);
  EXPECT_EQ(waypoint.pose.position.y, robot.pose.position.y);
}

TEST(WaypointTest, PicksPointAtLookaheadDistanceAhead)
{
  const nav_msgs::msg::Path path = straightPath();
  const geometry_msgs::msg::PoseStamped robot = makePose(2.0, 0.0);

  const geometry_msgs::msg::PoseStamped waypoint = selectLookaheadWaypoint(path, robot, 1.5);
  EXPECT_NEAR(waypoint.pose.position.x, 3.5, 1e-9);
}

TEST(WaypointTest, ShorterThanLookaheadReturnsLastPose)
{
  const nav_msgs::msg::Path path = straightPath();
  const geometry_msgs::msg::PoseStamped robot = makePose(9.5, 0.0);

  // 남은 경로(0.5m)가 lookahead(1.5m)보다 짧다 - 마지막 점(목표)을 돌려줘야 한다.
  const geometry_msgs::msg::PoseStamped waypoint = selectLookaheadWaypoint(path, robot, 1.5);
  EXPECT_NEAR(waypoint.pose.position.x, 10.0, 1e-9);
}

TEST(WaypointTest, IgnoresPastPointsBehindRobot)
{
  const nav_msgs::msg::Path path = straightPath();
  // 로봇이 경로 중간(5.0)에 있다 - 0.0~4.5의 지난 점들이 더 가깝다고 lookahead를
  // 뒤로 계산하면 안 된다.
  const geometry_msgs::msg::PoseStamped robot = makePose(5.0, 0.0);

  const geometry_msgs::msg::PoseStamped waypoint = selectLookaheadWaypoint(path, robot, 1.5);
  EXPECT_NEAR(waypoint.pose.position.x, 6.5, 1e-9);
}

}  // namespace amr_nav2_plugins
