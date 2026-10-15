#include <gtest/gtest.h>

#include "amr_nav2_plugins/obstacles.hpp"

namespace amr_nav2_plugins
{
namespace
{

amr_msgs::msg::TrackedObstacle makeTrack(double x, double y, bool is_dynamic)
{
  amr_msgs::msg::TrackedObstacle track;
  track.pose.pose.position.x = x;
  track.pose.pose.position.y = y;
  track.twist.twist.linear.x = 0.5;
  track.twist.twist.linear.y = -0.2;
  track.radius = 0.3f;
  track.is_dynamic = is_dynamic;
  return track;
}

}  // namespace

TEST(ObstaclesTest, KeepsOnlyDynamicTracks)
{
  amr_msgs::msg::TrackedObstacleArray msg;
  msg.obstacles.push_back(makeTrack(1.0, 2.0, false));  // 랙 모서리 같은 정적 트랙
  msg.obstacles.push_back(makeTrack(3.0, 4.0, true));

  const auto obstacles = toDynamicObstacles(msg);

  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(obstacles[0].x, 3.0);
  EXPECT_DOUBLE_EQ(obstacles[0].y, 4.0);
  EXPECT_DOUBLE_EQ(obstacles[0].vx, 0.5);
  EXPECT_DOUBLE_EQ(obstacles[0].vy, -0.2);
  EXPECT_FLOAT_EQ(static_cast<float>(obstacles[0].radius), 0.3f);
}

TEST(ObstaclesTest, AllStaticGivesNoObstacles)
{
  amr_msgs::msg::TrackedObstacleArray msg;
  msg.obstacles.push_back(makeTrack(1.0, 2.0, false));
  msg.obstacles.push_back(makeTrack(5.0, 6.0, false));

  EXPECT_TRUE(toDynamicObstacles(msg).empty());
}

}  // namespace amr_nav2_plugins
