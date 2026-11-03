#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

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

TEST(ObstaclesTest, MinDynamicTtcIgnoresStaticTracks)
{
  // CollisionRisk에는 is_dynamic이 없다 - perception/obstacles의 track_id로 짝지어 움직이는 트랙만 본다
  amr_msgs::msg::TrackedObstacleArray tracks;
  tracks.obstacles.push_back(makeTrack(1.0, 2.0, false));  // 랙 면 같은 정적 트랙
  tracks.obstacles.back().track_id = 1;
  tracks.obstacles.push_back(makeTrack(3.0, 4.0, true));
  tracks.obstacles.back().track_id = 2;
  tracks.obstacles.push_back(makeTrack(5.0, 6.0, true));
  tracks.obstacles.back().track_id = 3;
  const auto ids = dynamicTrackIds(tracks);
  ASSERT_EQ(ids.size(), 2u);

  amr_msgs::msg::CollisionRiskArray risks;
  auto add = [&](int32_t id, float ttc) {
      amr_msgs::msg::CollisionRisk risk;
      risk.track_id = id;
      risk.ttc = ttc;
      risks.risks.push_back(risk);
    };
  add(1, 0.5f);                                        // 정적 - 무시
  add(2, std::numeric_limits<float>::infinity());
  add(3, 2.5f);
  add(7, 0.1f);                                        // perception/obstacles에 없는 트랙 - 무시
  EXPECT_DOUBLE_EQ(minDynamicTtc(risks, ids), 2.5);

  risks.risks.pop_back();
  risks.risks.pop_back();
  add(3, std::numeric_limits<float>::quiet_NaN());     // NaN - 무시
  EXPECT_TRUE(std::isinf(minDynamicTtc(risks, ids)));
  EXPECT_TRUE(std::isinf(minDynamicTtc(amr_msgs::msg::CollisionRiskArray(), ids)));
}

}  // namespace amr_nav2_plugins
