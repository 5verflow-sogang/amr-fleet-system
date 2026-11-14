#include <gtest/gtest.h>

#include <cmath>

#include "amr_nav2_plugins/fleet_peers.hpp"

using amr_nav2_plugins::PeerPose;
using amr_nav2_plugins::PeerTracker;
using amr_nav2_plugins::footprintCorners;
using amr_nav2_plugins::pointInPolygon;
using amr_nav2_plugins::robotIdFromNamespace;

TEST(PeerTracker, ExtrapolatesWithEstimatedVelocity)
{
  PeerTracker tracker;
  tracker.update("amr_02", 10.0, 0.0, 0.0, 0.0);
  tracker.update("amr_02", 10.5, 0.25, 0.0, 0.0);  // 0.5 m/s 동쪽
  const auto poses = tracker.predict(11.0);
  ASSERT_EQ(poses.size(), 1u);
  EXPECT_NEAR(poses[0].x, 0.5, 1e-9);
  EXPECT_NEAR(poses[0].y, 0.0, 1e-9);
}

TEST(PeerTracker, CapsExtrapolationAndDropsStale)
{
  PeerTracker::Config config;
  config.timeout_s = 2.0;
  config.max_extrapolation_s = 1.0;
  PeerTracker tracker(config);
  tracker.update("amr_02", 0.0, 0.0, 0.0, 0.0);
  tracker.update("amr_02", 0.5, 0.25, 0.0, 0.0);
  EXPECT_NEAR(tracker.predict(2.0)[0].x, 0.75, 1e-9);  // 1.5 s 지났지만 1.0 s까지만 민다
  EXPECT_TRUE(tracker.predict(2.6).empty());            // 2.1 s 소식 없음
}

TEST(PeerTracker, IgnoresOlderStateAndClampsSpeed)
{
  PeerTracker tracker;
  tracker.update("amr_02", 1.0, 0.0, 0.0, 0.0);
  tracker.update("amr_02", 0.5, 5.0, 5.0, 0.0);  // 늦게 온 옛 상태
  EXPECT_NEAR(tracker.predict(1.0)[0].x, 0.0, 1e-9);
  tracker.update("amr_02", 1.5, 5.0, 0.0, 0.0);  // 위치 튐: 10 m/s -> 2 m/s로 자른다
  EXPECT_NEAR(tracker.predict(2.0)[0].x, 6.0, 1e-9);
}

TEST(PeerTracker, NoVelocityAcrossLongGap)
{
  PeerTracker tracker;
  tracker.update("amr_02", 0.0, 0.0, 0.0, 0.0);
  tracker.update("amr_02", 5.0, 1.0, 0.0, 0.0);
  EXPECT_NEAR(tracker.predict(5.5)[0].x, 1.0, 1e-9);
}

TEST(PeerTracker, NoExtrapolationWhenClockBehind)
{
  PeerTracker tracker;
  tracker.update("amr_02", 10.0, 0.0, 0.0, 0.0);
  tracker.update("amr_02", 10.5, 0.25, 0.0, 0.0);
  EXPECT_NEAR(tracker.predict(10.4)[0].x, 0.25, 1e-9);
}

TEST(Footprint, RotatedRectangleWithPadding)
{
  const auto corners = footprintCorners(PeerPose{1.0, 2.0, M_PI / 2}, 0.3, 0.2, 0.05);
  ASSERT_EQ(corners.size(), 4u);
  // yaw 90도: 길이 방향이 y축
  EXPECT_NEAR(corners[0].x, 1.0 - 0.25, 1e-9);
  EXPECT_NEAR(corners[0].y, 2.0 + 0.35, 1e-9);
  EXPECT_TRUE(pointInPolygon(corners, 1.0, 2.3));
  EXPECT_FALSE(pointInPolygon(corners, 1.3, 2.0));
}

TEST(Footprint, PointInPolygonNeedsThreePoints)
{
  EXPECT_FALSE(pointInPolygon({{0.0, 0.0}, {1.0, 1.0}}, 0.5, 0.5));
}

TEST(RobotId, FromNamespace)
{
  EXPECT_EQ(robotIdFromNamespace("/amr_01"), "amr_01");
  EXPECT_EQ(robotIdFromNamespace("/amr_03/global_costmap"), "amr_03");
  EXPECT_EQ(robotIdFromNamespace("/"), "");
  EXPECT_EQ(robotIdFromNamespace(""), "");
}
