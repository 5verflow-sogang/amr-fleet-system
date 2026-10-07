#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "amr_control/core/path_geometry.hpp"
#include "amr_control/core/pure_pursuit.hpp"
#include "amr_control/core/stanley.hpp"

namespace amr_control
{
namespace
{

Path straightPath(double length, double spacing)
{
  Path path;
  const int n = static_cast<int>(std::round(length / spacing));
  for (int i = 0; i <= n; ++i) {
    path.push_back(PathPoint{i * spacing, 0.0});
  }
  return path;
}

// 원점에서 +x로 출발해 왼쪽(반시계)으로 도는 반지름 radius 원호
Path arcPath(double radius, double angle, double spacing)
{
  Path path;
  const int n = static_cast<int>(std::ceil(radius * angle / spacing));
  for (int i = 0; i <= n; ++i) {
    const double a = angle * i / n;
    path.push_back(PathPoint{radius * std::sin(a), radius - radius * std::cos(a)});
  }
  return path;
}

TEST(PathGeometry, NormalizeAngle)
{
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(0.3), 0.3, 1e-12);
}

TEST(PathGeometry, CumulativeLengths)
{
  const Path path{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  const auto cum = cumulativeLengths(path);
  ASSERT_EQ(cum.size(), 3u);
  EXPECT_DOUBLE_EQ(cum[0], 0.0);
  EXPECT_DOUBLE_EQ(cum[1], 3.0);
  EXPECT_DOUBLE_EQ(cum[2], 7.0);
}

TEST(PathGeometry, ProjectionSignedCteLeftPositive)
{
  const Path path = straightPath(5.0, 0.5);
  const auto cum = cumulativeLengths(path);

  const auto left = projectOntoPath(path, cum, 2.2, 0.3);
  ASSERT_TRUE(left.valid);
  EXPECT_NEAR(left.x, 2.2, 1e-9);
  EXPECT_NEAR(left.y, 0.0, 1e-9);
  EXPECT_NEAR(left.s, 2.2, 1e-9);
  EXPECT_NEAR(left.yaw, 0.0, 1e-9);
  EXPECT_NEAR(left.cte, 0.3, 1e-9);

  const auto right = projectOntoPath(path, cum, 2.2, -0.3);
  EXPECT_NEAR(right.cte, -0.3, 1e-9);
}

TEST(PathGeometry, ProjectionSignFollowsPathDirection)
{
  // −x 방향으로 가는 경로에서는 y가 음수인 쪽이 왼쪽
  const Path path{{5.0, 0.0}, {0.0, 0.0}};
  const auto cum = cumulativeLengths(path);
  const auto proj = projectOntoPath(path, cum, 1.0, -0.2);
  EXPECT_NEAR(proj.cte, 0.2, 1e-9);
  EXPECT_NEAR(std::abs(proj.yaw), M_PI, 1e-9);
}

TEST(PathGeometry, ProjectionClampsToEnds)
{
  const Path path = straightPath(5.0, 1.0);
  const auto cum = cumulativeLengths(path);
  const auto past_end = projectOntoPath(path, cum, 6.0, 1.0);
  EXPECT_NEAR(past_end.x, 5.0, 1e-9);
  EXPECT_NEAR(past_end.s, 5.0, 1e-9);
  EXPECT_EQ(past_end.segment, path.size() - 2);
  EXPECT_NEAR(past_end.cte, std::hypot(1.0, 1.0), 1e-9);

  const auto before_start = projectOntoPath(path, cum, -1.0, 0.0);
  EXPECT_NEAR(before_start.x, 0.0, 1e-9);
  EXPECT_NEAR(before_start.s, 0.0, 1e-9);
}

TEST(PathGeometry, ProjectionSearchWindowSkipsOtherLeg)
{
  // U자 경로: (0,0)→(4,0)→(4,0.4)→(0,0.4). (1, 0.15)는 가는 길(아래)에 더 가깝다
  const Path path{{0.0, 0.0}, {4.0, 0.0}, {4.0, 0.4}, {0.0, 0.4}};
  const auto cum = cumulativeLengths(path);
  const auto from_start = projectOntoPath(path, cum, 1.0, 0.15);
  EXPECT_EQ(from_start.segment, 0u);

  // 이미 돌아오는 길(선분 2)에 들어섰으면 가는 길로 튀지 않는다
  const auto returning = projectOntoPath(path, cum, 1.0, 0.15, 2);
  EXPECT_EQ(returning.segment, 2u);
  EXPECT_NEAR(returning.y, 0.4, 1e-9);

  // 창(max_s) 밖 선분은 보지 않는다: 선분 0에서 시작해 창이 1 m면 돌아오는 길은 안 본다
  const auto windowed = projectOntoPath(path, cum, 1.0, 0.35, 0, 1.0);
  EXPECT_EQ(windowed.segment, 0u);
}

TEST(PathGeometry, ProjectionSkipsDuplicatePoints)
{
  const Path path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto cum = cumulativeLengths(path);
  const auto proj = projectOntoPath(path, cum, 1.0, 0.5);
  ASSERT_TRUE(proj.valid);
  EXPECT_NEAR(proj.yaw, 0.0, 1e-9);
  EXPECT_NEAR(proj.cte, 0.5, 1e-9);
}

TEST(PathGeometry, ProjectionDegenerateInputs)
{
  const Path empty;
  EXPECT_FALSE(projectOntoPath(empty, cumulativeLengths(empty), 0.0, 0.0).valid);

  const Path single{{1.0, 1.0}};
  const auto proj = projectOntoPath(single, cumulativeLengths(single), 1.0, 2.0);
  ASSERT_TRUE(proj.valid);
  EXPECT_NEAR(proj.cte, 1.0, 1e-9);
}

TEST(PathGeometry, PointAtArcLength)
{
  const Path path{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  const auto cum = cumulativeLengths(path);
  const auto mid = pointAtArcLength(path, cum, 5.0);
  EXPECT_NEAR(mid.x, 3.0, 1e-9);
  EXPECT_NEAR(mid.y, 2.0, 1e-9);
  const auto past = pointAtArcLength(path, cum, 100.0);
  EXPECT_NEAR(past.y, 4.0, 1e-9);
  const auto before = pointAtArcLength(path, cum, -1.0);
  EXPECT_NEAR(before.x, 0.0, 1e-9);
}

TEST(PathGeometry, CurvatureOfCircleAndLine)
{
  const Path arc = arcPath(2.0, M_PI, 0.05);
  const auto arc_cum = cumulativeLengths(arc);
  EXPECT_NEAR(curvatureAt(arc, arc_cum, 1.5, 0.25), 0.5, 0.01);
  // 경로 끝에서도 폭을 안으로 잘라 계산한다
  EXPECT_NEAR(curvatureAt(arc, arc_cum, 0.0, 0.25), 0.5, 0.02);

  const Path line = straightPath(5.0, 0.05);
  EXPECT_NEAR(curvatureAt(line, cumulativeLengths(line), 2.0, 0.25), 0.0, 1e-9);
}

TEST(PurePursuit, LookaheadScalesWithSpeed)
{
  PurePursuitConfig config;
  config.adaptive_lookahead = true;
  config.lookahead_time = 1.0;
  config.min_lookahead = 0.3;
  config.max_lookahead = 1.5;
  EXPECT_DOUBLE_EQ(purePursuitLookahead(config, 0.0), 0.3);
  EXPECT_DOUBLE_EQ(purePursuitLookahead(config, 0.8), 0.8);
  EXPECT_DOUBLE_EQ(purePursuitLookahead(config, -0.8), 0.8);
  EXPECT_DOUBLE_EQ(purePursuitLookahead(config, 3.0), 1.5);

  config.adaptive_lookahead = false;
  config.lookahead_distance = 0.6;
  EXPECT_DOUBLE_EQ(purePursuitLookahead(config, 2.0), 0.6);
}

TEST(PurePursuit, CurvatureThroughTarget)
{
  const Pose2D origin{0.0, 0.0, 0.0};
  EXPECT_NEAR(purePursuitCurvature(origin, PathPoint{1.0, 0.0}), 0.0, 1e-12);
  // (1, 1)을 지나고 원점에서 +x에 접하는 원: 반지름 1
  EXPECT_NEAR(purePursuitCurvature(origin, PathPoint{1.0, 1.0}), 1.0, 1e-12);
  EXPECT_NEAR(purePursuitCurvature(origin, PathPoint{1.0, -1.0}), -1.0, 1e-12);
  // 로봇이 +y를 보면 (−1, 1)이 왼쪽
  const Pose2D facing_up{0.0, 0.0, M_PI / 2.0};
  EXPECT_NEAR(purePursuitCurvature(facing_up, PathPoint{-1.0, 1.0}), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(purePursuitCurvature(origin, PathPoint{0.0, 0.0}), 0.0);
}

TEST(Stanley, ReferencePointAhead)
{
  const auto front = stanleyReferencePoint(Pose2D{1.0, 2.0, M_PI / 2.0}, 0.3);
  EXPECT_NEAR(front.x, 1.0, 1e-12);
  EXPECT_NEAR(front.y, 2.3, 1e-12);
}

TEST(Stanley, SteeringSigns)
{
  StanleyConfig config;
  config.cte_gain = 2.0;
  config.softening_speed = 0.2;
  config.max_steer = 1.2;
  EXPECT_DOUBLE_EQ(stanleySteeringAngle(config, 0.0, 0.0, 0.5), 0.0);
  // 경로 왼쪽에 있으면 오른쪽(−)으로 꺾는다
  EXPECT_LT(stanleySteeringAngle(config, 0.0, 0.1, 0.5), 0.0);
  EXPECT_GT(stanleySteeringAngle(config, 0.0, -0.1, 0.5), 0.0);
  // heading 오차만 있으면 그만큼 꺾는다
  EXPECT_NEAR(stanleySteeringAngle(config, 0.4, 0.0, 0.5), 0.4, 1e-12);
  // atan(k·e / (v_s + v)) 항
  EXPECT_NEAR(stanleySteeringAngle(config, 0.0, 0.1, 0.8), -std::atan(0.2 / 1.0), 1e-12);
  // 상한
  EXPECT_DOUBLE_EQ(stanleySteeringAngle(config, 0.0, 10.0, 0.0), -1.2);
}

TEST(Stanley, CurvatureFromSteer)
{
  StanleyConfig config;
  config.front_offset = 0.3;
  EXPECT_NEAR(stanleyCurvature(config, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(stanleyCurvature(config, M_PI / 4.0), 1.0 / 0.3, 1e-9);
}

}  // namespace
}  // namespace amr_control
