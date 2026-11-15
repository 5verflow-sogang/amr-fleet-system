#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "amr_planning_core/grid.hpp"
#include "amr_planning_core/passing.hpp"

namespace amr_planning_core
{
namespace
{

// 10 m x 10 m, 0.05 m 격자. 경로는 y = 5를 따라 +x로 간다 (왼쪽 = +y)
constexpr double kRes = 0.05;
constexpr std::size_t kCells = 200;

struct TestGrid
{
  std::vector<uint8_t> costs = std::vector<uint8_t>(kCells * kCells, 0);
  Grid view() const {return Grid(kCells, kCells, kRes, costs.data());}
  // y가 [y0, y1)인 띠를 벽으로
  void wall(double y0, double y1)
  {
    for (std::size_t cy = 0; cy < kCells; ++cy) {
      const double y = (cy + 0.5) * kRes;
      if (y < y0 || y >= y1) {
        continue;
      }
      for (std::size_t cx = 0; cx < kCells; ++cx) {
        costs[cy * kCells + cx] = 254;
      }
    }
  }
};

std::vector<Point2D> straightPath()
{
  std::vector<Point2D> path;
  for (double x = 0.5; x <= 9.5 + 1e-9; x += 0.05) {
    path.push_back(Point2D{x, 5.0});
  }
  return path;
}

// 제어 주기 시각 (20 Hz). 테스트마다 0부터
double g_now = 0.0;
double tick()
{
  g_now += 0.05;
  return g_now;
}

PassingConfig enabledConfig()
{
  PassingConfig c;
  c.enabled = true;
  c.clear_cycles = 3;
  return c;
}

RobotState robotAt(double x, double y = 5.0, double v = 0.5)
{
  RobotState r;
  r.x = x;
  r.y = y;
  r.v = v;
  return r;
}

DynamicObstacle walker(double x, double y, double vx, double vy, double radius = 0.25)
{
  DynamicObstacle o;
  o.x = x;
  o.y = y;
  o.vx = vx;
  o.vy = vy;
  o.radius = radius;
  return o;
}

// 로봇 x에서 옮긴 경로의 y (경로 점 중 x가 가장 가까운 점)
double shiftedYAt(const std::vector<Point2D> & path, double x)
{
  double best = 1e9;
  double y = 0.0;
  for (const Point2D & p : path) {
    if (std::abs(p.x - x) < best) {
      best = std::abs(p.x - x);
      y = p.y;
    }
  }
  return y;
}

TEST(PassingManeuver, DisabledReturnsPathUnchanged)
{
  g_now = 0.0;
  PassingConfig c;
  PassingManeuver m(c);
  TestGrid g;
  const auto path = straightPath();
  const auto out = m.update(robotAt(1.0), path, g.view(), {walker(3.0, 5.0, -1.0, 0.0)}, tick());
  EXPECT_FALSE(m.active());
  ASSERT_EQ(out.size(), path.size());
  EXPECT_DOUBLE_EQ(out[40].y, path[40].y);
}

TEST(PassingManeuver, ProjectGivesSignedLateral)
{
  g_now = 0.0;
  const auto path = straightPath();
  const auto s = PassingManeuver::arcLengths(path);
  const auto left = PassingManeuver::project(path, s, Point2D{2.0, 5.3});
  const auto right = PassingManeuver::project(path, s, Point2D{2.0, 4.8});
  ASSERT_TRUE(left && right);
  EXPECT_NEAR(left->s, 1.5, 1e-6);
  EXPECT_NEAR(left->d, 0.3, 1e-6);
  EXPECT_NEAR(right->d, -0.2, 1e-6);
  // 시작점(0.5)보다 뒤는 늘린 직선 위 s (음수), 끝점(9.5) 너머는 끝점에 붙는다
  const auto behind = PassingManeuver::project(path, s, Point2D{-0.5, 5.2});
  const auto beyond = PassingManeuver::project(path, s, Point2D{10.5, 4.9});
  ASSERT_TRUE(behind && beyond);
  EXPECT_NEAR(behind->s, -1.0, 1e-6);
  EXPECT_NEAR(behind->d, 0.2, 1e-6);
  EXPECT_NEAR(beyond->s, 9.0, 1e-6);
}

TEST(PassingManeuver, WaitsUntilTimeToMeetThenSteps)
{
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  const auto path = straightPath();
  // 1.0 m/s로 마주 오는 작업자. 닿기까지 (gap - 0.25 - 0.3) / 1.5 s ≤ ramp_out/v + start_margin = 1.6 s 일 때 시작
  m.update(robotAt(1.0), path, g.view(), {walker(6.0, 5.0, -1.0, 0.0)}, tick());
  // 4.45 m / 1.5 m/s = 3.0 s > ramp_out/v + start_margin = 2.0 s
  EXPECT_FALSE(m.active()) << "아직 이르다";
  EXPECT_TRUE(m.engaged()) << "곧 비킬 예정: 회피 모드를 끈다";
  m.update(robotAt(1.0), path, g.view(), {}, tick());
  EXPECT_TRUE(m.engaged()) << "트랙이 끊겨도 예측으로 이어 본다";
  g_now += 2.0;
  m.update(robotAt(1.0), path, g.view(), {}, tick());
  EXPECT_FALSE(m.engaged()) << "threat_memory(1.5 s)가 지나면 예정도 푼다";
  const auto out = m.update(robotAt(1.0), path, g.view(), {walker(4.4, 5.0, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active()) << "2.85 m / 1.5 m/s = 1.9 s - 시작";
  // 경로 가운데라 오른쪽(-y)으로, 작업자 반경 + 반폭 + 여유 = 0.25 + 0.2 + 0.3
  EXPECT_NEAR(m.offset(), -0.75, 1e-9);
  EXPECT_NEAR(shiftedYAt(out, 1.0), 5.0, 1e-6) << "시작 자리에서는 아직 0";
  EXPECT_NEAR(shiftedYAt(out, 1.0 + 0.6), 4.25, 1e-6) << "ramp_out 뒤 다 비켜선다";
  EXPECT_NEAR(shiftedYAt(out, 1.3), 5.0 - 0.75 * 0.5, 1e-6) << "S자 가운데 절반";
}

TEST(PassingManeuver, StepsAwayFromOffsetObstacleByOnlyWhatIsNeeded)
{
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  m.update(robotAt(1.0), straightPath(), g.view(), {walker(3.5, 5.2, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active());
  EXPECT_NEAR(m.offset(), 0.2 - 0.75, 1e-9) << "왼쪽에 치우친 작업자: 오른쪽으로 0.55 m만";
  PassingManeuver n(enabledConfig());
  n.update(robotAt(1.0), straightPath(), g.view(), {walker(3.5, 4.75, -1.0, 0.0)}, tick());
  ASSERT_TRUE(n.active());
  EXPECT_NEAR(n.offset(), -0.25 + 0.75, 1e-9) << "오른쪽에 치우친 작업자: 왼쪽으로 0.5 m";
}

TEST(PassingManeuver, IgnoresCrossingAndSameDirection)
{
  g_now = 0.0;
  TestGrid g;
  PassingManeuver crossing(enabledConfig());
  crossing.update(robotAt(1.0), straightPath(), g.view(), {walker(3.0, 5.0, 0.0, 1.0)}, tick());
  EXPECT_FALSE(crossing.active()) << "가로지르기는 기존 회피 모드·안전 노드가 맡는다";
  PassingManeuver ahead(enabledConfig());
  ahead.update(robotAt(1.0), straightPath(), g.view(), {walker(2.5, 5.0, 0.3, 0.0)}, tick());
  EXPECT_FALSE(ahead.active()) << "같은 방향으로 가는 작업자";
  PassingManeuver slow(enabledConfig());
  slow.update(robotAt(1.0), straightPath(), g.view(), {walker(2.5, 5.0, -0.1, 0.0)}, tick());
  EXPECT_FALSE(slow.active()) << "min_obstacle_speed 아래 (서 있는 것은 정적 장애물로 본다)";
  PassingManeuver aside(enabledConfig());
  aside.update(robotAt(1.0), straightPath(), g.view(), {walker(3.0, 5.9, -1.0, 0.0)}, tick());
  EXPECT_FALSE(aside.active()) << "경로를 막지 않는다 (0.9 m 옆)";
}

TEST(PassingManeuver, PicksOtherSideWhenWallAndSkipsWhenBothBlocked)
{
  g_now = 0.0;
  TestGrid g;
  g.wall(0.0, 4.6);   // 오른쪽(-y) 0.4 m 앞까지 벽: 0.75 m 비키면 차체가 닿는다
  PassingManeuver m(enabledConfig());
  m.update(robotAt(1.0), straightPath(), g.view(), {walker(3.5, 5.0, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active());
  EXPECT_NEAR(m.offset(), 0.75, 1e-9) << "오른쪽이 막혀 왼쪽으로";
  g.wall(5.4, 10.0);  // 왼쪽도 막는다 (0.6 m 통로)
  PassingManeuver n(enabledConfig());
  const auto path = straightPath();
  const auto out = n.update(robotAt(1.0), path, g.view(), {walker(3.5, 5.0, -1.0, 0.0)}, tick());
  EXPECT_FALSE(n.active()) << "비킬 자리가 없으면 기존 동작(감속·정지)";
  EXPECT_FALSE(n.engaged()) << "회피 모드도 그대로 둔다";
  EXPECT_DOUBLE_EQ(out[60].y, path[60].y);
}

TEST(PassingManeuver, SkipsWhenOffsetWouldExceedLimit)
{
  g_now = 0.0;
  PassingConfig c = enabledConfig();
  c.max_offset = 0.6;
  PassingManeuver m(c);
  TestGrid g;
  m.update(robotAt(1.0), straightPath(), g.view(), {walker(3.5, 5.0, -1.0, 0.0)}, tick());
  EXPECT_FALSE(m.active()) << "0.75 m 비켜야 하는데 상한 0.6 m";
}

TEST(PassingManeuver, HoldsWhileAlongsideAndReturnsWithinRampAfterPassing)
{
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  const auto path = straightPath();
  m.update(robotAt(1.0), path, g.view(), {walker(3.6, 5.0, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active());
  // 로봇 2.0 (다 비켜섬), 작업자 바로 옆 → 유지
  m.update(robotAt(2.0, 4.25), path, g.view(), {walker(2.1, 5.0, -1.0, 0.0)}, tick());
  EXPECT_EQ(m.phase(), PassingPhase::kHold);
  // 작업자가 차체 뒤 끝을 지났지만 아직 pass_margin 안 (2.2 - 0.3 - 0.25 - 0.2 = 1.45 < 1.5): 유지
  m.update(robotAt(2.2, 4.25), path, g.view(), {walker(1.5, 5.0, -1.0, 0.0)}, tick());
  EXPECT_EQ(m.phase(), PassingPhase::kHold);
  // 다 지나감 → clear_cycles(3) 연속 뒤 그 자리에서 들어오기 시작
  for (int i = 0; i < 3; ++i) {
    m.update(robotAt(2.3, 4.25), path, g.view(), {walker(0.8, 5.0, -1.0, 0.0)}, tick());
  }
  EXPECT_EQ(m.phase(), PassingPhase::kReturn);
  EXPECT_NEAR(m.lateral(2.3 - 0.5 + 0.6), 0.0, 1e-9) << "ramp_in 0.6 m 뒤 처음 경로 위";
  // ramp_in을 지나면 끝
  const auto out = m.update(robotAt(2.3 + 0.61, 5.0), path, g.view(), {}, tick());
  EXPECT_FALSE(m.active());
  EXPECT_DOUBLE_EQ(out[50].y, path[50].y);
}

TEST(PassingManeuver, TrackDropoutDoesNotTriggerEarlyReturn)
{
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  const auto path = straightPath();
  m.update(robotAt(1.0), path, g.view(), {walker(3.6, 5.0, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active());
  m.update(robotAt(1.8, 4.3), path, g.view(), {}, tick());           // 한 주기 추적 끊김
  m.update(robotAt(1.85, 4.3), path, g.view(), {walker(2.6, 5.0, -1.0, 0.0)}, tick());
  m.update(robotAt(1.9, 4.3), path, g.view(), {walker(2.5, 5.0, -1.0, 0.0)}, tick());
  EXPECT_NE(m.phase(), PassingPhase::kReturn) << "작업자가 아직 앞에 있다";
  EXPECT_NEAR(m.offset(), -0.75, 1e-9);
}

TEST(PassingManeuver, PredictionKeepsHoldingThroughLongDropoutUntilPredictedPass)
{
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  const auto path = straightPath();
  m.update(robotAt(1.0), path, g.view(), {walker(3.6, 5.0, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active());
  // 1.0 s 동안 트랙 없음: 예측 위치 3.6 - 1.0 = 2.6 (로봇 2.0 앞) → 계속 유지
  for (int i = 0; i < 20; ++i) {
    m.update(robotAt(2.0, 4.25), path, g.view(), {}, tick());
  }
  EXPECT_NE(m.phase(), PassingPhase::kReturn);
  // 예측으로 지나감(1.6 s 뒤 3.6 - 1.6 = 2.0 → 2.0 - 0.3 - 0.25 - 0.2 보다 뒤가 되면): 들어온다
  for (int i = 0; i < 20; ++i) {
    m.update(robotAt(2.0, 4.25), path, g.view(), {}, tick());
  }
  EXPECT_EQ(m.phase(), PassingPhase::kReturn);
}

TEST(PassingManeuver, NewPathResetsButRollingOriginShiftKeepsState)
{
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  auto path = straightPath();
  m.update(robotAt(1.0), path, g.view(), {walker(3.6, 5.0, -1.0, 0.0)}, tick());
  ASSERT_TRUE(m.active());
  // 롤링 costmap: 모든 점과 로봇·장애물이 같은 양 옮겨짐 → 같은 회피를 이어 간다
  for (Point2D & p : path) {
    p.x -= 0.1;
  }
  m.update(robotAt(0.95), path, g.view(), {walker(3.4, 5.0, -1.0, 0.0)}, tick());
  EXPECT_TRUE(m.active());
  // 새 전역 경로(모양이 다름) → 처음부터
  path.pop_back();
  m.update(robotAt(0.95), path, g.view(), {}, tick());
  EXPECT_FALSE(m.active());
}

TEST(PassingManeuver, NewPathFromRobotIgnoresObstacleAlreadyBehind)
{
  // 비켜 지나간 뒤 로봇 자리(x 3)에서 시작하는 새 전역 경로가 왔다. 작업자는 1 m 뒤에서 멀어지는 중
  // (경로 방향과 반대로 걸으니 '마주 오는' 방향이다). 시작점 뒤라 위협이 아니다
  g_now = 0.0;
  PassingManeuver m(enabledConfig());
  TestGrid g;
  std::vector<Point2D> path;
  for (double x = 3.0; x <= 9.5 + 1e-9; x += 0.05) {
    path.push_back(Point2D{x, 5.0});
  }
  for (int i = 0; i < 5; ++i) {
    m.update(robotAt(3.0 + 0.025 * i), path, g.view(), {walker(2.0 - 0.05 * i, 5.0, -1.0, 0.0)}, tick());
    EXPECT_FALSE(m.engaged());
  }
}

}  // namespace
}  // namespace amr_planning_core
