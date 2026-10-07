#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "amr_control/core/path_geometry.hpp"
#include "amr_control/core/path_tracker.hpp"

namespace amr_control
{
namespace
{

constexpr double kDt = 0.05;  // 제어 주기 20 Hz (config/path_tracker.yaml control_rate_hz)

Path straightPath(double length, double spacing = 0.05)
{
  Path path;
  const int n = static_cast<int>(std::round(length / spacing));
  for (int i = 0; i <= n; ++i) {
    path.push_back(PathPoint{i * spacing, 0.0});
  }
  return path;
}

// 원점에서 +x로 출발해 왼쪽(반시계)으로 도는 반지름 radius 원호
Path arcPath(double radius, double angle, double spacing = 0.05)
{
  Path path;
  const int n = static_cast<int>(std::ceil(radius * angle / spacing));
  for (int i = 0; i <= n; ++i) {
    const double a = angle * i / n;
    path.push_back(PathPoint{radius * std::sin(a), radius - radius * std::cos(a)});
  }
  return path;
}

struct Sample
{
  double x;
  double y;
  double cte;  // 로봇 중심의 부호 있는 CTE
  TrackingStatus status;
};

struct SimResult
{
  std::vector<Sample> samples;
  Pose2D final_pose;
  double final_v = 0.0;
  bool reached = false;
  double max_abs_w_cmd = 0.0;
};

// 차동구동 폐루프 시뮬레이션. 실제 속도는 명령을 1차 지연(시정수 tau)으로 따라간다
// (모터·속도 제어 지연을 흉내 내 이상적인 즉시 응답보다 보수적으로 본다).
SimResult simulate(
  PathTracker & tracker, Pose2D pose, double max_time = 60.0, double tau = 0.1)
{
  SimResult out;
  double v = 0.0;
  double w = 0.0;
  const double alpha = kDt / (tau + kDt);
  const int steps = static_cast<int>(max_time / kDt);
  for (int i = 0; i < steps; ++i) {
    const auto result = tracker.compute(pose, v, kDt);
    out.samples.push_back(Sample{pose.x, pose.y, result.projection.cte, result.status});
    out.max_abs_w_cmd = std::max(out.max_abs_w_cmd, std::abs(result.w));
    if (result.status == TrackingStatus::kGoalReached) {
      out.reached = true;
    }
    // 도착 뒤에는 남은 관성으로 미끄러지는 것까지 보고 끝낸다
    if (out.reached && std::abs(v) < 1e-3) {
      break;
    }
    v += alpha * (result.v - v);
    w += alpha * (result.w - w);
    pose.x += v * std::cos(pose.yaw) * kDt;
    pose.y += v * std::sin(pose.yaw) * kDt;
    pose.yaw = normalizeAngle(pose.yaw + w * kDt);
  }
  out.final_pose = pose;
  out.final_v = v;
  return out;
}

// predicate(sample)가 처음 참이 된 뒤 끝까지의 최대 |cte|
template<typename Pred>
double maxAbsCteAfter(const SimResult & sim, Pred predicate)
{
  double max_cte = 0.0;
  bool started = false;
  for (const auto & s : sim.samples) {
    started = started || predicate(s);
    if (started && s.status != TrackingStatus::kGoalReached) {
      max_cte = std::max(max_cte, std::abs(s.cte));
    }
  }
  return started ? max_cte : 1e9;
}

PathTrackerConfig makeConfig(ControllerType type)
{
  PathTrackerConfig config;
  config.controller = type;
  return config;
}

class TrackerTest : public ::testing::TestWithParam<ControllerType>
{
protected:
  PathTracker makeTracker(const Path & path) const
  {
    PathTracker tracker(makeConfig(GetParam()));
    tracker.setPath(path);
    return tracker;
  }
};

// 직선 경로: 0.5 m 옆에서 출발해 수렴하고, 수렴한 뒤로는 명세 KPI(직선 5 cm) 안에 머문다
TEST_P(TrackerTest, StraightPathConverges)
{
  auto tracker = makeTracker(straightPath(10.0));
  const auto sim = simulate(tracker, Pose2D{0.0, 0.5, 0.0});
  ASSERT_TRUE(sim.reached);
  EXPECT_LT(maxAbsCteAfter(sim, [](const Sample & s) {return s.x >= 3.0;}), 0.05);
  EXPECT_LT(std::abs(sim.samples.back().cte), 0.01);
}

// 횡방향 오프셋 출발: ±1 m 옆, 경로에서 멀어지는 쪽을 보고 출발해도 4 m 안에 5 cm로 들어오고,
// 반대쪽으로 크게 넘어가지 않는다
TEST_P(TrackerTest, LateralOffsetStart)
{
  for (const double side : {1.0, -1.0}) {
    SCOPED_TRACE("side " + std::to_string(side));
    auto tracker = makeTracker(straightPath(10.0));
    const auto sim = simulate(tracker, Pose2D{0.0, side, 0.3 * side});
    ASSERT_TRUE(sim.reached);
    EXPECT_LT(maxAbsCteAfter(sim, [](const Sample & s) {return s.x >= 4.0;}), 0.05);
    double overshoot = 0.0;
    for (const auto & s : sim.samples) {
      overshoot = std::max(overshoot, -side * s.cte);
    }
    EXPECT_LT(overshoot, 0.1);
  }
}

// 원호 경로: 경로 위에서 출발하면 명세 KPI(곡선 10 cm) 안에 머문다.
// Stanley는 기준점(앞 0.3 m)이 경로를 따라가므로 중심은 안쪽으로 약 d²/(2R)만큼 붙는다 (R=1 m에서 4.5 cm)
TEST_P(TrackerTest, ArcPathCteBounded)
{
  for (const double radius : {1.0, 2.0, 5.0}) {
    SCOPED_TRACE("radius " + std::to_string(radius));
    auto tracker = makeTracker(arcPath(radius, M_PI));
    const auto sim = simulate(tracker, Pose2D{0.0, 0.0, 0.0});
    ASSERT_TRUE(sim.reached);
    EXPECT_LT(maxAbsCteAfter(sim, [](const Sample &) {return true;}), 0.10);
  }
}

// 경로 끝 정지: 끝점 허용오차 안에서 서고, 끝점을 크게 지나치지 않으며, 도착 뒤에는 계속 (0, 0)
TEST_P(TrackerTest, StopsAtPathEnd)
{
  auto tracker = makeTracker(straightPath(3.0));
  const auto sim = simulate(tracker, Pose2D{0.0, 0.0, 0.0});
  ASSERT_TRUE(sim.reached);
  const double tol = tracker.config().goal_tolerance;
  const double dist_to_end = std::hypot(sim.final_pose.x - 3.0, sim.final_pose.y);
  // 1차 지연으로 정지 명령 뒤 조금 더 미끄러진다
  EXPECT_LT(dist_to_end, tol + 0.03);
  EXPECT_LT(sim.final_pose.x, 3.0 + 0.03);
  EXPECT_NEAR(sim.final_v, 0.0, 1e-3);

  for (int i = 0; i < 5; ++i) {
    const auto result = tracker.compute(sim.final_pose, 0.0, kDt);
    EXPECT_EQ(result.status, TrackingStatus::kGoalReached);
    EXPECT_DOUBLE_EQ(result.v, 0.0);
    EXPECT_DOUBLE_EQ(result.w, 0.0);
  }
}

// 경로 반대쪽을 보고 출발하면 서서 돈 뒤 따라간다
TEST_P(TrackerTest, RotatesInPlaceWhenFacingAway)
{
  auto tracker = makeTracker(straightPath(5.0));
  const auto first = tracker.compute(Pose2D{0.0, 0.0, M_PI}, 0.0, kDt);
  EXPECT_EQ(first.status, TrackingStatus::kRotating);
  EXPECT_DOUBLE_EQ(first.v, 0.0);
  EXPECT_GT(std::abs(first.w), 0.0);

  const auto sim = simulate(tracker, Pose2D{0.0, 0.0, M_PI});
  ASSERT_TRUE(sim.reached);
  EXPECT_NEAR(sim.final_pose.x, 5.0, tracker.config().goal_tolerance + 0.03);
}

// 급커브에서도 각속도 한계를 넘지 않는다 (넘으면 선속도를 낮춰 곡률을 지킨다)
TEST_P(TrackerTest, RespectsAngularSpeedLimit)
{
  auto tracker = makeTracker(arcPath(0.25, M_PI));
  const auto sim = simulate(tracker, Pose2D{0.0, 0.0, 0.0});
  EXPECT_LE(sim.max_abs_w_cmd, tracker.config().max_angular_speed + 1e-9);
}

// 주행 중 같은 경로를 다시 받아도(재계획) 속도가 0으로 떨어지지 않는다
TEST_P(TrackerTest, ReplanKeepsSpeed)
{
  const Path path = straightPath(10.0);
  auto tracker = makeTracker(path);
  Pose2D pose{0.0, 0.0, 0.0};
  double v = 0.0;
  for (int i = 0; i < 40; ++i) {
    v = tracker.compute(pose, v, kDt).v;
    pose.x += v * kDt;
  }
  ASSERT_GT(v, 0.4);
  tracker.setPath(path);
  EXPECT_GE(tracker.compute(pose, v, kDt).v, v - 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
  Controllers, TrackerTest,
  ::testing::Values(ControllerType::kPurePursuit, ControllerType::kStanley),
  [](const ::testing::TestParamInfo<ControllerType> & info) {
    return info.param == ControllerType::kStanley ? std::string("Stanley") :
    std::string("PurePursuit");
  });

TEST(PathTracker, NoPathGivesZero)
{
  PathTracker tracker;
  const auto result = tracker.compute(Pose2D{}, 0.0, kDt);
  EXPECT_EQ(result.status, TrackingStatus::kNoPath);
  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_DOUBLE_EQ(result.w, 0.0);
}

TEST(PathTracker, AccelerationLimited)
{
  PathTracker tracker;
  tracker.setPath(straightPath(10.0));
  const auto first = tracker.compute(Pose2D{}, 0.0, kDt);
  EXPECT_NEAR(first.v, tracker.config().max_linear_accel * kDt, 1e-9);
}

TEST(PathTracker, ControllerTypeNames)
{
  ControllerType type = ControllerType::kPurePursuit;
  EXPECT_TRUE(parseControllerType("stanley", type));
  EXPECT_EQ(type, ControllerType::kStanley);
  EXPECT_TRUE(parseControllerType("pure_pursuit", type));
  EXPECT_EQ(type, ControllerType::kPurePursuit);
  EXPECT_FALSE(parseControllerType("dwa", type));
  EXPECT_EQ(controllerTypeName(ControllerType::kStanley), "stanley");
  EXPECT_EQ(controllerTypeName(ControllerType::kPurePursuit), "pure_pursuit");
}

}  // namespace
}  // namespace amr_control
