#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
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

// 직선(straight_len) 뒤에 이어지는 원호(반지름 radius, 각도 angle, 왼쪽으로). 커브 진입 때
// 각속도 한계가 v를 깎는 상황을 재현하려고(#279 리뷰) 순항 속도까지 올린 뒤 커브에 들어가게 한다
Path straightThenArc(double straight_len, double radius, double angle, double spacing = 0.05)
{
  Path path = straightPath(straight_len, spacing);
  const int n = static_cast<int>(std::ceil(radius * angle / spacing));
  for (int i = 1; i <= n; ++i) {
    const double a = angle * i / n;
    path.push_back(PathPoint{straight_len + radius * std::sin(a), radius - radius * std::cos(a)});
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

TEST(PathTracker, JerkLimitSlowsInitialAcceleration)
{
  PathTrackerConfig jerk_config;
  jerk_config.max_linear_jerk = 2.5;
  PathTracker jerk_tracker(jerk_config);
  jerk_tracker.setPath(straightPath(10.0));
  const auto jerk_first = jerk_tracker.compute(Pose2D{}, 0.0, kDt);

  // 기본값(저크 제한 꺼짐, max_linear_jerk == 0)과 비교하면 첫 틱부터 덜 밟는다
  PathTracker accel_only_tracker;
  accel_only_tracker.setPath(straightPath(10.0));
  const auto accel_first = accel_only_tracker.compute(Pose2D{}, 0.0, kDt);

  EXPECT_LT(jerk_first.v, accel_first.v);
  EXPECT_NEAR(jerk_first.v, jerk_config.max_linear_jerk * kDt * kDt, 1e-9);
}

TEST(PathTracker, NegativeJerkDisablesLimit)
{
  // 0이나 음수면 가속 제한만 쓰는 기존 동작과 같다
  PathTrackerConfig config;
  config.max_linear_jerk = -1.0;
  PathTracker tracker(config);
  tracker.setPath(straightPath(10.0));
  const auto first = tracker.compute(Pose2D{}, 0.0, kDt);
  EXPECT_NEAR(first.v, config.max_linear_accel * kDt, 1e-9);
}

// 저크 제한을 켜도 끝 감속 상한은 그대로 매 틱 씌우므로, 정지 정확도(StopsAtPathEnd와 같은 기준)는
// 그대로 지켜진다 (navigation.md 5-3절 "끝 감속·정지 정확도는 그대로")
TEST(PathTracker, JerkLimitDoesNotBreakStopAccuracy)
{
  PathTrackerConfig config;
  config.max_linear_jerk = 2.5;
  PathTracker tracker(config);
  tracker.setPath(straightPath(3.0));
  const auto sim = simulate(tracker, Pose2D{0.0, 0.0, 0.0});
  ASSERT_TRUE(sim.reached);
  const double tol = tracker.config().goal_tolerance;
  const double dist_to_end = std::hypot(sim.final_pose.x - 3.0, sim.final_pose.y);
  EXPECT_LT(dist_to_end, tol + 0.03);
  EXPECT_LT(sim.final_pose.x, 3.0 + 0.03);
  EXPECT_NEAR(sim.final_v, 0.0, 1e-3);
}

// #279 민지 리뷰 재현: 커브 진입 때 각속도 한계가 v를 깎으면(순항 → 곡률 상한), 저크가 그
// 감속을 "가속도 이력"으로 들고 가서 다음 틱부터 회복을 계속 느리게 묶어 거의 멈춰 섰다
// (κ=5에서 0.5 → 0.3 → 0.11 → 0). 고친 뒤에는 원호 구간 내내 v가 max_angular_speed/κ
// (곡률이 요구하는 값) 근처에 머물러야 한다 - 그보다 크게 더 떨어지면 버그가 재발한 것
TEST(PathTracker, JerkLimitDoesNotStallOnCurveEntry)
{
  for (const double kappa : {4.0, 5.0}) {
    SCOPED_TRACE("kappa " + std::to_string(kappa));
    const double radius = 1.0 / kappa;

    PathTrackerConfig config;
    config.max_linear_jerk = 2.5;
    PathTracker tracker(config);
    tracker.setPath(straightThenArc(3.0, radius, M_PI));

    const double v_floor = config.max_angular_speed / kappa;
    double min_v_in_curve = std::numeric_limits<double>::infinity();
    bool entered_curve = false;
    double v = 0.0;
    double w = 0.0;
    Pose2D pose{0.0, 0.0, 0.0};
    for (int i = 0; i < static_cast<int>(20.0 / kDt); ++i) {
      const auto result = tracker.compute(pose, v, kDt);
      if (result.status == TrackingStatus::kGoalReached) {
        break;
      }
      // 곡률이 커브 값(kappa)에 가까워지면 원호 구간으로 본다(직선 구간은 곡률 0)
      if (result.path_curvature > kappa * 0.5) {
        entered_curve = true;
        min_v_in_curve = std::min(min_v_in_curve, result.v);
      }
      v = result.v;
      w = result.w;
      pose.x += v * std::cos(pose.yaw) * kDt;
      pose.y += v * std::sin(pose.yaw) * kDt;
      pose.yaw = normalizeAngle(pose.yaw + w * kDt);
    }
    ASSERT_TRUE(entered_curve);
    // Pure Pursuit는 lookahead가 속도에 비례해 줄기 때문에(purePursuitLookahead), 커브 전체에서
    // 곡률 추정이 매 틱 조금씩 더 커지며 v가 서서히 v_floor 밑으로 내려간다 - 저크를 완전히 꺼도
    // (max_linear_jerk=0) 같은 경로에서 직접 재보면 동일하게 떨어진다(이 테스트와 같은 커브,
    // kappa=4: 79.6%, kappa=5: 78.3%, 저크 켰을 때(79.9%/79.3%)와 거의 같음) - 저크와 무관한
    // Pure Pursuit 고유 특성이다. 그 바닥(약 78~80%)보다 5%p 여유를 둔 75%를 기준으로 삼는다 -
    // 고치기 전 버그(0까지 주저앉음)와는 확실히 구분되면서도, 더 민감하게 재발을 잡는다
    EXPECT_GT(min_v_in_curve, v_floor * 0.75);
  }
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
