#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "amr_planning_core/dwa.hpp"
#include "amr_planning_core/grid.hpp"

namespace amr_planning_core
{
namespace
{

// 테스트용 격자. costs_ 버퍼를 소유하고, Grid는 그 버퍼를 가리키는 뷰만 갖는다.
// (test_astar.cpp의 TestGrid와 같은 패턴이지만, world 좌표 영역을 바로 채우는
//  setRegion()이 있어 DWA 시나리오를 좌표로 기술하기 편하다.)
class TestGrid
{
public:
  TestGrid(std::size_t width, std::size_t height, double resolution, uint8_t default_cost = 0)
  : width_(width), height_(height), resolution_(resolution),
    costs_(width * height, default_cost)
  {
  }

  void setCell(int x, int y, uint8_t cost)
  {
    if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= width_ ||
      static_cast<std::size_t>(y) >= height_)
    {
      return;
    }
    costs_[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)] = cost;
  }

  // world 좌표 사각 영역([x_min,x_max] x [y_min,y_max], 경계 포함)을 cost로 채운다.
  void setRegion(double x_min, double x_max, double y_min, double y_max, uint8_t cost)
  {
    const int cx_min = static_cast<int>(std::floor(x_min / resolution_));
    const int cx_max = static_cast<int>(std::floor(x_max / resolution_));
    const int cy_min = static_cast<int>(std::floor(y_min / resolution_));
    const int cy_max = static_cast<int>(std::floor(y_max / resolution_));
    for (int cy = cy_min; cy <= cy_max; ++cy) {
      for (int cx = cx_min; cx <= cx_max; ++cx) {
        setCell(cx, cy, cost);
      }
    }
  }

  Grid grid() const {return Grid(width_, height_, resolution_, costs_.data());}

private:
  std::size_t width_;
  std::size_t height_;
  double resolution_;
  std::vector<uint8_t> costs_;
};

// 폐루프·좁은 통로 테스트는 수백 번 computeVelocityCommand를 반복 호출하므로,
// 기본 설정(20x30 샘플)보다 샘플을 줄여 테스트 시간을 줄인다. sim_time은 기본값과
// 같게 유지한다 - 짧게 줄이면 장애물을 더 늦게(가까이 와서야) "미래 충돌"로
// 감지해서 매 스텝 조금씩 미리 피하지 못하고 막판에 급커브를 강요받는다. 동적
// 윈도우(한 스텝에 꺾을 수 있는 각속도, acc_w*dt)는 sim_time과 무관하므로 그걸로는
// 못 메운다 - 그래서 호라이즌은 그대로 두고 샘플 개수만 줄인다.
DwaConfig makeFastConfig()
{
  DwaConfig config;
  config.v_samples = 5;
  config.w_samples = 7;
  return config;
}

}  // namespace

TEST(DwaPlannerTest, DynamicWindowRespectsAccelerationLimits)
{
  DwaConfig config;
  DwaPlanner planner(config);

  // 중간 속도에서 출발 - 가속 한계 안이라 양쪽 다 acc*dt만큼만 벌어져야 한다.
  RobotState cruising;
  cruising.v = 1.0;
  cruising.w = 0.0;
  const DynamicWindow cruising_window = planner.computeDynamicWindow(cruising);
  EXPECT_NEAR(cruising_window.max_v - cruising.v, config.acc_v * config.dt, 1e-9);
  EXPECT_NEAR(cruising.v - cruising_window.min_v, config.acc_v * config.dt, 1e-9);
  EXPECT_NEAR(cruising_window.max_w, config.acc_w * config.dt, 1e-9);
  EXPECT_NEAR(cruising_window.min_w, -config.acc_w * config.dt, 1e-9);

  // 로봇 한계(max_v) 근처에서 출발 - 가속 한계대로면 max_v를 넘어서야 하지만
  // 로봇 한계로 잘려야 한다.
  RobotState near_max;
  near_max.v = config.max_v - 0.05;
  near_max.w = 0.0;
  const DynamicWindow clipped_window = planner.computeDynamicWindow(near_max);
  EXPECT_NEAR(clipped_window.max_v, config.max_v, 1e-9);
  EXPECT_LE(clipped_window.max_v, config.max_v + 1e-9);
}

TEST(DwaPlannerTest, EmptyMapMovesTowardGoal)
{
  TestGrid test_grid(200, 100, 0.05);  // 10m x 5m, 전부 비어 있음
  DwaPlanner planner;

  RobotState state;
  state.x = 1.0;
  state.y = 1.0;
  state.yaw = 0.0;
  state.v = 0.8;
  state.w = 0.0;

  const Point2D goal{6.0, 1.0};  // 정면으로 멀리

  const DwaCommand command = planner.computeVelocityCommand(state, goal, test_grid.grid());
  ASSERT_EQ(command.status, DwaStatus::kOk);

  const DynamicWindow window = planner.computeDynamicWindow(state);
  // 장애물이 없고 목표가 정면이므로 가능한 한 빠르게, 거의 직진을 골라야 한다.
  EXPECT_NEAR(command.v, window.max_v, 1e-6);
  EXPECT_NEAR(command.w, 0.0, 0.05);
}

TEST(DwaPlannerTest, FrontObstacleTurnsAway)
{
  TestGrid test_grid(160, 80, 0.05);  // 8m x 4m
  // 로봇 경로 폭보다 넓은 벽. 동적 윈도우가 한 스텝에 꺾을 수 있는 각속도는
  // acc_w*dt(기본값 0.32 rad/s)뿐이다. 벽을 x 2.6~2.8m에 두면 직진(w~0)은 모든
  // 속도 샘플에서 충돌하고, |w|>=0.23 rad/s인 궤적은 충돌 없이 피해간다 - 실제
  // 값으로 수치 시뮬레이션해서 확인한 위치다(위아래는 열려 있음).
  test_grid.setRegion(2.6, 2.8, 1.7, 2.3, 254);

  DwaPlanner planner;

  RobotState state;
  state.x = 0.5;
  state.y = 2.0;
  state.yaw = 0.0;
  state.v = 1.0;
  state.w = 0.0;

  const Point2D goal{6.0, 2.0};  // 벽 너머 정면

  const DwaCommand command = planner.computeVelocityCommand(state, goal, test_grid.grid());
  ASSERT_EQ(command.status, DwaStatus::kOk);
  // 직진(w=0)은 모든 속도에서 벽에 부딪히므로, 피하려면 회전해야 한다.
  EXPECT_GT(std::abs(command.w), 0.05);

  // 고른 (v, w)로 시뮬레이션한 궤적이 실제로 충돌하지 않는지 다시 확인한다.
  const std::vector<TrajectoryPoint> trajectory =
    planner.simulateTrajectory(state, command.v, command.w);
  for (const TrajectoryPoint & point : trajectory) {
    EXPECT_FALSE(planner.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw));
  }
}

TEST(DwaPlannerTest, AllSidesBlockedReturnsNoValidTrajectory)
{
  TestGrid test_grid(60, 60, 0.05, /*default_cost=*/254);  // 전부 장애물
  DwaPlanner planner;

  RobotState state;
  state.x = 1.5;
  state.y = 1.5;
  state.yaw = 0.0;
  state.v = 0.0;
  state.w = 0.0;

  const Point2D goal{2.5, 1.5};

  const DwaCommand command = planner.computeVelocityCommand(state, goal, test_grid.grid());
  EXPECT_EQ(command.status, DwaStatus::kNoValidTrajectory);
  EXPECT_EQ(command.v, 0.0);
  EXPECT_EQ(command.w, 0.0);
}

TEST(DwaPlannerTest, ClosedLoopReachesGoalWithOneObstacle)
{
  TestGrid test_grid(120, 80, 0.1, 0);  // 12m x 8m
  // 시작과 목표 사이에 장애물 하나. 위아래로 돌아갈 공간은 열려 있다.
  test_grid.setRegion(5.0, 6.0, 3.2, 4.8, 254);

  const DwaConfig config = makeFastConfig();
  DwaPlanner planner(config);

  RobotState state;
  state.x = 1.0;
  state.y = 4.0;
  state.yaw = 0.0;
  state.v = 0.0;
  state.w = 0.0;

  const Point2D goal{10.0, 4.0};

  bool reached = false;
  // 장애물을 피하려고 크게 돌아가는 구간이 있어 여유 있게 잡는다(40초 분량).
  constexpr int kMaxIterations = 400;
  for (int i = 0; i < kMaxIterations; ++i) {
    const DwaCommand command = planner.computeVelocityCommand(state, goal, test_grid.grid());
    ASSERT_NE(command.status, DwaStatus::kNoValidTrajectory)
      << "iteration " << i << " found no valid trajectory";

    if (command.status == DwaStatus::kGoalReached) {
      reached = true;
      break;
    }

    state.v = command.v;
    state.w = command.w;
    state.x += state.v * std::cos(state.yaw) * config.dt;
    state.y += state.v * std::sin(state.yaw) * config.dt;
    state.yaw += state.w * config.dt;
  }

  EXPECT_TRUE(reached) << "목표에 도달하지 못하고 반복 상한에 도달함";
}

TEST(DwaPlannerTest, PassesThroughNarrowCorridorWithoutFootprintCollision)
{
  TestGrid test_grid(60, 20, 0.1, 0);  // 6m x 2m
  // x 2.0~4.0m 구간만 y 0.7~1.3m(폭 0.6m)로 좁히고, 그 밖은 전부 열어 둔다.
  test_grid.setRegion(2.0, 4.0, 0.0, 0.7, 254);
  test_grid.setRegion(2.0, 4.0, 1.3, 2.0, 254);

  const DwaConfig config = makeFastConfig();
  DwaPlanner planner(config);

  RobotState state;
  state.x = 0.5;
  state.y = 1.0;
  state.yaw = 0.0;
  state.v = 0.0;
  state.w = 0.0;

  const Point2D goal{5.5, 1.0};

  bool reached = false;
  constexpr int kMaxIterations = 200;
  for (int i = 0; i < kMaxIterations; ++i) {
    const DwaCommand command = planner.computeVelocityCommand(state, goal, test_grid.grid());
    ASSERT_NE(command.status, DwaStatus::kNoValidTrajectory)
      << "iteration " << i << " found no valid trajectory";

    if (command.status == DwaStatus::kGoalReached) {
      reached = true;
      break;
    }

    state.v = command.v;
    state.w = command.w;
    state.x += state.v * std::cos(state.yaw) * config.dt;
    state.y += state.v * std::sin(state.yaw) * config.dt;
    state.yaw += state.w * config.dt;

    // 매 스텝 실제 풋프린트가 통로 벽과 충돌하지 않았는지 확인한다.
    ASSERT_FALSE(planner.footprintCollides(test_grid.grid(), state.x, state.y, state.yaw))
      << "iteration " << i << " footprint collided";
  }

  EXPECT_TRUE(reached) << "목표에 도달하지 못하고 반복 상한에 도달함";
}

// --- 3주차: 동적 장애물(등속 예측) ---

TEST(DwaPlannerTest, StationaryDynamicObstacleStillBlocksRegardlessOfTime)
{
  DwaPlanner planner;  // predict_dynamic_obstacles = true 기본값

  DynamicObstacle stopped;
  stopped.x = 3.0;
  stopped.y = 2.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;
  stopped.radius = 0.3;
  const std::vector<DynamicObstacle> obstacles{stopped};

  // 속도가 0이면 등속 예측을 켜도(t가 커져도) 위치가 안 바뀌므로, 장애물 위에
  // 있으면 어느 시각에 확인해도 똑같이 충돌이어야 한다 - "정지 장애물은 기존
  // (정적 장애물) 결과와 같아야 한다"는 요구를 등속 예측 수식으로 만족하는지 확인.
  EXPECT_TRUE(planner.dynamicObstacleCollides(obstacles, 0.0, 3.0, 2.0, 0.0));
  EXPECT_TRUE(planner.dynamicObstacleCollides(obstacles, 2.0, 3.0, 2.0, 0.0));
  EXPECT_TRUE(planner.dynamicObstacleCollides(obstacles, 100.0, 3.0, 2.0, 0.0));

  // 충분히 떨어진 지점은 시각과 상관없이 충돌이 아니다.
  EXPECT_FALSE(planner.dynamicObstacleCollides(obstacles, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(planner.dynamicObstacleCollides(obstacles, 50.0, 0.0, 0.0, 0.0));
}

TEST(DwaPlannerTest, NoDynamicObstaclesMatchesStaticOnlyBehaviorExactly)
{
  // computeVelocityCommand에 동적 장애물을 안 넘기면(기본 인자, 빈 벡터) 2주차
  // 정적 장애물 전용 경로와 완전히 같은 결과가 나와야 한다 - dynamic_weight가
  // 곱해지는 항이 정확히 0.0이 되는지 확인한다(부동소수점 오차 없이).
  TestGrid test_grid(160, 80, 0.05);
  test_grid.setRegion(2.6, 2.8, 1.7, 2.3, 254);

  DwaPlanner planner;
  RobotState state;
  state.x = 0.5;
  state.y = 2.0;
  state.v = 1.0;

  const Point2D goal{6.0, 2.0};

  const DwaCommand without_arg = planner.computeVelocityCommand(state, goal, test_grid.grid());
  const DwaCommand with_empty_vector =
    planner.computeVelocityCommand(state, goal, test_grid.grid(), {});

  EXPECT_EQ(without_arg.status, with_empty_vector.status);
  EXPECT_EQ(without_arg.v, with_empty_vector.v);
  EXPECT_EQ(without_arg.w, with_empty_vector.w);
}

TEST(DwaPlannerTest, CrossingObstacleCollisionDependsOnPredictedTime)
{
  // (3,0)에서 y축 방향으로 2 m/s로 가로지르는 장애물 - t=1s에 (3,2)에 와서
  // 로봇과 같은 자리가 된다. "등속 예측" 없이는(predict_dynamic_obstacles=false)
  // 관측 당시 위치(3,0)에 그대로 있다고 보므로 (3,2)는 늘 안전하다고 오판한다.
  DynamicObstacle crossing;
  crossing.x = 3.0;
  crossing.y = 0.0;
  crossing.vx = 0.0;
  crossing.vy = 2.0;
  crossing.radius = 0.3;
  const std::vector<DynamicObstacle> obstacles{crossing};

  DwaPlanner predicting;  // predict_dynamic_obstacles = true
  EXPECT_FALSE(predicting.dynamicObstacleCollides(obstacles, 0.0, 3.0, 2.0, 0.0))
    << "t=0에는 장애물이 아직 (3,0)에 있어 2m 떨어져 있다";
  EXPECT_TRUE(predicting.dynamicObstacleCollides(obstacles, 1.0, 3.0, 2.0, 0.0))
    << "t=1에는 장애물이 (3,2)로 올라와 로봇과 같은 자리다";

  DwaConfig no_predict_config;
  no_predict_config.predict_dynamic_obstacles = false;
  DwaPlanner not_predicting(no_predict_config);
  EXPECT_FALSE(not_predicting.dynamicObstacleCollides(obstacles, 1.0, 3.0, 2.0, 0.0))
    << "예측을 꺼면 t=1에도 관측 위치(3,0) 기준으로만 보므로 충돌을 놓친다";
}

TEST(DwaPlannerTest, FrontApproachingObstacleIsAvoidedOnlyWithPrediction)
{
  TestGrid test_grid(320, 160, 0.05);  // 16m x 8m, 전부 비어 있음(정적 장애물 없음)

  RobotState state;
  state.x = 0.0;
  state.y = 2.0;
  state.yaw = 0.0;
  state.v = 2.0;  // 이미 최고속 근처 - 동적 윈도우가 [1.9, 2.0]으로 고정돼 계산이 쉽다
  state.w = 0.0;
  const Point2D goal{10.0, 2.0};  // 정면으로 멀리

  // 8 m 앞에서 로봇을 향해 3 m/s로 다가오는 장애물. 동적 윈도우 안에서 로봇이
  // 낼 수 있는 최대 전진 거리는 window.max_v(2.0)*sim_time(2.0)=4m(풋프린트
  // 앞면까지 더해도 4.3m)뿐이라, 관측 위치(8,2)에 멈춰 있다고 보면(예측 끔)
  // 전혀 안 걸린다(표면까지 3.4m 이상 남는다). 등속으로 예측하면 t=1.6s에
  // 장애물이 로봇과 같은 x(3.2m)로 와서 경로를 막는다(8-3*1.6 = 2*1.6 = 3.2).
  DynamicObstacle approaching;
  approaching.x = 8.0;
  approaching.y = 2.0;
  approaching.vx = -3.0;
  approaching.vy = 0.0;
  approaching.radius = 0.3;
  const std::vector<DynamicObstacle> obstacles{approaching};

  DwaPlanner predicting;  // predict_dynamic_obstacles = true 기본값
  const DynamicWindow window = predicting.computeDynamicWindow(state);

  // 시나리오가 실제로 위협적인지 먼저 확인: 장애물을 무시하고 정면 최고속으로
  // 가는 궤적은 예측된 장애물과 충돌해야 한다.
  const std::vector<TrajectoryPoint> naive_trajectory =
    predicting.simulateTrajectory(state, window.max_v, 0.0);
  bool naive_collides = false;
  for (std::size_t i = 0; i < naive_trajectory.size(); ++i) {
    const double t = static_cast<double>(i + 1) * predicting.config().dt;
    const TrajectoryPoint & p = naive_trajectory[i];
    if (predicting.dynamicObstacleCollides(obstacles, t, p.x, p.y, p.yaw)) {
      naive_collides = true;
      break;
    }
  }
  ASSERT_TRUE(naive_collides) << "시나리오 전제: 장애물을 무시하면 정면 직진은 충돌해야 한다";

  // 실제 계획은 그 충돌을 피해야 한다.
  const DwaCommand command = predicting.computeVelocityCommand(state, goal, test_grid.grid(), obstacles);
  ASSERT_EQ(command.status, DwaStatus::kOk);

  const std::vector<TrajectoryPoint> chosen_trajectory =
    predicting.simulateTrajectory(state, command.v, command.w);
  for (std::size_t i = 0; i < chosen_trajectory.size(); ++i) {
    const double t = static_cast<double>(i + 1) * predicting.config().dt;
    const TrajectoryPoint & p = chosen_trajectory[i];
    EXPECT_FALSE(predicting.dynamicObstacleCollides(obstacles, t, p.x, p.y, p.yaw))
      << "step " << i << "에서 예측된 장애물과 충돌";
  }

  // 예측을 끄면(관측 위치에 멈춰 있다고 봄) 이 장애물은 4m보다 멀어 사실상
  // 안 보이므로, 장애물이 전혀 없을 때(EmptyMapMovesTowardGoal)와 같이 정면
  // 최고속을 그대로 고른다.
  DwaConfig no_predict_config;
  no_predict_config.predict_dynamic_obstacles = false;
  DwaPlanner not_predicting(no_predict_config);
  const DwaCommand ignoring_command =
    not_predicting.computeVelocityCommand(state, goal, test_grid.grid(), obstacles);
  ASSERT_EQ(ignoring_command.status, DwaStatus::kOk);
  EXPECT_NEAR(ignoring_command.v, window.max_v, 1e-6);
  EXPECT_NEAR(ignoring_command.w, 0.0, 0.05);
}

// --- rotateToGoalYaw: 최종 접근(위치는 도착, 각도만 맞추는 제자리 회전) ---

TEST(DwaPlannerTest, RotateToGoalYawReachedWithinTolerance)
{
  DwaConfig config;
  DwaPlanner planner(config);

  RobotState state;
  state.yaw = 0.1;
  const DwaCommand command = planner.rotateToGoalYaw(state, /*goal_yaw=*/0.3, /*yaw_tolerance=*/0.25);

  EXPECT_EQ(command.status, DwaStatus::kGoalReached);
  EXPECT_NEAR(command.v, 0.0, 1e-9);
  EXPECT_NEAR(command.w, 0.0, 1e-9);
}

TEST(DwaPlannerTest, RotateToGoalYawTurnsTowardShorterDirection)
{
  DwaConfig config;
  DwaPlanner planner(config);
  RobotState state;  // yaw = 0, w = 0

  const DwaCommand positive = planner.rotateToGoalYaw(state, /*goal_yaw=*/1.0, /*yaw_tolerance=*/0.01);
  ASSERT_EQ(positive.status, DwaStatus::kOk);
  EXPECT_NEAR(positive.v, 0.0, 1e-9);
  EXPECT_GT(positive.w, 0.0);

  const DwaCommand negative = planner.rotateToGoalYaw(state, /*goal_yaw=*/-1.0, /*yaw_tolerance=*/0.01);
  ASSERT_EQ(negative.status, DwaStatus::kOk);
  EXPECT_LT(negative.w, 0.0);
}

TEST(DwaPlannerTest, RotateToGoalYawRespectsAngularAccelerationLimit)
{
  DwaConfig config;
  DwaPlanner planner(config);
  RobotState state;  // w = 0

  // 오차가 커도(여기선 거의 180도) 한 스텝에서는 acc_w*dt만큼만 바뀐다.
  const DwaCommand command = planner.rotateToGoalYaw(state, /*goal_yaw=*/3.0, /*yaw_tolerance=*/0.01);
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_NEAR(command.w, config.acc_w * config.dt, 1e-9);
}

TEST(DwaPlannerTest, RotateToGoalYawDoesNotExceedMaxAngularVelocity)
{
  DwaConfig config;
  DwaPlanner planner(config);
  RobotState state;
  state.w = config.max_w;  // 이미 최고 각속도로 돌고 있다

  const DwaCommand command = planner.rotateToGoalYaw(state, /*goal_yaw=*/3.0, /*yaw_tolerance=*/0.01);
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_LE(std::abs(command.w), config.max_w + 1e-9);
}

TEST(DwaPlannerTest, RotateToGoalYawConvergesWithoutOvershootOscillation)
{
  // 제자리 회전을 반복 적용하면 허용 오차 안에 들어와서 멈추고, 그 과정에서
  // 목표 각도를 크게 지나쳐 반대로 다시 도는 진동이 없어야 한다(등가감속으로
  // 정확히 서는 각속도를 고르기 때문 - speed_cap_for_stop과 같은 발상).
  DwaConfig config;
  DwaPlanner planner(config);
  constexpr double kYawTolerance = 0.05;
  constexpr double kGoalYaw = 1.0;  // 약 57도, 사용자가 겪은 34.7도 오차보다 크게 잡음

  RobotState state;
  double max_abs_error_after_first_entering_tolerance = 0.0;
  bool reached = false;

  for (int i = 0; i < 200; ++i) {
    const DwaCommand command = planner.rotateToGoalYaw(state, kGoalYaw, kYawTolerance);
    state.w = command.w;
    state.yaw += state.w * config.dt;

    if (command.status == DwaStatus::kGoalReached) {
      reached = true;
      break;
    }
    // 허용 오차의 2배를 넘겨 반대쪽으로 지나가면(오버슈트해서 되돌아오는 진동) 실패로 본다.
    const double error = std::abs(kGoalYaw - state.yaw);
    max_abs_error_after_first_entering_tolerance =
      std::max(max_abs_error_after_first_entering_tolerance, error);
  }

  EXPECT_TRUE(reached) << "200스텝 안에 허용 오차 안으로 들어오지 못함";
  EXPECT_LE(max_abs_error_after_first_entering_tolerance, kGoalYaw + kYawTolerance)
    << "목표 각도를 지나쳐 반대쪽으로 넘어감(오버슈트)";
}

namespace
{

// 허용 속도 테스트용: 직진만(max_w=0) 남겨 장애물까지 거리와 선속도만 보게 하고,
// 선속도는 촘촘히(0.01 m/s 간격) 샘플링한다.
DwaConfig makeStraightOnlyConfig(double acc_v)
{
  DwaConfig config;
  config.acc_v = acc_v;
  config.max_w = 0.0;
  config.w_samples = 1;
  config.v_samples = 21;
  return config;
}

// 로봇(x=1.0, 앞면 x=1.3)에서 앞면까지 gap(m) 떨어진 벽. gap이 음수면 벽 없음.
// 목표는 벽 너머 정면이라 모든 직진 궤적의 heading 비용이 같다.
DwaCommand driveTowardWall(const DwaConfig & config, double gap, double v0 = 1.0)
{
  TestGrid test_grid(200, 80, 0.05);
  if (gap >= 0.0) {
    const double wall_x = 1.3 + gap;
    test_grid.setRegion(wall_x + 0.01, wall_x + 0.2, 1.5, 2.5, 254);
  }
  DwaPlanner planner(config);
  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  state.v = v0;
  return planner.computeVelocityCommand(state, Point2D{8.0, 2.0}, test_grid.grid());
}

}  // namespace

TEST(DwaPlannerTest, AdmissibleVelocityDropsFastSamplesNearObstacle)
{
  const DwaConfig config = makeStraightOnlyConfig(1.0);
  const DwaPlanner planner(config);
  RobotState state;
  state.v = 1.0;
  const DynamicWindow window = planner.computeDynamicWindow(state);  // [0.9, 1.1]

  // 장애물이 없으면 윈도우 최고 속도.
  const DwaCommand open = driveTowardWall(config, -1.0);
  ASSERT_EQ(open.status, DwaStatus::kOk);
  EXPECT_NEAR(open.v, window.max_v, 1e-9);

  // 앞면 0.5m 앞 벽: 윈도우 안 모든 직진 궤적이 sim_time 안에 닿지만, sqrt(2*acc_v*dist)
  // 이하인 샘플은 설 수 있어 남는다. 그보다 빠른 샘플은 빠진다.
  const DwaCommand near = driveTowardWall(config, 0.5);
  ASSERT_EQ(near.status, DwaStatus::kOk);
  EXPECT_LT(near.v, window.max_v - 0.05);
  // 풋프린트 샘플링 간격(해상도/2)만큼 여유를 둔다.
  EXPECT_LE(near.v, planner.admissibleVelocity(0.5 + 0.025) + 1e-9);
  EXPECT_GE(near.v, window.min_v);
}

TEST(DwaPlannerTest, AdmissibleVelocityShrinksWithSmallerAcceleration)
{
  EXPECT_LT(DwaPlanner(makeStraightOnlyConfig(0.7)).admissibleVelocity(1.0),
    DwaPlanner(makeStraightOnlyConfig(1.0)).admissibleVelocity(1.0));
  EXPECT_LT(DwaPlanner(makeStraightOnlyConfig(1.0)).admissibleVelocity(1.0),
    DwaPlanner(makeStraightOnlyConfig(1.3)).admissibleVelocity(1.0));
  EXPECT_NEAR(DwaPlanner(makeStraightOnlyConfig(0.5)).admissibleVelocity(1.0), 1.0, 1e-12);

  // 같은 벽(앞면 1.05m 앞)에서 acc_v=1.0이면 윈도우 최고 속도(1.1)도 설 수 있어 그대로
  // 고르고, acc_v=0.5면 허용 최대 속도(~1.0)가 윈도우 최고 속도(1.05)보다 낮아 더
  // 느린 샘플을 고른다.
  const DwaCommand strong = driveTowardWall(makeStraightOnlyConfig(1.0), 1.05);
  const DwaCommand weak = driveTowardWall(makeStraightOnlyConfig(0.5), 1.05);
  ASSERT_EQ(strong.status, DwaStatus::kOk);
  ASSERT_EQ(weak.status, DwaStatus::kOk);
  EXPECT_NEAR(strong.v, 1.1, 1e-9);
  EXPECT_LT(weak.v, 1.05 - 1e-6);
  EXPECT_LE(weak.v, DwaPlanner(makeStraightOnlyConfig(0.5)).admissibleVelocity(1.05 + 0.025));
  EXPECT_LT(weak.v, strong.v);
}

TEST(DwaPlannerTest, AdmissibleVelocityDisabledKeepsPreviousBehavior)
{
  DwaConfig config = makeStraightOnlyConfig(1.0);
  config.use_admissible_velocity = false;

  // 이전 동작: sim_time 안에서 닿는 궤적은 설 수 있어도 모두 뺀다.
  const DwaCommand near = driveTowardWall(config, 0.5);
  EXPECT_EQ(near.status, DwaStatus::kNoValidTrajectory);
  EXPECT_EQ(near.v, 0.0);
  EXPECT_EQ(near.w, 0.0);

  // 호라이즌 안에 장애물이 없으면 켜든 끄든 같다(기본 acc_v·sim_time이면 2*acc_v*sim_time
  // > max_v라 허용 조건이 늘 성립).
  DwaConfig on_config = config;
  on_config.use_admissible_velocity = true;
  const DwaCommand off_open = driveTowardWall(config, -1.0);
  const DwaCommand on_open = driveTowardWall(on_config, -1.0);
  EXPECT_EQ(off_open.status, on_open.status);
  EXPECT_EQ(off_open.v, on_open.v);
  EXPECT_EQ(off_open.w, on_open.w);

  // 기존 회피 시나리오(FrontObstacleTurnsAway)를 끄고 돌리면 고른 궤적이 충돌하지 않는다.
  TestGrid test_grid(160, 80, 0.05);
  test_grid.setRegion(2.6, 2.8, 1.7, 2.3, 254);
  DwaConfig legacy;
  legacy.use_admissible_velocity = false;
  const DwaPlanner planner(legacy);
  RobotState state;
  state.x = 0.5;
  state.y = 2.0;
  state.v = 1.0;
  const DwaCommand command =
    planner.computeVelocityCommand(state, Point2D{6.0, 2.0}, test_grid.grid());
  ASSERT_EQ(command.status, DwaStatus::kOk);
  for (const TrajectoryPoint & point : planner.simulateTrajectory(state, command.v, command.w)) {
    EXPECT_FALSE(planner.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw));
  }
}

namespace
{

struct GoalApproachResult
{
  bool reached = false;
  double reach_distance = 0.0;      // kGoalReached를 받은 시점의 목표까지 거리
  double reach_speed = 0.0;         // 그 직전 명령 선속도
  double stop_x = 0.0;              // 그 뒤 acc_v로 감속해 실제로 서는 x
  double slowdown_distance = 0.0;   // 선속도가 처음 시작 속도보다 줄어든 시점의 목표까지 거리
};

// 빈 지도에서 x축을 따라 v0로 달리다 정면의 최종 목표(goal_x, 2.0)에 다가가는 닫힌 루프.
// 명령을 그대로 따른다고 보고(이상 추종) 상태를 적분한다. kGoalReached가 나오면 거기서
// v=0 명령을 받은 로봇이 acc_v로 감속해 서는 위치까지 계산한다.
GoalApproachResult approachGoal(const DwaConfig & config, double v0, double goal_x, bool pass_final)
{
  TestGrid test_grid(200, 40, 0.1);  // 20m x 4m, 빈 지도라 거칠게(풋프린트 샘플 수를 줄여 테스트 시간 단축)
  const DwaPlanner planner(config);
  const Point2D goal{goal_x, 2.0};

  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  state.v = v0;

  GoalApproachResult result;
  bool slowed = false;
  for (int step = 0; step < 1000; ++step) {
    const DwaCommand command = planner.computeVelocityCommand(
      state, goal, test_grid.grid(), {},
      pass_final ? std::optional<Point2D>(goal) : std::nullopt);
    const double distance = std::hypot(goal.x - state.x, goal.y - state.y);
    if (command.status == DwaStatus::kGoalReached) {
      result.reached = true;
      result.reach_distance = distance;
      result.reach_speed = state.v;
      result.stop_x = state.x + state.v * state.v / (2.0 * config.acc_v);
      break;
    }
    if (command.status != DwaStatus::kOk) {
      break;
    }
    if (!slowed && command.v < v0 - 1e-6) {
      slowed = true;
      result.slowdown_distance = distance;
    }
    state.x += command.v * std::cos(state.yaw) * config.dt;
    state.y += command.v * std::sin(state.yaw) * config.dt;
    state.yaw += command.w * config.dt;
    state.v = command.v;
    state.w = command.w;
  }
  return result;
}

}  // namespace

TEST(DwaPlannerTest, GoalSlowdownStopsWithinToleranceFromFullSpeed)
{
  DwaConfig config = makeFastConfig();
  config.v_samples = 11;
  const double goal_x = 9.0;

  const GoalApproachResult slowed = approachGoal(config, config.max_v, goal_x, true);
  ASSERT_TRUE(slowed.reached);
  EXPECT_LE(slowed.reach_distance, config.goal_tolerance + 1e-9);
  // v=0 명령 뒤 acc_v로 서도 목표를 tolerance 넘게 지나치지 않는다.
  EXPECT_LE(slowed.stop_x, goal_x + config.goal_tolerance);
  EXPECT_LE(slowed.reach_speed, std::sqrt(2.0 * config.acc_v * 0.3));
}

TEST(DwaPlannerTest, GoalSlowdownStartsEarlierWithSmallerAcceleration)
{
  // heading 항이 이미 궤적 끝이 목표를 넘지 않게(v <= d_goal/sim_time, 2 m/s면 4m 앞부터)
  // 줄이므로, 목표 앞 감속(v^2/(2*acc_v) 앞부터)이 먼저 걸리려면 v0 > 2*acc_v*sim_time
  // 이어야 한다 - 그 속도는 허용 속도 조건이 빼므로 여기서는 끄고 acc_v를 작게 준다
  // (navigation.md 4-7절).
  DwaConfig strong = makeFastConfig();
  strong.v_samples = 11;
  strong.use_admissible_velocity = false;
  strong.acc_v = 0.4;
  DwaConfig weak = strong;
  weak.acc_v = 0.25;

  const GoalApproachResult strong_result = approachGoal(strong, 2.0, 14.0, true);
  const GoalApproachResult weak_result = approachGoal(weak, 2.0, 14.0, true);
  ASSERT_TRUE(strong_result.reached);
  ASSERT_TRUE(weak_result.reached);
  // 2 m/s에서 서는 거리 v^2/(2*acc_v): acc_v=0.4면 5m, 0.25면 8m 앞부터 줄인다.
  EXPECT_NEAR(strong_result.slowdown_distance, 5.0, 0.25);
  EXPECT_NEAR(weak_result.slowdown_distance, 8.0, 0.25);
  EXPECT_GT(weak_result.slowdown_distance, strong_result.slowdown_distance);
  EXPECT_LE(strong_result.stop_x, 14.0 + strong.goal_tolerance);
  EXPECT_LE(weak_result.stop_x, 14.0 + weak.goal_tolerance);

  // 끄면 둘 다 heading 항 때문에 같은 지점(~4m 앞)부터 줄인다.
  strong.use_goal_slowdown = false;
  weak.use_goal_slowdown = false;
  EXPECT_NEAR(approachGoal(strong, 2.0, 14.0, true).slowdown_distance, 4.0, 0.25);
  EXPECT_NEAR(approachGoal(weak, 2.0, 14.0, true).slowdown_distance, 4.0, 0.25);
}

TEST(DwaPlannerTest, GoalSlowdownDisabledKeepsPreviousBehavior)
{
  DwaConfig off = makeFastConfig();
  off.use_goal_slowdown = false;
  const DwaPlanner planner(off);

  TestGrid test_grid(160, 80, 0.05);
  test_grid.setRegion(2.6, 2.8, 1.7, 2.3, 254);
  RobotState state;
  state.x = 0.5;
  state.y = 2.0;
  state.v = 1.9;
  const Point2D goal{3.5, 2.0};  // 가까운 최종 목표 - 감속이 켜져 있으면 제한이 걸릴 거리

  const DwaCommand with_final = planner.computeVelocityCommand(
    state, goal, test_grid.grid(), {}, goal);
  const DwaCommand without_final = planner.computeVelocityCommand(state, goal, test_grid.grid());
  EXPECT_EQ(with_final.status, without_final.status);
  EXPECT_EQ(with_final.v, without_final.v);
  EXPECT_EQ(with_final.w, without_final.w);

  // 켜도 final_goal을 안 넘기면 기존과 같다.
  const DwaPlanner on_planner(makeFastConfig());
  const DwaCommand on_without_final =
    on_planner.computeVelocityCommand(state, goal, test_grid.grid());
  EXPECT_EQ(on_without_final.status, without_final.status);
  EXPECT_EQ(on_without_final.v, without_final.v);
  EXPECT_EQ(on_without_final.w, without_final.w);
}

namespace
{

// (x0, y) -> (x1, y) 직선 경로, 0.05m 간격(전역 경로 해상도와 비슷하게).
std::vector<Point2D> straightPath(double x0, double x1, double y)
{
  std::vector<Point2D> path;
  for (double x = x0; x <= x1 + 1e-9; x += 0.05) {
    path.push_back(Point2D{x, y});
  }
  path.back().x = x1;
  return path;
}

struct PathRunResult
{
  bool reached = false;
  double max_v = 0.0;
  double stop_x = 0.0;              // kGoalReached 뒤 acc_v로 감속해 서는 x
  double slowdown_distance = 0.0;   // 최고 속도보다 0.05 m/s 넘게 줄어든 시점의 경로 끝까지 거리
  int steps = 0;                    // kGoalReached까지 걸린 제어 주기 수
};

// 빈 지도에서 직선 경로를 따라가는 닫힌 루프(명령을 그대로 따른다고 보고 적분).
// start_y·start_yaw: 경로(y=2.0) 기준 옆 오차·방향 오차로 출발한다(연구 2 환경처럼 경로 위에 딱 있지 않음).
PathRunResult followStraightPath(
  const DwaConfig & config, double v0, double goal_x, double start_y = 2.0, double start_yaw = 0.0)
{
  TestGrid test_grid(200, 40, 0.1);  // 20m x 4m, 빈 지도라 거칠게(테스트 시간 단축)
  const DwaPlanner planner(config);
  const std::vector<Point2D> path = straightPath(1.0, goal_x, 2.0);

  RobotState state;
  state.x = 1.0;
  state.y = start_y;
  state.yaw = start_yaw;
  state.v = v0;

  PathRunResult result;
  result.max_v = v0;
  bool slowed = false;
  for (int step = 0; step < 2000; ++step) {
    const DwaCommand command = planner.computeVelocityCommand(state, path, test_grid.grid());
    if (command.status == DwaStatus::kGoalReached) {
      result.reached = true;
      result.stop_x = state.x + state.v * state.v / (2.0 * config.acc_v);
      result.steps = step;
      break;
    }
    if (command.status != DwaStatus::kOk) {
      break;
    }
    // 경로를 다시 맞추는 작은 조향(w)으로 생기는 0.01 m/s 수준의 흔들림은 감속으로 안 본다.
    if (!slowed && command.v < result.max_v - 0.05) {
      slowed = true;
      result.slowdown_distance = goal_x - state.x;
    }
    result.max_v = std::max(result.max_v, command.v);
    state.x += command.v * std::cos(state.yaw) * config.dt;
    state.y += command.v * std::sin(state.yaw) * config.dt;
    state.yaw += command.w * config.dt;
    state.v = command.v;
    state.w = command.w;
  }
  return result;
}

}  // namespace

TEST(DwaPlannerTest, AdaptiveLookaheadReachesNearMaxSpeedOnStraightPath)
{
  DwaConfig config = makeFastConfig();
  config.v_samples = 11;

  const PathRunResult adaptive = followStraightPath(config, 0.0, 15.0);
  ASSERT_TRUE(adaptive.reached);
  EXPECT_GE(adaptive.max_v, 0.95 * config.max_v);
  EXPECT_LE(adaptive.stop_x, 15.0 + config.goal_tolerance);

  // 고정 기준점이면 궤적 끝이 lookahead_distance 앞 점을 넘는 순간 heading 비용이 최대라
  // lookahead_distance/sim_time(0.25 m/s) 근처에 묶인다(Gazebo에서 0.246 m/s).
  config.adaptive_lookahead = false;
  const PathRunResult fixed = followStraightPath(config, 0.0, 15.0);
  EXPECT_LE(fixed.max_v, config.lookahead_distance / config.sim_time + 0.1);
}

TEST(DwaPlannerTest, AdaptiveLookaheadUsesFixedTargetWhenStopped)
{
  // 기준점은 궤적 끝보다 lookahead_distance 앞(궤적 길이 + lookahead_distance)이라, 정지(v=0)에서는
  // 고정 lookahead 점과 같다. (#216 전에는 |v|*sim_time <= lookahead_distance인 느린 속도 전체에서
  // 같았다 - 그 규칙이 궤적 끝과 기준점을 붙여 놓아 속도가 떨어지는 원인이었다.)
  const std::vector<Point2D> path = {
    {1.0, 2.0}, {1.2, 2.0}, {1.4, 2.0}, {1.4, 2.2}, {1.4, 2.4}, {1.4, 2.6}, {1.4, 2.8}};
  EXPECT_EQ(pathPointAhead(path, 1.0, 2.0, 0.5).point.x, 1.4);
  EXPECT_EQ(pathPointAhead(path, 1.0, 2.0, 0.5).point.y, 2.2);
  EXPECT_EQ(pathPointAhead(path, 1.0, 2.0, 10.0).point.y, 2.8);
  EXPECT_TRUE(pathPointAhead(path, 1.0, 2.0, 10.0).at_path_end);

  TestGrid test_grid(80, 80, 0.05);
  test_grid.setRegion(1.7, 1.9, 1.6, 2.4, 254);
  DwaConfig on = makeFastConfig();
  on.acc_v = 1e-6;  // 동적 윈도우가 사실상 v=0 하나 - 궤적 길이 0
  DwaConfig off = on;
  off.adaptive_lookahead = false;
  for (const double v0 : {0.0}) {
    RobotState state;
    state.x = 1.0;
    state.y = 2.0;
    state.v = v0;
    const DwaCommand a = DwaPlanner(on).computeVelocityCommand(state, path, test_grid.grid());
    const DwaCommand b = DwaPlanner(off).computeVelocityCommand(state, path, test_grid.grid());
    EXPECT_EQ(a.status, b.status) << "v0=" << v0;
    EXPECT_EQ(a.v, b.v) << "v0=" << v0;
    EXPECT_EQ(a.w, b.w) << "v0=" << v0;
  }
}

TEST(DwaPlannerTest, AdaptiveLookaheadDisabledMatchesFixedWaypointCall)
{
  // 끄면 경로 버전이 이전 플러그인 흐름(lookahead_distance 앞 점 하나 + 최종 목표)과 같다.
  TestGrid test_grid(160, 80, 0.05);
  test_grid.setRegion(2.6, 2.8, 1.7, 2.3, 254);
  DwaConfig off = makeFastConfig();
  off.adaptive_lookahead = false;
  const DwaPlanner planner(off);
  const std::vector<Point2D> path = straightPath(0.5, 6.0, 2.0);

  for (const double v0 : {0.0, 0.5, 1.0, 1.9}) {
    RobotState state;
    state.x = 0.5;
    state.y = 2.0;
    state.v = v0;
    const Point2D waypoint = pathPointAhead(path, state.x, state.y, off.lookahead_distance).point;
    const DwaCommand by_path = planner.computeVelocityCommand(state, path, test_grid.grid());
    const DwaCommand by_point = planner.computeVelocityCommand(
      state, waypoint, test_grid.grid(), {}, path.back());
    EXPECT_EQ(by_path.status, by_point.status) << "v0=" << v0;
    EXPECT_EQ(by_path.v, by_point.v) << "v0=" << v0;
    EXPECT_EQ(by_path.w, by_point.w) << "v0=" << v0;
  }
}

TEST(DwaPlannerTest, AdaptiveLookaheadLetsGoalSlowdownTakeEffect)
{
  // 켜면 경로 끝 앞의 속도를 heading이 아니라 목표 앞 감속(sqrt(2*acc_v*d))이 정한다:
  // 2 m/s에서 v^2/(2*acc_v) 앞부터 줄인다 - acc_v=1.0이면 2m, 0.6이면 3.33m. 감속 판정은
  // 0.05 m/s 넘게 줄었을 때라 (2-0.05)^2/(2*acc_v), 한 스텝(0.2m) 오차를 둔다.
  DwaConfig strong = makeFastConfig();
  strong.v_samples = 11;
  strong.acc_v = 1.0;
  DwaConfig weak = strong;
  weak.acc_v = 0.6;

  const PathRunResult strong_result = followStraightPath(strong, 2.0, 15.0);
  const PathRunResult weak_result = followStraightPath(weak, 2.0, 15.0);
  ASSERT_TRUE(strong_result.reached);
  ASSERT_TRUE(weak_result.reached);
  EXPECT_NEAR(strong_result.slowdown_distance, 1.95 * 1.95 / 2.0, 0.25);
  EXPECT_NEAR(weak_result.slowdown_distance, 1.95 * 1.95 / 1.2, 0.25);
  EXPECT_GT(weak_result.slowdown_distance, strong_result.slowdown_distance);
  EXPECT_LE(strong_result.stop_x, 15.0 + strong.goal_tolerance);
  EXPECT_LE(weak_result.stop_x, 15.0 + weak.goal_tolerance);
}

TEST(DwaPlannerTest, AdaptiveLookaheadKeepsSpeedWhenSlightlyOffPath)
{
  // #216 증상 1(연구 2 S2 0 kg): 경로에서 몇 cm 옆·몇 도 틀어진 채 달리면 1.5 m/s에서 0.15 m/s로 떨어져
  // 기어갔다. heading 기준점을 궤적 길이(|v|*sim_time)만큼 앞의 경로 점으로 잡아, 궤적 끝과 기준점이 거의
  // 같은 곳이라 끝 -> 기준점 방향이 경로 점 간격(0.05 m)·옆 오차 몇 mm에 따라 크게 튀었다 - 빠른 샘플의
  // heading 비용만 들쭉날쭉 커져 속도 비용(0.1)으로는 못 이겼다.
  DwaConfig config = makeFastConfig();
  config.v_samples = 11;
  config.acc_v = 2.07;  // 연구 2 0 kg
  const PathRunResult result = followStraightPath(config, 1.5, 15.0, 2.0 - 0.02, 0.02);
  ASSERT_TRUE(result.reached);
  EXPECT_GE(result.max_v, 0.95 * config.max_v);
  // 14 m를 최고 속도 근처로 가면 10초 안쪽이다(기어가면 수십 초)
  EXPECT_LT(result.steps * config.dt, 12.0) << "steps=" << result.steps;
  EXPECT_LE(result.stop_x, 15.0 + config.goal_tolerance);
}

TEST(DwaPlannerTest, DynamicObstacleAlreadyWithinMarginAllowsTrajectoriesMovingAway)
{
  // #216 증상 2(연구 2 S1 가로지름): 장애물이 이미 안전 여유(0.2 m) 안에 있으면 궤적 첫 점부터 여유
  // 안이라, 멀어지는 궤적까지 모두 버려져 장애물이 비킬 때까지 거의 멈춰 있었다.
  TestGrid test_grid(160, 80, 0.05);
  const DwaPlanner planner(makeFastConfig());

  RobotState state;
  state.x = 2.0;
  state.y = 2.0;
  state.v = 0.5;
  const Point2D goal{6.0, 2.0};

  // 왼쪽 옆에 0.1 m 떨어져 서 있는 장애물(로봇 중심보다 조금 뒤) - 앞으로 가면 멀어진다
  DynamicObstacle beside;
  beside.radius = 0.25;
  beside.x = 2.0 - 0.1;
  beside.y = 2.0 + 0.2 + 0.1 + beside.radius;
  const DwaCommand away =
    planner.computeVelocityCommand(state, goal, test_grid.grid(), {beside});
  ASSERT_EQ(away.status, DwaStatus::kOk);
  EXPECT_GT(away.v, 0.5);
  // 고른 궤적은 지금(0.1 m)보다 장애물에 가까워지지 않는다
  for (const TrajectoryPoint & point : planner.simulateTrajectory(state, away.v, away.w)) {
    const double dx = beside.x - point.x;
    const double dy = beside.y - point.y;
    const double lx = std::abs(dx * std::cos(point.yaw) + dy * std::sin(point.yaw)) - 0.3;
    const double ly = std::abs(-dx * std::sin(point.yaw) + dy * std::cos(point.yaw)) - 0.2;
    EXPECT_GE(std::hypot(std::max(0.0, lx), std::max(0.0, ly)) - beside.radius, 0.1 - 1e-3);
  }

  // 정면 0.1 m 앞에 있는 장애물은 앞으로 가면 가까워지므로 여전히 막는다
  DynamicObstacle ahead;
  ahead.radius = 0.25;
  ahead.x = 2.0 + 0.3 + 0.1 + ahead.radius;
  ahead.y = 2.0;
  RobotState stopped = state;
  stopped.v = 0.0;
  // 서 있는 로봇에서 앞으로 가는 샘플은 첫 점부터 가까워지고, 제자리 회전은 앞 모서리가 다가가 궤적 중간에
  // 가까워진다(v≈0 회전 충돌은 버린다) - 지금 거리를 지키는 건 그대로 서 있기(v=0, w=0) 하나다
  const DwaCommand blocked =
    planner.computeVelocityCommand(stopped, goal, test_grid.grid(), {ahead});
  ASSERT_EQ(blocked.status, DwaStatus::kOk);
  EXPECT_EQ(blocked.v, 0.0);
  EXPECT_EQ(blocked.w, 0.0);
}

TEST(DwaPlannerTest, PredictedObstacleApproachingWithinMarginLeavesNoValidTrajectory)
{
  // 예측(predict_dynamic_obstacles) 켬, 이미 여유 안(왼쪽 0.1 m)에서 0.5 m/s로 다가오는 장애물: 서 있어도 첫
  // 점(0.1 s 뒤)에 장애물이 0.05 m 더 다가오고, 로봇은 한 주기에 그만큼 못 비킨다 - 멀어지는 궤적이 없어
  // kNoValidTrajectory(정지)다. "지금보다 가까워지지 않으면 허용"이 다가오는 장애물까지 허용하면 안 된다.
  TestGrid test_grid(160, 80, 0.05);
  const DwaPlanner planner(makeFastConfig());
  ASSERT_TRUE(planner.config().predict_dynamic_obstacles);
  RobotState state;
  state.x = 2.0;
  state.y = 2.0;
  DynamicObstacle approaching;
  approaching.radius = 0.25;
  approaching.x = 2.0;
  approaching.y = 2.0 + 0.2 + 0.1 + approaching.radius;
  approaching.vy = -0.5;
  const DwaCommand command =
    planner.computeVelocityCommand(state, Point2D{6.0, 2.0}, test_grid.grid(), {approaching});
  EXPECT_EQ(command.status, DwaStatus::kNoValidTrajectory);
}

TEST(DwaPlannerTest, WithinMarginObstaclePrefersTrajectoriesMovingAway)
{
  // #228 연우 리뷰: 이미 여유 안인 장애물은 궤적을 따라 잰 최소 거리가 늘 여유 안이라 거리비용이 모든 궤적에서
  // 1.0이었다 - 나란히 가는 궤적과 멀어지는 궤적을 가르지 못했다. 궤적 끝 거리로 매기면 멀어질수록 싸다.
  // 다른 비용 항을 끄고 동적 거리비용만 남겨 본다(전에는 모든 후보가 같아 첫 후보, 직진을 골랐다).
  TestGrid test_grid(160, 80, 0.05);
  DwaConfig config = makeFastConfig();
  config.heading_weight = 0.0;
  config.clearance_weight = 0.0;
  config.velocity_weight = 0.0;
  const DwaPlanner planner(config);
  RobotState state;
  state.x = 2.0;
  state.y = 2.0;
  state.v = 0.5;
  // 오른쪽 0.1 m(여유 안)에서 로봇과 같은 속도로 나란히 가는 장애물
  DynamicObstacle alongside;
  alongside.radius = 0.25;
  alongside.x = 2.0;
  alongside.y = 2.0 - 0.2 - 0.1 - alongside.radius;
  alongside.vx = 0.5;
  const DwaCommand command =
    planner.computeVelocityCommand(state, Point2D{6.0, 2.0}, test_grid.grid(), {alongside});
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_GT(command.w, 0.0);  // 왼쪽(장애물 반대쪽)으로 꺾어 멀어진다
}

TEST(DwaPlannerTest, GoalSlowdownStopsBeforeGoalAccountingForControlPeriod)
{
  // #216: 상한을 sqrt(2*acc_v*d)(연속 시간)로 두면 매 주기 그 속도로 dt만큼 더 가는 만큼 상한보다 늦게
  // 줄어, 최대로 감속해도 목표를 지나쳤다(연구 2 S2 25 kg, 고친 heading으로 빨리 도착하자 드러남).
  // 상한이 |v|*dt + v^2/(2*acc_v) <= d이면 도착 판정 때 그 자리에서 acc_v로 서도 목표를 안 넘는다.
  DwaConfig config = makeFastConfig();
  config.v_samples = 21;
  config.acc_v = 1.113;  // 연구 2 25 kg
  const PathRunResult result = followStraightPath(config, 2.0, 12.0);
  ASSERT_TRUE(result.reached);
  EXPECT_LE(result.stop_x, 12.0 + 1e-6);
  EXPECT_NEAR(DwaPlanner(config).goalStoppingVelocity(0.0), 0.0, 1e-12);
  const double v = DwaPlanner(config).goalStoppingVelocity(2.0);
  EXPECT_NEAR(v * config.dt + v * v / (2.0 * config.acc_v), 2.0, 1e-9);
}

namespace
{

constexpr double kTestPi = 3.14159265358979323846;

double wrapAngle(double a)
{
  while (a > kTestPi) {a -= 2.0 * kTestPi;}
  while (a < -kTestPi) {a += 2.0 * kTestPi;}
  return a;
}

// (x0, y0) -> (x1, y1) 직선을 0.05 m 간격으로 path 뒤에 잇는다
void appendLine(std::vector<Point2D> & path, double x0, double y0, double x1, double y1)
{
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
  for (int i = path.empty() ? 0 : 1; i <= n; ++i) {
    const double r = static_cast<double>(i) / n;
    path.push_back(Point2D{x0 + (x1 - x0) * r, y0 + (y1 - y0) * r});
  }
}

// 창고 스택 기준 설정(max_v 0.5 m/s, nav2_params.yaml)으로 촘촘하게 - 좁은 통로·회전 테스트용
DwaConfig makeStackConfig()
{
  DwaConfig config;
  config.max_v = 0.5;
  config.v_samples = 6;
  config.w_samples = 21;
  return config;
}

struct LoopResult
{
  bool reached = false;
  bool collided = false;
  int steps = 0;
  int rotate_steps = 0;              // rotatingInPlace()였던 주기 수
  double rotate_displacement = 0.0;  // 회전 모드 동안 출발점에서 가장 멀어진 거리
  double first_v = 0.0;
  double first_w = 0.0;
  RobotState final_state;
};

// 경로를 따라가는 닫힌 루프(명령 그대로 추종). 매 주기 풋프린트 충돌을 검사한다.
LoopResult runPath(
  const DwaConfig & config, const Grid & grid, const std::vector<Point2D> & path, RobotState state,
  int max_steps)
{
  const DwaPlanner planner(config);
  LoopResult result;
  const double x0 = state.x;
  const double y0 = state.y;
  for (int step = 0; step < max_steps; ++step) {
    const DwaCommand command = planner.computeVelocityCommand(state, path, grid);
    if (step == 0) {
      result.first_v = command.v;
      result.first_w = command.w;
    }
    if (command.status == DwaStatus::kGoalReached) {
      result.reached = true;
      break;
    }
    if (command.status != DwaStatus::kOk) {
      break;
    }
    if (planner.rotatingInPlace()) {
      ++result.rotate_steps;
      result.rotate_displacement =
        std::max(result.rotate_displacement, std::hypot(state.x - x0, state.y - y0));
    }
    state.x += command.v * std::cos(state.yaw) * config.dt;
    state.y += command.v * std::sin(state.yaw) * config.dt;
    state.yaw = wrapAngle(state.yaw + command.w * config.dt);
    state.v = command.v;
    state.w = command.w;
    result.steps = step + 1;
    if (planner.footprintCollides(grid, state.x, state.y, state.yaw)) {
      result.collided = true;
      break;
    }
  }
  result.final_state = state;
  return result;
}

}  // namespace

TEST(DwaPlannerTest, RotatesInPlaceWhenStartingFarOffHeading)
{
  // 10/14 하연 정리: v≈0이면 heading 비용이 1.0으로 고정돼 제자리 회전이 점수에서 이득이 없어, 목표가
  // 뒤나 옆이면 크게 도는 전진 호를 골랐다. 회전 모드에서는 제자리에서 목표 쪽으로 돈 뒤 출발한다.
  TestGrid test_grid(200, 80, 0.05);  // 10m x 4m, 빈 지도
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 8.0, 2.0);
  for (const double yaw : {kTestPi / 2.0, kTestPi}) {
    RobotState state;
    state.x = 1.0;
    state.y = 2.0;
    state.yaw = yaw;
    const LoopResult result = runPath(makeStackConfig(), test_grid.grid(), path, state, 800);
    EXPECT_TRUE(result.reached) << "yaw=" << yaw;
    EXPECT_FALSE(result.collided) << "yaw=" << yaw;
    EXPECT_GT(result.rotate_steps, 0) << "yaw=" << yaw;
    // 회전 모드 동안 제자리(출발점 근처)에 있다
    EXPECT_LT(result.rotate_displacement, 0.05) << "yaw=" << yaw;
    EXPECT_NEAR(result.first_v, 0.0, 1e-9) << "yaw=" << yaw;
    if (yaw < kTestPi - 1e-6) {
      EXPECT_LT(result.first_w, 0.0);  // 목표(+x)는 오른쪽 - 시계 방향으로 돈다
    } else {
      EXPECT_NE(result.first_w, 0.0);
    }
  }

  // 끄면(rotate_in_place_min_angle 0) 예전처럼 v≈0 샘플은 heading 1.0이라 전진 호로 돈다
  DwaConfig off = makeStackConfig();
  off.rotate_in_place_min_angle = 0.0;
  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  state.yaw = kTestPi / 2.0;
  const DwaCommand arc = DwaPlanner(off).computeVelocityCommand(state, path, test_grid.grid());
  ASSERT_EQ(arc.status, DwaStatus::kOk);
  EXPECT_GT(arc.v, 0.0);
}

TEST(DwaPlannerTest, RotateInPlaceModeHasHysteresis)
{
  // 들어갈 때 45°, 나올 때 30° - 그 사이(40°)에서는 지금 모드를 그대로 둔다(회전/전진이 매 주기 떨지 않게)
  TestGrid test_grid(200, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 8.0, 2.0);
  const DwaPlanner planner(makeStackConfig());
  auto at = [&](double yaw_deg) {
      RobotState state;
      state.x = 1.0;
      state.y = 2.0;
      state.yaw = -yaw_deg * kTestPi / 180.0;  // 목표(+x)에서 오른쪽으로 yaw_deg만큼 틀어짐
      return planner.computeVelocityCommand(state, path, test_grid.grid());
    };

  const DwaCommand at40_forward = at(40.0);
  EXPECT_FALSE(planner.rotatingInPlace());  // 아직 45°를 안 넘었다
  EXPECT_GT(at40_forward.v, 0.0);
  at(50.0);
  EXPECT_TRUE(planner.rotatingInPlace());   // 45° 넘음 - 들어감
  const DwaCommand at40_rotate = at(40.0);
  EXPECT_TRUE(planner.rotatingInPlace());   // 30°보다 크다 - 그대로 회전
  EXPECT_NEAR(at40_rotate.v, 0.0, 1e-9);
  EXPECT_GT(at40_rotate.w, 0.0);            // 왼쪽(목표 쪽)으로 돈다
  at(35.0);
  EXPECT_TRUE(planner.rotatingInPlace());
  at(25.0);
  EXPECT_FALSE(planner.rotatingInPlace());  // 30° 이하 - 나옴
  at(40.0);
  EXPECT_FALSE(planner.rotatingInPlace());  // 다시 45°를 넘기 전까지는 전진
}

TEST(DwaPlannerTest, RotationSamplesInsideNarrowCorridorAreFiltered)
{
  // 풋프린트 대각선 0.72 m > 좁은 통로 폭 0.6 m - 통로 안에서 제자리로 크게 돌면 벽에 닿는다. v≈0 샘플도
  // 사각 풋프린트로 매 스텝(yaw가 바뀐 자세) 검사해 걸러야 한다.
  TestGrid test_grid(200, 80, 0.05);
  test_grid.setRegion(0.0, 9.99, 0.0, 1.69, 254);   // 통로 아래 벽(y < 1.70)
  test_grid.setRegion(0.0, 9.99, 2.31, 3.99, 254);  // 통로 위 벽(y >= 2.30)
  const DwaPlanner planner(makeStackConfig());

  RobotState state;
  state.x = 4.0;
  state.y = 2.0;
  state.yaw = 0.0;
  // 크게 도는 제자리 회전은 궤적 중간에 벽에 닿는다(v=0이어도 충돌 검사가 된다)
  bool wide_rotation_hits = false;
  for (const TrajectoryPoint & point : planner.simulateTrajectory(state, 0.0, 1.0)) {
    wide_rotation_hits = wide_rotation_hits ||
      planner.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw);
  }
  EXPECT_TRUE(wide_rotation_hits);

  // 목표가 뒤(-x)라 회전 모드에 들어가지만, 고른 명령은 통로 안에서 벽에 닿지 않는다
  std::vector<Point2D> path;
  appendLine(path, 4.0, 2.0, 1.0, 2.0);
  for (int step = 0; step < 50; ++step) {
    const DwaCommand command = planner.computeVelocityCommand(state, path, test_grid.grid());
    if (command.status != DwaStatus::kOk) {
      break;
    }
    for (const TrajectoryPoint & point : planner.simulateTrajectory(state, command.v, command.w)) {
      ASSERT_FALSE(planner.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw))
        << "step=" << step << " v=" << command.v << " w=" << command.w;
    }
    state.x += command.v * std::cos(state.yaw) * planner.config().dt;
    state.y += command.v * std::sin(state.yaw) * planner.config().dt;
    state.yaw = wrapAngle(state.yaw + command.w * planner.config().dt);
    state.v = command.v;
    state.w = command.w;
    ASSERT_FALSE(planner.footprintCollides(test_grid.grid(), state.x, state.y, state.yaw));
  }
}

TEST(DwaPlannerTest, TurnsNinetyDegreesIntoNarrowCorridor)
{
  // 좁은 통로 ①처럼 넓은 통로(폭 2 m)에서 90° 꺾어 폭 0.6 m 통로로 들어간다 - 꺾는 곳에서 제자리로 돌고
  // 들어가야 한다(통로 안에서는 못 돈다).
  TestGrid test_grid(120, 120, 0.05, 254);        // 6m x 6m, 전부 벽
  test_grid.setRegion(0.0, 5.99, 0.0, 1.99, 0);   // 넓은 통로 y < 2.0
  test_grid.setRegion(2.70, 3.29, 2.0, 5.99, 0);  // 좁은 통로 x ∈ [2.70, 3.30)
  std::vector<Point2D> path;
  appendLine(path, 0.6, 1.0, 3.0, 1.0);
  appendLine(path, 3.0, 1.0, 3.0, 5.0);
  RobotState state;
  state.x = 0.6;
  state.y = 1.0;
  const LoopResult result = runPath(makeStackConfig(), test_grid.grid(), path, state, 1500);
  EXPECT_FALSE(result.collided);
  ASSERT_TRUE(result.reached) << "steps=" << result.steps << " x=" << result.final_state.x
                              << " y=" << result.final_state.y << " yaw=" << result.final_state.yaw;
  EXPECT_GT(result.rotate_steps, 0);
  EXPECT_NEAR(result.final_state.x, 3.0, 0.1);
  EXPECT_GT(result.final_state.y, 4.5);
}

TEST(DwaPlannerTest, FinalApproachRotatesToGoalYawAfterArriving)
{
  // 플러그인(dwa_controller.cpp)처럼: 경로 끝과 xy_goal_tolerance(0.25 m) 안이면 rotateToGoalYaw로 목표
  // 방향까지 제자리 회전한다. 도착 뒤 90° 돌아야 하는 목표에서 제자리로 돌아 허용오차 안에 선다.
  TestGrid test_grid(200, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 5.0, 2.0);
  const DwaConfig config = makeStackConfig();
  const DwaPlanner planner(config);
  constexpr double kXyTolerance = 0.25;
  constexpr double kYawTolerance = 0.25;
  const double goal_yaw = kTestPi / 2.0;

  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  bool done = false;
  bool arrived = false;
  double arrive_x = 0.0;
  double arrive_y = 0.0;
  for (int step = 0; step < 600 && !done; ++step) {
    DwaCommand command;
    if (std::hypot(5.0 - state.x, 2.0 - state.y) <= kXyTolerance) {
      if (!arrived) {
        arrived = true;
        arrive_x = state.x;
        arrive_y = state.y;
      }
      command = planner.rotateToGoalYaw(state, goal_yaw, kYawTolerance);
      command.v = 0.0;
      done = command.status == DwaStatus::kGoalReached;
    } else {
      command = planner.computeVelocityCommand(state, path, test_grid.grid());
      if (command.status == DwaStatus::kGoalReached) {
        command.v = 0.0;
        command.w = 0.0;
      }
    }
    state.x += command.v * std::cos(state.yaw) * config.dt;
    state.y += command.v * std::sin(state.yaw) * config.dt;
    state.yaw = wrapAngle(state.yaw + command.w * config.dt);
    state.v = command.v;
    state.w = command.w;
  }
  ASSERT_TRUE(arrived);
  ASSERT_TRUE(done);
  EXPECT_LE(std::abs(wrapAngle(goal_yaw - state.yaw)), kYawTolerance);
  // 최종 접근 회전은 제자리(도착한 자리에서 안 움직임)
  EXPECT_NEAR(state.x, arrive_x, 1e-9);
  EXPECT_NEAR(state.y, arrive_y, 1e-9);
}

TEST(DwaPlannerTest, LethalCellBesideFootprintKeepsStopAndForwardSamples)
{
  // 센서 노이즈로 발밑 바로 옆(풋프린트 모서리 밖)에 치명 셀이 찍혀도, 도는 동안 모서리가 그 셀을 쓸고
  // 지나는 회전 샘플만 빠지고 정지(v=0, w=0)·전진 샘플은 남는다 - kNoValidTrajectory가 되지 않는다.
  TestGrid test_grid(200, 80, 0.05);
  // 왼쪽 앞 모서리(4.3, 2.2) 안쪽 대각선 셀 [4.25, 4.30) x [2.25, 2.30) - 풋프린트 밖이지만 중심에서 0.354 m로
  // 모서리 회전 반경(0.36 m) 안이라, 왼쪽(반시계)으로 약 11° 돌면 모서리가 닿는다
  test_grid.setRegion(4.25, 4.25, 2.25, 2.25, 254);
  std::vector<Point2D> path;
  appendLine(path, 4.0, 2.0, 8.0, 2.0);
  const DwaPlanner planner(makeStackConfig());
  RobotState state;
  state.x = 4.0;
  state.y = 2.0;
  ASSERT_FALSE(planner.footprintCollides(test_grid.grid(), state.x, state.y, state.yaw));

  bool rotation_hits = false;
  for (const TrajectoryPoint & point : planner.simulateTrajectory(state, 0.0, 0.3)) {
    rotation_hits = rotation_hits ||
      planner.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw);
  }
  EXPECT_TRUE(rotation_hits);  // 왼쪽으로 제자리 회전하면 모서리가 그 셀에 닿는다

  const DwaCommand command = planner.computeVelocityCommand(state, path, test_grid.grid());
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_GT(command.v, 0.0);

  // 목표가 뒤(회전 모드)여도 정지·반대 방향 회전이 남아 kNoValidTrajectory가 아니다
  std::vector<Point2D> behind;
  appendLine(behind, 4.0, 2.0, 1.0, 2.0);
  const DwaCommand turn = planner.computeVelocityCommand(state, behind, test_grid.grid());
  ASSERT_EQ(turn.status, DwaStatus::kOk);
  for (const TrajectoryPoint & point : planner.simulateTrajectory(state, turn.v, turn.w)) {
    EXPECT_FALSE(planner.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw));
  }
}

TEST(DwaPlannerTest, LethalCellUnderFootprintIsNoValidAsBefore)
{
  // 지금 발밑(풋프린트 안)이 이미 치명 셀이면 모든 궤적이 첫 점부터 닿아 kNoValidTrajectory다. 이건 v≈0 회전
  // 규칙(4-9절) 전부터 허용 속도의 "첫 점부터 닿으면 뺀다" 규칙(#188)으로 같았다 - 정지 샘플은 움직이지 않아
  // 첫 점에서 닿거나 끝까지 안 닿으므로 v≈0 회전 규칙(궤적 중간에 닿으면 뺀다)이 정지 샘플을 바꾸지 않는다.
  TestGrid test_grid(200, 80, 0.05);
  test_grid.setRegion(4.10, 4.11, 2.10, 2.11, 254);  // 풋프린트 안 한 셀
  std::vector<Point2D> path;
  appendLine(path, 4.0, 2.0, 8.0, 2.0);
  RobotState state;
  state.x = 4.0;
  state.y = 2.0;
  DwaConfig config = makeStackConfig();
  EXPECT_EQ(DwaPlanner(config).computeVelocityCommand(state, path, test_grid.grid()).status,
    DwaStatus::kNoValidTrajectory);
  config.rotate_in_place_min_angle = 0.0;
  EXPECT_EQ(DwaPlanner(config).computeVelocityCommand(state, path, test_grid.grid()).status,
    DwaStatus::kNoValidTrajectory);
}

TEST(DwaPlannerTest, StoppedBesideDynamicObstacleWithinMarginStillMovesAway)
{
  // #228(d09d947) 상황을 서 있는 로봇(v=0)으로: 왼쪽 옆 0.1 m(안전 여유 0.2 m 안)에 장애물이 있어도 앞으로
  // 멀어지는 궤적이 남는다. v≈0 샘플 가운데 장애물 쪽으로 도는 회전만 "지금보다 가까워짐"으로 빠진다.
  TestGrid test_grid(160, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 2.0, 2.0, 6.0, 2.0);
  const DwaPlanner planner(makeStackConfig());
  RobotState state;
  state.x = 2.0;
  state.y = 2.0;
  DynamicObstacle beside;
  beside.radius = 0.25;
  beside.x = 2.0 - 0.1;
  beside.y = 2.0 + 0.2 + 0.1 + beside.radius;
  const DwaCommand command =
    planner.computeVelocityCommand(state, path, test_grid.grid(), {beside});
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_GT(command.v, 0.0);
  EXPECT_LE(command.w, 1e-9);  // 장애물(왼쪽) 쪽으로 돌지 않는다
}

TEST(DwaPlannerTest, DynamicWindowDoesNotInvertAboveMaxSpeed)
{
  // 지금 속도가 max_v보다 한 주기 감속 이상 빠르면(v > max_v + acc_v*dt) min_v > max_v로 창이 뒤집혀, 샘플이
  // 한 주기 최대 감속보다 더 줄인 속도(max_v)까지 나왔다. 최대 감속 속도 하나만 남는다(#228 연우 리뷰).
  DwaConfig config = makeFastConfig();
  config.max_v = 0.5;
  const DwaPlanner planner(config);
  RobotState fast;
  fast.v = 1.5;
  fast.w = 2.5;  // max_w(1.5) + acc_w*dt(0.32)보다 빠른 각속도도 같이
  const DynamicWindow window = planner.computeDynamicWindow(fast);
  EXPECT_NEAR(window.min_v, fast.v - config.acc_v * config.dt, 1e-12);
  EXPECT_NEAR(window.max_v, window.min_v, 1e-12);
  EXPECT_GE(window.max_w, window.min_w);
  EXPECT_NEAR(window.max_w, fast.w - config.acc_w * config.dt, 1e-12);

  TestGrid test_grid(200, 80, 0.05);
  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  state.v = 1.5;
  const DwaCommand command =
    planner.computeVelocityCommand(state, Point2D{8.0, 2.0}, test_grid.grid());
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_NEAR(command.v, state.v - config.acc_v * config.dt, 1e-12);
}

TEST(DwaPlannerTest, DynamicWindowDoesNotInvertBelowNegativeLimits)
{
  // 음수 쪽도 같다: w가 -max_w보다 한 주기 변화 이상 낮으면(-2.5 < -1.5 - 0.32) 창이 {-1.5}로 튀어 한 주기에
  // 1.0 rad/s가 바뀌었다. 한계 쪽으로 최대로 줄인 값(-2.5 + acc_w*dt) 하나만 남는다. 속도도(min_v 0 아래) 같다.
  DwaConfig config = makeFastConfig();
  const DwaPlanner planner(config);
  RobotState state;
  state.v = -0.5;
  state.w = -2.5;
  const DynamicWindow window = planner.computeDynamicWindow(state);
  EXPECT_NEAR(window.min_w, state.w + config.acc_w * config.dt, 1e-12);
  EXPECT_NEAR(window.max_w, window.min_w, 1e-12);
  EXPECT_NEAR(window.min_v, state.v + config.acc_v * config.dt, 1e-12);
  EXPECT_NEAR(window.max_v, window.min_v, 1e-12);
}

TEST(DwaPlannerTest, RotateExitAngleIsClampedToMinAngle)
{
  // exit_angle > min_angle이면 모드에 들어가자마자 나와 매 주기 바뀐다 - min_angle로 줄인다
  DwaConfig config = makeStackConfig();
  config.rotate_in_place_min_angle = 0.5;
  config.rotate_in_place_exit_angle = 1.0;
  EXPECT_DOUBLE_EQ(DwaPlanner(config).config().rotate_in_place_exit_angle, 0.5);
  config.rotate_in_place_exit_angle = 0.3;
  EXPECT_DOUBLE_EQ(DwaPlanner(config).config().rotate_in_place_exit_angle, 0.3);
}

TEST(DwaPlannerTest, StationarySampleSurvivesApproachingDynamicObstacle)
{
  // 서 있는 로봇 쪽으로 사람이 걸어오면(예측 켬) 정지 궤적도 예측 궤적 중간에 닿는다. 정지(v=0, w=0)는 그래도
  // 후보로 남아야 한다(허용 속도 조건으로 tier 1) - 제자리 회전만 버린다(#244 연우 리뷰 1). 이번 주기 전진을
  // 못 하는 창(max_v 0)으로 정지·제자리 회전만 남겨 본다. 예전에는 둘 다 버려 kNoValidTrajectory였다.
  TestGrid test_grid(160, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 2.0, 2.0, 6.0, 2.0);
  DwaConfig config = makeStackConfig();
  config.max_v = 0.0;
  const DwaPlanner planner(config);
  ASSERT_TRUE(planner.config().predict_dynamic_obstacles);
  RobotState state;
  state.x = 2.0;
  state.y = 2.0;
  DynamicObstacle walker;  // 왼쪽 1.0 m 밖에서 1.0 m/s로 다가온다 - 약 0.7 s 뒤 안전 여유 안
  walker.radius = 0.25;
  walker.x = 2.0;
  walker.y = 2.0 + 0.2 + 1.0 + walker.radius;
  walker.vy = -1.0;
  const DwaCommand command =
    planner.computeVelocityCommand(state, path, test_grid.grid(), {walker});
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_EQ(command.v, 0.0);
  EXPECT_EQ(command.w, 0.0);
}

TEST(DwaPlannerTest, BlockedRotationFallsBackWithoutStationaryHeadingTrap)
{
  // #244 연우 리뷰 2·3: 폭 0.46 m 통로(옆 여유 0.03 m)라 제자리 회전이 양쪽 다 거의 막힌다. 기준점이 왼쪽
  // 뒤(약 120°)라 회전 모드다. 회전이 막혀 전체 창으로 다시 고를 때는 정지를 평소처럼 1.0으로 매겨야
  // 한다 - 지금 방향 오차로 매기면 "목표 보고 서 있기"가 전진보다 싸져 그 자리에 선다. state.w = 0.01이면
  // w 샘플에 정확한 0이 없어 거의 안 도는 샘플(w≈0.01)이 "회전"으로 남는데, 그걸 채택해도 그 자리에 선다.
  TestGrid test_grid(400, 200, 0.01);                // 4m x 2m
  test_grid.setRegion(0.0, 3.999, 0.0, 0.769, 254);  // 통로 아래 벽(y < 0.77)
  test_grid.setRegion(0.0, 3.999, 1.23, 1.999, 254); // 통로 위 벽(y >= 1.23)
  std::vector<Point2D> path;
  appendLine(path, 2.0, 1.0, 2.0 - 1.0, 1.0 + 1.7);  // 왼쪽 뒤 약 120°
  const DwaPlanner planner(makeStackConfig());
  for (const double w0 : {0.0, 0.01}) {
    RobotState state;
    state.x = 2.0;
    state.y = 1.0;
    state.w = w0;
    ASSERT_FALSE(planner.footprintCollides(test_grid.grid(), state.x, state.y, state.yaw));
    const DwaCommand command = planner.computeVelocityCommand(state, path, test_grid.grid());
    EXPECT_TRUE(planner.rotatingInPlace()) << "w0=" << w0;
    ASSERT_EQ(command.status, DwaStatus::kOk) << "w0=" << w0;
    EXPECT_GT(std::abs(command.v), 1e-3) << "w0=" << w0 << " w=" << command.w;  // 서 있지 않는다
  }
}

TEST(DwaPlannerTest, NoRotateInPlaceNearDynamicObstacle)
{
  // #244 연우 리뷰 4: 사람을 피하느라 경로에서 벗어나 기준점 오차가 45°를 넘어도, 동적 장애물이
  // safety_margin + influence_radius(1.2 m) 안이면 회전 모드에 들어가지 않는다 - 서서 경로 쪽(사람이 오는 쪽)으로
  // 돌지 않게. 멀면(1.2 m 밖) 들어간다.
  TestGrid test_grid(200, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 8.0, 2.0);
  const DwaPlanner planner(makeStackConfig());
  RobotState state;
  state.x = 2.0;
  state.y = 2.0;
  state.yaw = kTestPi / 2.0;  // 기준점(+x)이 오른쪽 90°
  DynamicObstacle person;
  person.radius = 0.25;
  person.x = 2.0 + 0.2 + 0.8 + person.radius;  // 오른쪽 옆에서 0.8 m(1.2 m 안)
  person.y = 2.0;
  planner.computeVelocityCommand(state, path, test_grid.grid(), {person});
  EXPECT_FALSE(planner.rotatingInPlace());
  person.x = 2.0 + 0.2 + 1.5 + person.radius;  // 1.5 m(1.2 m 밖)
  planner.computeVelocityCommand(state, path, test_grid.grid(), {person});
  EXPECT_TRUE(planner.rotatingInPlace());
  planner.computeVelocityCommand(state, path, test_grid.grid());
  EXPECT_TRUE(planner.rotatingInPlace());
}

TEST(DwaPlannerTest, RotateInPlaceKeepsClearanceFromStaticObstacle)
{
  // #244 연우 리뷰 5: 안전 노드는 제자리 회전 중 모서리가 0.05 m(side_margin) 안으로 가까워지면 막는다. 회전
  // 모드의 회전 후보는 풋프린트를 rotate_in_place_clearance만큼 키워 검사해, 모서리가 0.05 m 안으로 스치는
  // 회전을 고르지 않는다. 왼쪽 앞 대각선 0.37~0.40 m(모서리 회전 반경 0.36 m 밖)에 치명 셀 - 키우지 않으면
  // 왼쪽(반시계) 회전이 모서리 0.01~0.04 m로 스치며 통과한다.
  TestGrid test_grid(400, 200, 0.02);                // 8m x 4m
  test_grid.setRegion(4.265, 4.275, 2.265, 2.275, 254);  // 한 셀 [4.26, 4.28) x [2.26, 2.28)
  std::vector<Point2D> path;
  appendLine(path, 4.0, 2.0, 4.0 + 3.0 * std::cos(2.6), 2.0 + 3.0 * std::sin(2.6));  // 왼쪽 뒤 약 150°
  DwaConfig inflated = makeStackConfig();
  inflated.footprint_half_length += 0.05;
  inflated.footprint_half_width += 0.05;
  const DwaPlanner margin_check(inflated);  // 0.05 m 여유를 지키는지 보는 검사용
  auto keeps_clearance = [&](const DwaPlanner & planner, const DwaCommand & command, RobotState s) {
      for (const TrajectoryPoint & point : planner.simulateTrajectory(s, command.v, command.w)) {
        if (margin_check.footprintCollides(test_grid.grid(), point.x, point.y, point.yaw)) {
          return false;
        }
      }
      return true;
    };
  RobotState state;
  state.x = 4.0;
  state.y = 2.0;
  ASSERT_FALSE(margin_check.footprintCollides(test_grid.grid(), state.x, state.y, state.yaw));

  const DwaPlanner planner(makeStackConfig());
  ASSERT_DOUBLE_EQ(planner.config().rotate_in_place_clearance, 0.05);
  // 왼쪽 제자리 회전은 0.05 m 여유를 못 지켜 빠지고, 오른쪽은 기준점에서 멀어져 채택되지 않는다 - 전체 창으로
  // 다시 골라 전진하며 돈다(여유는 제자리 회전 샘플에만 둔다)
  const DwaCommand command = planner.computeVelocityCommand(state, path, test_grid.grid());
  ASSERT_EQ(command.status, DwaStatus::kOk);
  EXPECT_TRUE(planner.rotatingInPlace());
  EXPECT_GT(command.v, 1e-3) << "w=" << command.w;

  // 0이면(예전) 왼쪽으로 돌며 모서리가 0.05 m 안으로 스친다
  DwaConfig off = makeStackConfig();
  off.rotate_in_place_clearance = 0.0;
  const DwaPlanner no_margin(off);
  const DwaCommand tight = no_margin.computeVelocityCommand(state, path, test_grid.grid());
  ASSERT_EQ(tight.status, DwaStatus::kOk);
  EXPECT_GT(tight.w, 0.0);
  EXPECT_FALSE(keeps_clearance(no_margin, tight, state));
}

TEST(DwaPlannerTest, AvoidModeSwapsConfigAndBack)
{
  // 회피 모드(4-10절): 안전 여유 += avoid_margin_add, dynamic_weight *= avoid_weight_scale,
  // sim_time = avoid_sim_time. 끄면 생성 때 설정으로 돌아온다
  DwaConfig config = makeStackConfig();
  config.avoid_mode_enabled = true;
  DwaPlanner planner(config);
  EXPECT_FALSE(planner.avoidMode());
  planner.setAvoidMode(true);
  EXPECT_TRUE(planner.avoidMode());
  EXPECT_DOUBLE_EQ(
    planner.config().dynamic_obstacle_safety_margin, config.dynamic_obstacle_safety_margin + 0.4);
  EXPECT_DOUBLE_EQ(planner.config().dynamic_weight, config.dynamic_weight * 2.0);
  EXPECT_DOUBLE_EQ(planner.rolloutTime(), 3.0);
  EXPECT_EQ(planner.simulateTrajectory(RobotState(), 0.5, 0.0).size(), 30u);
  EXPECT_DOUBLE_EQ(planner.config().sim_time, config.sim_time);  // 기준점·비용은 평소 sim_time
  EXPECT_EQ(planner.config().v_samples, config.v_samples);
  planner.setAvoidMode(false);
  EXPECT_FALSE(planner.avoidMode());
  EXPECT_DOUBLE_EQ(
    planner.config().dynamic_obstacle_safety_margin, config.dynamic_obstacle_safety_margin);
  EXPECT_DOUBLE_EQ(planner.config().dynamic_weight, config.dynamic_weight);
  EXPECT_DOUBLE_EQ(planner.config().sim_time, config.sim_time);
  EXPECT_DOUBLE_EQ(planner.rolloutTime(), config.sim_time);
}

TEST(DwaPlannerTest, AvoidModeKeepsHeadingTargetWithoutObstacles)
{
  // 회피 모드는 궤적 시뮬레이션 길이만 늘린다(avoid_rollout_time). heading 기준점(|v|*sim_time 앞)과 비용을
  // 매기는 궤적 끝은 평소 sim_time 그대로라, 장애물이 없으면 모드가 바뀌어도 명령이 같다 - 기준점이 튀지 않는다.
  // 90° 꺾이는 경로 앞에서 순항(기준점이 꺾인 곳 너머)·정지 출발·목표가 뒤(회전 모드) 세 경우로 본다.
  TestGrid test_grid(200, 200, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 3.0, 2.0);
  appendLine(path, 3.0, 2.0, 3.0, 8.0);
  DwaConfig config = makeStackConfig();
  config.avoid_mode_enabled = true;
  std::vector<RobotState> states(3);
  states[0].x = 1.8;
  states[0].y = 2.0;
  states[0].v = 0.5;
  states[1].x = 1.0;
  states[1].y = 2.0;
  states[2].x = 1.5;
  states[2].y = 2.0;
  states[2].yaw = kTestPi;
  for (std::size_t i = 0; i < states.size(); ++i) {
    DwaPlanner normal(config);
    DwaPlanner avoiding(config);
    avoiding.setAvoidMode(true);
    const DwaCommand a = normal.computeVelocityCommand(states[i], path, test_grid.grid());
    const DwaCommand b = avoiding.computeVelocityCommand(states[i], path, test_grid.grid());
    EXPECT_EQ(a.status, b.status) << "case " << i;
    EXPECT_EQ(a.v, b.v) << "case " << i;
    EXPECT_EQ(a.w, b.w) << "case " << i;
    EXPECT_EQ(normal.rotatingInPlace(), avoiding.rotatingInPlace()) << "case " << i;
  }
}

TEST(DwaPlannerTest, AvoidModeWidensDynamicMargin)
{
  // 왼쪽 앞에 서 있는 사람(동적 트랙, 속도 0): 직진 궤적 차체에서 0.35 m. 평소(여유 0.2 m)에는 그 옆을 지나는
  // 궤적을 고르지만, 회피 모드(여유 0.6 m)에서는 그 궤적이 "충돌"이라 0.6 m 밖으로 비켜 가는 궤적만 남는다
  TestGrid test_grid(200, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 8.0, 2.0);
  DwaConfig config = makeStackConfig();
  config.avoid_mode_enabled = true;
  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  state.v = 0.5;
  DynamicObstacle person;
  person.radius = 0.25;
  person.x = 2.2;
  person.y = 2.0 + 0.2 + 0.35 + person.radius;

  DwaPlanner normal(config);
  const DwaCommand near_pass = normal.computeVelocityCommand(state, path, test_grid.grid(), {person});
  DwaPlanner avoiding(config);
  avoiding.setAvoidMode(true);
  const DwaCommand wide_pass =
    avoiding.computeVelocityCommand(state, path, test_grid.grid(), {person});
  ASSERT_EQ(near_pass.status, DwaStatus::kOk);
  ASSERT_EQ(wide_pass.status, DwaStatus::kOk);

  // 회피 모드 여유(0.6 m)로 검사: 평소 궤적은 닿고, 회피 모드 궤적은 안 닿는다
  auto hits_wide_margin = [&](const DwaCommand & command) {
      double t = 0.0;
      for (const TrajectoryPoint & point : avoiding.simulateTrajectory(state, command.v, command.w)) {
        t += avoiding.config().dt;
        if (avoiding.dynamicObstacleCollides({person}, t, point.x, point.y, point.yaw)) {
          return true;
        }
      }
      return false;
    };
  EXPECT_TRUE(hits_wide_margin(near_pass)) << "v=" << near_pass.v << " w=" << near_pass.w;
  EXPECT_FALSE(hits_wide_margin(wide_pass)) << "v=" << wide_pass.v << " w=" << wide_pass.w;
}

TEST(DwaPlannerTest, AvoidModeDisabledMatchesPreviousBehavior)
{
  // avoid_mode_enabled = false면 setAvoidMode(true)를 무시한다 - 설정도 명령도 회피 모드 없는 플래너와 같다
  TestGrid test_grid(200, 80, 0.05);
  std::vector<Point2D> path;
  appendLine(path, 1.0, 2.0, 8.0, 2.0);
  const DwaConfig config = makeStackConfig();
  ASSERT_FALSE(config.avoid_mode_enabled);
  DwaPlanner disabled(config);
  disabled.setAvoidMode(true);
  EXPECT_FALSE(disabled.avoidMode());
  EXPECT_DOUBLE_EQ(disabled.config().sim_time, config.sim_time);
  EXPECT_DOUBLE_EQ(
    disabled.config().dynamic_obstacle_safety_margin, config.dynamic_obstacle_safety_margin);
  EXPECT_DOUBLE_EQ(disabled.config().dynamic_weight, config.dynamic_weight);

  const DwaPlanner plain(config);
  RobotState state;
  state.x = 1.0;
  state.y = 2.0;
  state.v = 0.5;
  DynamicObstacle person;
  person.radius = 0.25;
  person.x = 2.2;
  person.y = 2.0 + 0.2 + 0.35 + person.radius;
  person.vy = -0.3;
  for (int step = 0; step < 30; ++step) {
    const DwaCommand a = disabled.computeVelocityCommand(state, path, test_grid.grid(), {person});
    const DwaCommand b = plain.computeVelocityCommand(state, path, test_grid.grid(), {person});
    ASSERT_EQ(a.status, b.status) << "step=" << step;
    ASSERT_EQ(a.v, b.v) << "step=" << step;
    ASSERT_EQ(a.w, b.w) << "step=" << step;
    if (a.status != DwaStatus::kOk) {
      break;
    }
    state.x += a.v * std::cos(state.yaw) * config.dt;
    state.y += a.v * std::sin(state.yaw) * config.dt;
    state.yaw = wrapAngle(state.yaw + a.w * config.dt);
    state.v = a.v;
    state.w = a.w;
    person.y += person.vy * config.dt;
  }
}

}  // namespace amr_planning_core
