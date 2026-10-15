#include <gtest/gtest.h>

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

}  // namespace amr_planning_core
