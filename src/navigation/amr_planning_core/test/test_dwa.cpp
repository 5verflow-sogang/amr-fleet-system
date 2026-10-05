#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
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

}  // namespace amr_planning_core
