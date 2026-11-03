#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "amr_planning_core/astar.hpp"
#include "amr_planning_core/grid.hpp"

namespace amr_planning_core
{
namespace
{

// 테스트용 격자. costs_ 버퍼를 소유하고, Grid는 그 버퍼를 가리키는 뷰만 갖는다.
class TestGrid
{
public:
  TestGrid(std::size_t width, std::size_t height, uint8_t default_cost = 0)
  : width_(width), costs_(width * height, default_cost)
  {
  }

  void setCost(int x, int y, uint8_t cost)
  {
    costs_[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)] = cost;
  }

  Grid grid() const
  {
    return Grid(width_, costs_.size() / width_, 0.05, costs_.data());
  }

private:
  std::size_t width_;
  std::vector<uint8_t> costs_;
};

bool containsCell(const std::vector<Cell> & path, const Cell & target)
{
  for (const auto & cell : path) {
    if (cell == target) {
      return true;
    }
  }
  return false;
}

double pathLength(const std::vector<Cell> & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = static_cast<double>(path[i].x - path[i - 1].x);
    const double dy = static_cast<double>(path[i].y - path[i - 1].y);
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

}  // namespace

TEST(AStarPlannerTest, StraightLinePath)
{
  TestGrid test_grid(6, 1);
  AStarPlanner planner;

  const PlanResult result = planner.plan(test_grid.grid(), Cell{0, 0}, Cell{5, 0});

  ASSERT_EQ(result.status, PlanStatus::kSuccess);
  ASSERT_EQ(result.path.size(), 6u);
  EXPECT_EQ(result.path.front(), (Cell{0, 0}));
  EXPECT_EQ(result.path.back(), (Cell{5, 0}));
  for (const auto & cell : result.path) {
    EXPECT_EQ(cell.y, 0);
  }
}

TEST(AStarPlannerTest, ObstacleAvoidance)
{
  TestGrid test_grid(5, 5);
  // x=2 열을 y=0..3까지 막고 y=4만 뚫어둔다.
  for (int y = 0; y <= 3; ++y) {
    test_grid.setCost(2, y, 254);
  }
  AStarPlanner planner;

  const PlanResult result = planner.plan(test_grid.grid(), Cell{0, 2}, Cell{4, 2});

  ASSERT_EQ(result.status, PlanStatus::kSuccess);
  EXPECT_FALSE(containsCell(result.path, (Cell{2, 0})));
  EXPECT_FALSE(containsCell(result.path, (Cell{2, 1})));
  EXPECT_FALSE(containsCell(result.path, (Cell{2, 2})));
  EXPECT_FALSE(containsCell(result.path, (Cell{2, 3})));
  EXPECT_TRUE(containsCell(result.path, (Cell{2, 4})));
}

TEST(AStarPlannerTest, UnreachableGoalReturnsNoPathFound)
{
  TestGrid test_grid(5, 5);
  const Cell goal{2, 2};
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      test_grid.setCost(goal.x + dx, goal.y + dy, 254);
    }
  }
  AStarPlanner planner;

  const PlanResult result = planner.plan(test_grid.grid(), Cell{0, 0}, goal);

  EXPECT_EQ(result.status, PlanStatus::kNoPathFound);
  EXPECT_TRUE(result.path.empty());
}

TEST(AStarPlannerTest, StartEqualsGoalReturnsSingleCellPath)
{
  TestGrid test_grid(5, 5);
  AStarPlanner planner;

  const PlanResult result = planner.plan(test_grid.grid(), Cell{3, 3}, Cell{3, 3});

  ASSERT_EQ(result.status, PlanStatus::kSuccess);
  ASSERT_EQ(result.path.size(), 1u);
  EXPECT_EQ(result.path.front(), (Cell{3, 3}));
}

TEST(AStarPlannerTest, OutOfBoundsCellsAreRejected)
{
  TestGrid test_grid(5, 5);
  AStarPlanner planner;

  const PlanResult start_result = planner.plan(test_grid.grid(), Cell{-1, 0}, Cell{4, 4});
  EXPECT_EQ(start_result.status, PlanStatus::kStartOutOfBounds);

  const PlanResult goal_result = planner.plan(test_grid.grid(), Cell{0, 0}, Cell{10, 10});
  EXPECT_EQ(goal_result.status, PlanStatus::kGoalOutOfBounds);
}

TEST(AStarPlannerTest, OccupiedStartOrGoalAreRejected)
{
  TestGrid start_blocked_grid(5, 5);
  start_blocked_grid.setCost(0, 0, 254);
  AStarPlanner planner;

  const PlanResult start_result = planner.plan(start_blocked_grid.grid(), Cell{0, 0}, Cell{4, 4});
  EXPECT_EQ(start_result.status, PlanStatus::kStartOccupied);

  TestGrid goal_blocked_grid(5, 5);
  goal_blocked_grid.setCost(4, 4, 254);
  const PlanResult goal_result = planner.plan(goal_blocked_grid.grid(), Cell{0, 0}, Cell{4, 4});
  EXPECT_EQ(goal_result.status, PlanStatus::kGoalOccupied);
}

TEST(AStarPlannerTest, OctileAndEuclideanFindEqualCostOptimalPath)
{
  TestGrid test_grid(6, 6);

  AStarConfig octile_config;
  octile_config.heuristic = HeuristicType::kOctile;
  AStarPlanner octile_planner(octile_config);

  AStarConfig euclidean_config;
  euclidean_config.heuristic = HeuristicType::kEuclidean;
  AStarPlanner euclidean_planner(euclidean_config);

  const PlanResult octile_result = octile_planner.plan(test_grid.grid(), Cell{0, 0}, Cell{5, 5});
  const PlanResult euclidean_result =
    euclidean_planner.plan(test_grid.grid(), Cell{0, 0}, Cell{5, 5});

  ASSERT_EQ(octile_result.status, PlanStatus::kSuccess);
  ASSERT_EQ(euclidean_result.status, PlanStatus::kSuccess);

  const double octile_cost = pathLength(octile_result.path);
  const double euclidean_cost = pathLength(euclidean_result.path);

  // 두 휴리스틱 모두 admissible하므로 최적 경로 비용은 같아야 한다(대각선 5칸, 5*sqrt2).
  EXPECT_NEAR(octile_cost, euclidean_cost, 1e-6);
  EXPECT_NEAR(octile_cost, 5.0 * std::sqrt(2.0), 1e-6);
}

TEST(AStarPlannerTest, CostWeightRoutesAroundExpensiveCell)
{
  TestGrid test_grid(5, 3);
  test_grid.setCost(2, 1, 200);  // 직선 경로 위의 비싼 셀. 통과는 가능(치명 비용 미달).

  AStarConfig zero_weight_config;
  zero_weight_config.cost_weight = 0.0;
  AStarPlanner zero_weight_planner(zero_weight_config);

  AStarConfig high_weight_config;
  high_weight_config.cost_weight = 5.0;
  AStarPlanner high_weight_planner(high_weight_config);

  const PlanResult direct_result =
    zero_weight_planner.plan(test_grid.grid(), Cell{0, 1}, Cell{4, 1});
  const PlanResult detour_result =
    high_weight_planner.plan(test_grid.grid(), Cell{0, 1}, Cell{4, 1});

  ASSERT_EQ(direct_result.status, PlanStatus::kSuccess);
  ASSERT_EQ(detour_result.status, PlanStatus::kSuccess);

  EXPECT_TRUE(containsCell(direct_result.path, (Cell{2, 1})));
  EXPECT_FALSE(containsCell(detour_result.path, (Cell{2, 1})));
}

TEST(AStarPlannerTest, ReusedPlannerGivesSameResultsAcrossCallsAndGridSizes)
{
  // 작업 버퍼를 호출 사이에 재사용한다(astar.hpp). 앞 탐색의 g·닫힘이 남아 다음 탐색을 망치거나,
  // 격자 크기가 바뀔 때 버퍼를 다시 안 잡으면 경로가 달라진다
  TestGrid walled(20, 10);
  for (int y = 0; y < 8; ++y) {
    walled.setCost(10, y, 254);
  }
  TestGrid open_small(6, 4);

  const AStarPlanner reused;
  const PlanResult first = reused.plan(walled.grid(), Cell{1, 1}, Cell{18, 1});
  const PlanResult small = reused.plan(open_small.grid(), Cell{0, 0}, Cell{5, 3});
  const PlanResult again = reused.plan(walled.grid(), Cell{1, 1}, Cell{18, 1});
  const PlanResult blocked = reused.plan(walled.grid(), Cell{1, 1}, Cell{10, 2});

  ASSERT_EQ(first.status, PlanStatus::kSuccess);
  ASSERT_EQ(small.status, PlanStatus::kSuccess);
  ASSERT_EQ(again.status, PlanStatus::kSuccess);
  EXPECT_EQ(blocked.status, PlanStatus::kGoalOccupied);
  EXPECT_NEAR(pathLength(small.path), 3.0 * std::sqrt(2.0) + 2.0, 1e-9);
  ASSERT_EQ(first.path.size(), again.path.size());
  for (std::size_t i = 0; i < first.path.size(); ++i) {
    EXPECT_EQ(first.path[i], again.path[i]);
  }
  const PlanResult fresh = AStarPlanner().plan(walled.grid(), Cell{1, 1}, Cell{18, 1});
  EXPECT_NEAR(pathLength(first.path), pathLength(fresh.path), 1e-9);
}

TEST(AStarPlannerTest, PathIsConnectedAndOptimalOnWideGridWithWall)
{
  // 셀 인덱스(y * width + x)로 부모를 기록하므로 폭과 높이가 다른 격자에서 경로를 되짚는지 확인한다.
  // 벽 x=60 (y 0..37)을 위쪽 틈으로 돌아간다
  TestGrid test_grid(120, 40);
  for (int y = 0; y < 38; ++y) {
    test_grid.setCost(60, y, 254);
  }
  const PlanResult result = AStarPlanner().plan(test_grid.grid(), Cell{5, 5}, Cell{115, 5});
  ASSERT_EQ(result.status, PlanStatus::kSuccess);
  EXPECT_EQ(result.path.front(), (Cell{5, 5}));
  EXPECT_EQ(result.path.back(), (Cell{115, 5}));
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_LE(std::abs(result.path[i].x - result.path[i - 1].x), 1);
    EXPECT_LE(std::abs(result.path[i].y - result.path[i - 1].y), 1);
    EXPECT_FALSE(result.path[i] == result.path[i - 1]);
  }
  // 틈 (60, 38)을 지나는 최단 경로: (5,5)→(59,38)→(61,38)→(115,5)의 옥타일 길이
  const double leg = 33.0 * std::sqrt(2.0) + 21.0;
  EXPECT_NEAR(pathLength(result.path), 2.0 * leg + 2.0, 1e-6);
}

}  // namespace amr_planning_core
