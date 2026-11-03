#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "amr_planning_core/astar.hpp"
#include "amr_planning_core/grid.hpp"
#include "amr_planning_core/path_smoothing.hpp"

namespace amr_planning_core
{
namespace
{

class TestGrid
{
public:
  TestGrid(std::size_t width, std::size_t height)
  : width_(width), costs_(width * height, 0)
  {
  }

  void setCost(int x, int y, uint8_t cost)
  {
    costs_[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)] = cost;
  }

  // 사각형 장애물(254)과 Nav2 inflation 같은 비용(내접 반경 안 253, 밖은 거리에 따라 지수 감소)
  void addInflatedBox(int x0, int y0, int x1, int y1, double inscribed_cells, double radius_cells)
  {
    const int height = static_cast<int>(costs_.size() / width_);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < static_cast<int>(width_); ++x) {
        const double dx = std::max({x0 - x, 0, x - x1});
        const double dy = std::max({y0 - y, 0, y - y1});
        const double d = std::hypot(dx, dy);
        uint8_t cost = 0;
        if (d == 0.0) {
          cost = 254;
        } else if (d <= inscribed_cells) {
          cost = 253;
        } else if (d <= radius_cells) {
          cost = static_cast<uint8_t>(252.0 * std::exp(-0.3 * (d - inscribed_cells)));
        }
        const std::size_t i = static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
        costs_[i] = std::max(costs_[i], cost);
      }
    }
  }

  Grid grid() const
  {
    return Grid(width_, costs_.size() / width_, 0.05, costs_.data());
  }

private:
  std::size_t width_;
  std::vector<uint8_t> costs_;
};

double polylineLength(const std::vector<Point2D> & points)
{
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  return total;
}

double cellPathLength(const std::vector<Cell> & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return total;
}

uint8_t maxCostOnCells(const Grid & grid, const std::vector<Cell> & path)
{
  uint8_t max_cost = 0;
  for (const Cell & cell : path) {
    max_cost = std::max(max_cost, grid.cost(cell));
  }
  return max_cost;
}

uint8_t maxCostOnPolyline(const Grid & grid, const std::vector<Point2D> & points)
{
  uint8_t max_cost = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    max_cost = std::max(max_cost, maxCostAlongSegment(grid, points[i - 1], points[i]));
  }
  return max_cost;
}

double maxSpacing(const std::vector<Point2D> & points)
{
  double spacing = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    spacing = std::max(spacing, std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y));
  }
  return spacing;
}

std::vector<Cell> lPath(int corner)
{
  std::vector<Cell> path;
  for (int x = 0; x <= corner; ++x) {
    path.push_back({x, 0});
  }
  for (int y = 1; y <= corner; ++y) {
    path.push_back({corner, y});
  }
  return path;
}

}  // namespace

TEST(PathSmoothingTest, PathsOfTwoOrFewerCellsAreReturnedAsIs)
{
  TestGrid tg(5, 5);
  const Grid grid = tg.grid();
  EXPECT_TRUE(smoothPath(grid, {}).empty());

  const std::vector<Point2D> one = smoothPath(grid, {{2, 3}});
  ASSERT_EQ(one.size(), 1u);
  EXPECT_DOUBLE_EQ(one[0].x, 2.0);
  EXPECT_DOUBLE_EQ(one[0].y, 3.0);

  const std::vector<Point2D> two = smoothPath(grid, {{1, 1}, {2, 2}});
  ASSERT_EQ(two.size(), 2u);
  EXPECT_DOUBLE_EQ(two[1].x, 2.0);
}

TEST(PathSmoothingTest, StraightPathStaysOnTheLine)
{
  TestGrid tg(30, 5);
  std::vector<Cell> path;
  for (int x = 0; x < 30; ++x) {
    path.push_back({x, 2});
  }
  const std::vector<Point2D> out = smoothPath(tg.grid(), path);
  ASSERT_GE(out.size(), 2u);
  for (const Point2D & p : out) {
    EXPECT_NEAR(p.y, 2.0, 1e-9);
  }
  EXPECT_NEAR(polylineLength(out), 29.0, 1e-9);
}

TEST(PathSmoothingTest, StaircaseBecomesNearlyStraightAndShorter)
{
  // 8방향 A*는 (0,0)→(40,10)을 대각선 10칸 + 직선 30칸으로 간다(옥타일 길이 44.14). 평활화하면
  // 직선(41.23)에 가까워진다
  TestGrid tg(45, 15);
  const Grid grid = tg.grid();
  const PlanResult raw = AStarPlanner().plan(grid, {0, 0}, {40, 10});
  ASSERT_EQ(raw.status, PlanStatus::kSuccess);
  const std::vector<Point2D> out = smoothPath(grid, raw.path);

  EXPECT_NEAR(cellPathLength(raw.path), 30.0 + 10.0 * std::sqrt(2.0), 1e-9);
  EXPECT_LT(polylineLength(out), cellPathLength(raw.path));
  EXPECT_NEAR(polylineLength(out), std::hypot(40.0, 10.0), 0.05);
}

TEST(PathSmoothingTest, EndpointsArePreservedAndSpacingIsAtMostOneCell)
{
  TestGrid tg(45, 15);
  const Grid grid = tg.grid();
  const PlanResult raw = AStarPlanner().plan(grid, {1, 2}, {40, 12});
  ASSERT_EQ(raw.status, PlanStatus::kSuccess);
  const std::vector<Point2D> out = smoothPath(grid, raw.path);

  ASSERT_GE(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out.front().x, 1.0);
  EXPECT_DOUBLE_EQ(out.front().y, 2.0);
  EXPECT_DOUBLE_EQ(out.back().x, 40.0);
  EXPECT_DOUBLE_EQ(out.back().y, 12.0);
  EXPECT_LE(maxSpacing(out), 1.0 + 1e-9);
}

TEST(PathSmoothingTest, GradientRoundsTheCornerAndShortcutAloneKeepsIt)
{
  // 빈 격자의 L자 경로: 지름길만 쓰면 대각선 한 번으로 질러가므로 끈다. 경사 하강은 꼭짓점 (20,0)을
  // 안쪽으로 당겨 둥글게 하고, 끄면 꼭짓점을 그대로 지난다
  TestGrid tg(25, 25);
  const Grid grid = tg.grid();
  SmoothingConfig config;
  config.shortcut = false;

  const auto distance_to_corner = [](const std::vector<Point2D> & points) {
      double best = 1e9;
      for (const Point2D & p : points) {
        best = std::min(best, std::hypot(p.x - 20.0, p.y));
      }
      return best;
    };

  config.gradient = false;
  EXPECT_NEAR(distance_to_corner(smoothPath(grid, lPath(20), config)), 0.0, 1e-9);

  config.gradient = true;
  const std::vector<Point2D> rounded = smoothPath(grid, lPath(20), config);
  EXPECT_GT(distance_to_corner(rounded), 1.0);
  EXPECT_LT(polylineLength(rounded), 40.0);
  EXPECT_LE(maxSpacing(rounded), 1.0 + 1e-9);
}

TEST(PathSmoothingTest, NeverMovesThePathIntoCostlierCells)
{
  // 비용 가중치 0인 A*는 장애물의 inflation 가장자리를 바짝 돈다. 평활화(지름길·경사 하강)가
  // 그 모서리를 더 깎아 원래 경로보다 비싼(장애물에 더 가까운) 셀로 들어가면 안 된다
  TestGrid tg(60, 40);
  tg.addInflatedBox(25, 10, 35, 30, 4.0, 11.0);
  const Grid grid = tg.grid();
  const PlanResult raw = AStarPlanner().plan(grid, {5, 20}, {55, 20});
  ASSERT_EQ(raw.status, PlanStatus::kSuccess);
  const uint8_t raw_max = maxCostOnCells(grid, raw.path);
  ASSERT_GT(raw_max, 0);  // 경로가 inflation 영역을 지나야 이 시험이 의미가 있다

  const std::vector<Point2D> out = smoothPath(grid, raw.path);
  EXPECT_LE(maxCostOnPolyline(grid, out), raw_max);
  EXPECT_LT(polylineLength(out), cellPathLength(raw.path));
}

TEST(PathSmoothingTest, ShortcutDoesNotCrossAWallThroughAGap)
{
  // 벽(x=20)에 y=18..22 틈. 지름길이 벽을 가로질러 출발→목표로 바로 이으면 안 된다
  TestGrid tg(40, 40);
  for (int y = 0; y < 40; ++y) {
    if (y < 18 || y > 22) {
      tg.setCost(20, y, 254);
    }
  }
  const Grid grid = tg.grid();
  const PlanResult raw = AStarPlanner().plan(grid, {5, 5}, {35, 35});
  ASSERT_EQ(raw.status, PlanStatus::kSuccess);

  const std::vector<Point2D> out = smoothPath(grid, raw.path);
  EXPECT_LT(maxCostOnPolyline(grid, out), 253);
  bool through_gap = false;
  for (const Point2D & p : out) {
    through_gap = through_gap || (std::abs(p.x - 20.0) < 0.5 && p.y >= 17.5 && p.y <= 22.5);
  }
  EXPECT_TRUE(through_gap);
}

TEST(PathSmoothingTest, MaxCostAlongSegmentSeesOutOfBoundsAsBlocked)
{
  TestGrid tg(5, 5);
  tg.setCost(2, 2, 100);
  const Grid grid = tg.grid();
  EXPECT_EQ(maxCostAlongSegment(grid, {0.0, 2.0}, {4.0, 2.0}), 100);
  EXPECT_EQ(maxCostAlongSegment(grid, {0.0, 0.0}, {4.0, 0.0}), 0);
  EXPECT_EQ(maxCostAlongSegment(grid, {0.0, 0.0}, {6.0, 0.0}), 255);
}

}  // namespace amr_planning_core
