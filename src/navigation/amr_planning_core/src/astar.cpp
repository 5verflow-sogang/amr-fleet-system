#include "amr_planning_core/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <vector>

namespace amr_planning_core
{

namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

struct Move
{
  int dx;
  int dy;
  double distance;
};

const std::vector<Move> kMoves = {
  {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
  {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2},
};

struct OpenNode
{
  double f_score;
  double g_score;
  Cell cell;
};

// f가 같으면 g가 큰(목표에 더 가까이 간) 노드를 먼저 꺼낸다. 8방향 격자의 빈 공간에는 f가 같은
// 노드가 아주 많아서, 동점을 아무렇게나 풀면 최적 경로 둘레의 셀을 거의 다 펼친다. 이 순서는
// 어느 최적 경로를 고르는지만 바꾸고 경로 비용(최적성)은 그대로다.
struct OpenNodeCompare
{
  bool operator()(const OpenNode & a, const OpenNode & b) const
  {
    if (a.f_score != b.f_score) {
      return a.f_score > b.f_score;
    }
    return a.g_score < b.g_score;
  }
};

std::size_t encode(const Cell & cell, std::size_t width)
{
  return static_cast<std::size_t>(cell.y) * width + static_cast<std::size_t>(cell.x);
}

}  // namespace

AStarPlanner::AStarPlanner(AStarConfig config)
: config_(config)
{
}

double AStarPlanner::heuristic(const Cell & a, const Cell & b) const
{
  const double dx = static_cast<double>(std::abs(a.x - b.x));
  const double dy = static_cast<double>(std::abs(a.y - b.y));

  if (config_.heuristic == HeuristicType::kEuclidean) {
    return std::sqrt(dx * dx + dy * dy);
  }

  // Octile: 대각선으로 갈 수 있는 만큼 대각선으로, 나머지는 직선으로 간다는 가정의 추정치.
  const double diag = std::min(dx, dy);
  const double straight = std::max(dx, dy) - diag;
  return diag * kSqrt2 + straight;
}

double AStarPlanner::moveCost(double base_distance, uint8_t neighbor_cost) const
{
  const double cost_factor = config_.cost_weight * (static_cast<double>(neighbor_cost) / 255.0);
  return base_distance * (1.0 + cost_factor);
}

bool AStarPlanner::isTraversable(const Grid & grid, const Cell & cell) const
{
  return grid.inBounds(cell) && grid.cost(cell) < config_.lethal_cost_threshold;
}

PlanResult AStarPlanner::plan(const Grid & grid, const Cell & start, const Cell & goal) const
{
  PlanResult result;

  if (!grid.inBounds(start)) {
    result.status = PlanStatus::kStartOutOfBounds;
    return result;
  }
  if (!grid.inBounds(goal)) {
    result.status = PlanStatus::kGoalOutOfBounds;
    return result;
  }
  if (grid.cost(start) >= config_.lethal_cost_threshold) {
    result.status = PlanStatus::kStartOccupied;
    return result;
  }
  if (grid.cost(goal) >= config_.lethal_cost_threshold) {
    result.status = PlanStatus::kGoalOccupied;
    return result;
  }

  if (start == goal) {
    result.status = PlanStatus::kSuccess;
    result.path = {start};
    return result;
  }

  // 셀마다 g·부모·닫힘을 격자 크기 배열(작업 버퍼, astar.hpp)로 둔다. 해시 맵은 창고 지도(0.025 m,
  // 2408 x 1728 셀)에서 펼친 셀이 수십만 개가 되면 해싱·재할당이 탐색 시간의 대부분을 차지했다.
  const std::size_t width = grid.width();
  const std::size_t cell_count = width * grid.height();
  if (g_.size() != cell_count || generation_ == std::numeric_limits<uint32_t>::max()) {
    g_.assign(cell_count, 0.0);
    parent_.assign(cell_count, 0);
    seen_.assign(cell_count, 0);
    closed_.assign(cell_count, 0);
    generation_ = 0;
  }
  const uint32_t generation = ++generation_;
  std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeCompare> open;

  const std::size_t start_key = encode(start, width);
  g_[start_key] = 0.0;
  parent_[start_key] = static_cast<uint32_t>(start_key);
  seen_[start_key] = generation;
  open.push({heuristic(start, goal), 0.0, start});

  while (!open.empty()) {
    const Cell current = open.top().cell;
    open.pop();
    const std::size_t current_key = encode(current, width);

    if (closed_[current_key] == generation) {
      continue;
    }
    closed_[current_key] = generation;

    if (current == goal) {
      std::vector<Cell> path;
      std::size_t key = current_key;
      while (true) {
        path.push_back(Cell{static_cast<int>(key % width), static_cast<int>(key / width)});
        if (key == start_key) {
          break;
        }
        key = parent_[key];
      }
      std::reverse(path.begin(), path.end());
      result.status = PlanStatus::kSuccess;
      result.path = std::move(path);
      return result;
    }

    const double current_g = g_[current_key];

    for (const Move & move : kMoves) {
      const Cell neighbor{current.x + move.dx, current.y + move.dy};

      if (!isTraversable(grid, neighbor)) {
        continue;
      }

      if (config_.prevent_corner_cutting && move.dx != 0 && move.dy != 0) {
        const Cell side_a{current.x + move.dx, current.y};
        const Cell side_b{current.x, current.y + move.dy};
        if (!isTraversable(grid, side_a) || !isTraversable(grid, side_b)) {
          continue;
        }
      }

      const std::size_t neighbor_key = encode(neighbor, width);
      if (closed_[neighbor_key] == generation) {
        continue;
      }

      const double tentative_g = current_g + moveCost(move.distance, grid.cost(neighbor));

      if (seen_[neighbor_key] != generation || tentative_g < g_[neighbor_key]) {
        seen_[neighbor_key] = generation;
        g_[neighbor_key] = tentative_g;
        parent_[neighbor_key] = static_cast<uint32_t>(current_key);
        open.push({tentative_g + heuristic(neighbor, goal), tentative_g, neighbor});
      }
    }
  }

  result.status = PlanStatus::kNoPathFound;
  return result;
}

}  // namespace amr_planning_core
