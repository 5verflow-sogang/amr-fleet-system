#include "amr_planning_core/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <unordered_map>
#include <unordered_set>
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
  Cell cell;
};

struct OpenNodeCompare
{
  bool operator()(const OpenNode & a, const OpenNode & b) const
  {
    return a.f_score > b.f_score;
  }
};

int64_t encode(const Cell & cell, std::size_t width)
{
  return static_cast<int64_t>(cell.y) * static_cast<int64_t>(width) +
         static_cast<int64_t>(cell.x);
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

  std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeCompare> open;
  std::unordered_map<int64_t, double> g_score;
  std::unordered_map<int64_t, Cell> came_from;
  std::unordered_set<int64_t> closed;

  const int64_t start_key = encode(start, grid.width());
  g_score[start_key] = 0.0;
  open.push({heuristic(start, goal), start});

  while (!open.empty()) {
    const Cell current = open.top().cell;
    open.pop();
    const int64_t current_key = encode(current, grid.width());

    if (closed.count(current_key) > 0) {
      continue;
    }
    closed.insert(current_key);

    if (current == goal) {
      std::vector<Cell> path;
      int64_t key = current_key;
      path.push_back(current);
      while (key != start_key) {
        const Cell prev = came_from.at(key);
        path.push_back(prev);
        key = encode(prev, grid.width());
      }
      std::reverse(path.begin(), path.end());
      result.status = PlanStatus::kSuccess;
      result.path = std::move(path);
      return result;
    }

    const double current_g = g_score.at(current_key);

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

      const int64_t neighbor_key = encode(neighbor, grid.width());
      if (closed.count(neighbor_key) > 0) {
        continue;
      }

      const double tentative_g = current_g + moveCost(move.distance, grid.cost(neighbor));

      const auto it = g_score.find(neighbor_key);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[neighbor_key] = tentative_g;
        came_from[neighbor_key] = current;
        open.push({tentative_g + heuristic(neighbor, goal), neighbor});
      }
    }
  }

  result.status = PlanStatus::kNoPathFound;
  return result;
}

}  // namespace amr_planning_core
