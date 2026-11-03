#include "amr_planning_core/path_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace amr_planning_core
{

namespace
{

// 선분 위를 이 간격(셀)으로 샘플링해 지나는 셀을 찾는다
constexpr double kSegmentSampleStep = 0.1;
constexpr uint8_t kOutOfBoundsCost = 255;

Cell cellAt(const Point2D & p)
{
  return Cell{static_cast<int>(std::floor(p.x + 0.5)), static_cast<int>(std::floor(p.y + 0.5))};
}

uint8_t costAt(const Grid & grid, const Point2D & p)
{
  const Cell cell = cellAt(p);
  return grid.inBounds(cell) ? grid.cost(cell) : kOutOfBoundsCost;
}

Point2D toPoint(const Cell & cell)
{
  return Point2D{static_cast<double>(cell.x), static_cast<double>(cell.y)};
}

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

// a→b를 spacing 이하 간격으로 나눈 점들 (a는 빼고 b는 넣는다)
void appendSubdivided(const Point2D & a, const Point2D & b, double spacing, std::vector<Point2D> * out)
{
  const int pieces = std::max(1, static_cast<int>(std::ceil(distance(a, b) / spacing - 1e-9)));
  for (int k = 1; k <= pieces; ++k) {
    const double t = static_cast<double>(k) / pieces;
    out->push_back(Point2D{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
  }
}

std::vector<Point2D> shortcut(const Grid & grid, const std::vector<Cell> & path, uint8_t lethal)
{
  std::vector<Point2D> result{toPoint(path.front())};
  std::size_t i = 0;
  while (i + 1 < path.size()) {
    // 바로 다음 셀은 A*가 이미 지나갈 수 있다고 본 이동이라 늘 받는다
    std::size_t best = i + 1;
    uint8_t span_max = std::max(grid.cost(path[i]), grid.cost(path[i + 1]));
    for (std::size_t j = i + 2; j < path.size(); ++j) {
      span_max = std::max(span_max, grid.cost(path[j]));
      const uint8_t along = maxCostAlongSegment(grid, toPoint(path[i]), toPoint(path[j]));
      if (along >= lethal || along > span_max) {
        // 보이는 범위는 이어지지 않을 수 있지만(가렸다 다시 보임), 처음 막히는 곳에서 멈춰 O(n * 선분 길이)로 둔다
        break;
      }
      best = j;
    }
    result.push_back(toPoint(path[best]));
    i = best;
  }
  return result;
}

void gradientSmooth(const Grid & grid, const SmoothingConfig & config, std::vector<Point2D> * points)
{
  const std::vector<Point2D> data = *points;
  std::vector<Point2D> & y = *points;
  // 점마다 옮겨도 되는 셀 비용의 상한: 지름길 경로에서 그 점이 있던 셀의 비용
  std::vector<uint8_t> limit(data.size());
  for (std::size_t k = 0; k < data.size(); ++k) {
    limit[k] = costAt(grid, data[k]);
  }
  const auto allowed = [&](std::size_t k, const Point2D & candidate) {
      if (costAt(grid, candidate) > limit[k]) {
        return false;
      }
      // 이웃 점과 잇는 선분도 두 점 상한 중 큰 쪽 이하여야 한다 - 점 간격이 1셀보다 넓어서
      // 점은 괜찮아도 사이에서 비싼 셀(모서리)을 지날 수 있다
      return maxCostAlongSegment(grid, y[k - 1], candidate) <= std::max(limit[k - 1], limit[k]) &&
             maxCostAlongSegment(grid, candidate, y[k + 1]) <= std::max(limit[k], limit[k + 1]);
    };

  for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
    double change = 0.0;
    for (std::size_t k = 1; k + 1 < y.size(); ++k) {
      const Point2D candidate{
        y[k].x + config.weight_data * (data[k].x - y[k].x) +
        config.weight_smooth * (y[k - 1].x + y[k + 1].x - 2.0 * y[k].x),
        y[k].y + config.weight_data * (data[k].y - y[k].y) +
        config.weight_smooth * (y[k - 1].y + y[k + 1].y - 2.0 * y[k].y)};
      if (allowed(k, candidate)) {
        change += distance(y[k], candidate);
        y[k] = candidate;
      }
    }
    if (change < config.tolerance) {
      break;
    }
  }
}

}  // namespace

uint8_t maxCostAlongSegment(const Grid & grid, const Point2D & a, const Point2D & b)
{
  const int samples = std::max(1, static_cast<int>(std::ceil(distance(a, b) / kSegmentSampleStep)));
  uint8_t max_cost = 0;
  for (int k = 0; k <= samples; ++k) {
    const double t = static_cast<double>(k) / samples;
    max_cost = std::max(max_cost, costAt(grid, Point2D{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}));
  }
  return max_cost;
}

std::vector<Point2D> smoothPath(
  const Grid & grid, const std::vector<Cell> & path, const SmoothingConfig & config)
{
  std::vector<Point2D> points;
  points.reserve(path.size());
  if (path.size() <= 2) {
    for (const Cell & cell : path) {
      points.push_back(toPoint(cell));
    }
    return points;
  }

  if (config.shortcut) {
    points = shortcut(grid, path, config.lethal_cost_threshold);
  } else {
    for (const Cell & cell : path) {
      points.push_back(toPoint(cell));
    }
  }

  if (config.gradient) {
    std::vector<Point2D> spaced{points.front()};
    for (std::size_t k = 1; k < points.size(); ++k) {
      appendSubdivided(points[k - 1], points[k], std::max(config.point_spacing, 1.0), &spaced);
    }
    if (spaced.size() > 2) {
      gradientSmooth(grid, config, &spaced);
    }
    points = std::move(spaced);
  }

  std::vector<Point2D> dense{points.front()};
  for (std::size_t k = 1; k < points.size(); ++k) {
    appendSubdivided(points[k - 1], points[k], 1.0, &dense);
  }
  return dense;
}

}  // namespace amr_planning_core
