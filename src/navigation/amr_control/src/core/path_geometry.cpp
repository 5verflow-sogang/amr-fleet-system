#include "amr_control/core/path_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_control
{

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

std::vector<double> cumulativeLengths(const Path & path)
{
  std::vector<double> cum(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return cum;
}

PathProjection projectOntoPath(
  const Path & path, const std::vector<double> & cum_lengths, double x, double y,
  std::size_t first_segment, double max_s)
{
  PathProjection best;
  if (path.empty() || cum_lengths.size() != path.size()) {
    return best;
  }
  if (path.size() == 1) {
    best.valid = true;
    best.x = path[0].x;
    best.y = path[0].y;
    best.cte = std::hypot(x - path[0].x, y - path[0].y);
    return best;
  }

  double best_d2 = std::numeric_limits<double>::infinity();
  const std::size_t last_segment = path.size() - 2;
  const std::size_t start = std::min(first_segment, last_segment);
  for (std::size_t i = start; i <= last_segment; ++i) {
    // 첫 선분은 max_s와 상관없이 본다 (결과가 항상 있도록)
    if (i > start && cum_lengths[i] > max_s) {
      break;
    }
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double dx = path[i + 1].x - ax;
    const double dy = path[i + 1].y - ay;
    const double len2 = dx * dx + dy * dy;
    // 겹친 점(길이 0 선분)은 방향이 없으므로 건너뛴다
    if (len2 <= 0.0) {
      continue;
    }
    const double t = std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0);
    const double px = ax + t * dx;
    const double py = ay + t * dy;
    const double d2 = (x - px) * (x - px) + (y - py) * (y - py);
    // 같은 거리면 앞쪽 선분을 고른다 (꺾이는 점에서 다음 선분 방향을 쓰게)
    if (d2 <= best_d2) {
      best_d2 = d2;
      best.segment = i;
      best.t = t;
      best.x = px;
      best.y = py;
    }
  }

  if (!std::isfinite(best_d2)) {
    // 찾은 범위의 선분이 전부 길이 0
    best.valid = true;
    best.segment = start;
    best.x = path[start].x;
    best.y = path[start].y;
    best.s = cum_lengths[start];
    best.cte = std::hypot(x - best.x, y - best.y);
    return best;
  }

  const PathPoint & a = path[best.segment];
  const PathPoint & b = path[best.segment + 1];
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  best.valid = true;
  best.s = cum_lengths[best.segment] + best.t * len;
  best.yaw = std::atan2(dy, dx);
  const double dist = std::sqrt(best_d2);
  // 진행 방향 × (로봇 − 투영점)의 z 성분이 +면 왼쪽
  const double cross = dx * (y - best.y) - dy * (x - best.x);
  best.cte = cross >= 0.0 ? dist : -dist;
  return best;
}

PathPoint pointAtArcLength(const Path & path, const std::vector<double> & cum_lengths, double s)
{
  if (path.empty() || cum_lengths.size() != path.size()) {
    return PathPoint{};
  }
  if (s <= 0.0) {
    return path.front();
  }
  if (s >= cum_lengths.back()) {
    return path.back();
  }
  // cum_lengths[i] <= s < cum_lengths[i + 1]인 i
  const auto it = std::upper_bound(cum_lengths.begin(), cum_lengths.end(), s);
  const std::size_t i = static_cast<std::size_t>(it - cum_lengths.begin()) - 1;
  const double seg = cum_lengths[i + 1] - cum_lengths[i];
  const double t = seg > 0.0 ? (s - cum_lengths[i]) / seg : 0.0;
  return PathPoint{
    path[i].x + t * (path[i + 1].x - path[i].x),
    path[i].y + t * (path[i + 1].y - path[i].y)};
}

double curvatureAt(
  const Path & path, const std::vector<double> & cum_lengths, double s, double half_span)
{
  if (path.size() < 3 || cum_lengths.size() != path.size()) {
    return 0.0;
  }
  const double total = cum_lengths.back();
  const double s1 = std::max(0.0, s - half_span);
  const double s3 = std::min(total, s + half_span);
  const PathPoint p1 = pointAtArcLength(path, cum_lengths, s1);
  const PathPoint p2 = pointAtArcLength(path, cum_lengths, 0.5 * (s1 + s3));
  const PathPoint p3 = pointAtArcLength(path, cum_lengths, s3);
  const double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
  const double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
  const double c = std::hypot(p3.x - p1.x, p3.y - p1.y);
  if (a * b * c <= 1e-12) {
    return 0.0;
  }
  const double cross = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
  return 2.0 * std::abs(cross) / (a * b * c);
}

}  // namespace amr_control
