#include "amr_planning_core/passing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_planning_core
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// 0 → 1 S자 (양 끝 기울기 0)
double smooth(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

// i번째 점의 경로 방향 (단위 벡터). 마지막 점은 앞 구간 방향
Point2D tangentAt(const std::vector<Point2D> & path, std::size_t i)
{
  const std::size_t a = (i + 1 < path.size()) ? i : i - 1;
  const double dx = path[a + 1].x - path[a].x;
  const double dy = path[a + 1].y - path[a].y;
  const double n = std::hypot(dx, dy);
  return n > 1e-9 ? Point2D{dx / n, dy / n} : Point2D{1.0, 0.0};
}

}  // namespace

PassingManeuver::PassingManeuver(const PassingConfig & config)
: config_(config)
{
}

void PassingManeuver::resetManeuver() noexcept
{
  phase_ = PassingPhase::kIdle;
  offset_ = 0.0;
  s_start_ = 0.0;
  s_return_ = 0.0;
  clear_count_ = 0;
}

void PassingManeuver::reset() noexcept
{
  resetManeuver();
  armed_ = false;
  memory_.reset();
}

std::vector<double> PassingManeuver::arcLengths(const std::vector<Point2D> & path)
{
  std::vector<double> s(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return s;
}

std::optional<PassingManeuver::Frenet> PassingManeuver::project(
  const std::vector<Point2D> & path, const std::vector<double> & s, const Point2D & p)
{
  if (path.size() < 2 || s.size() != path.size()) {
    return std::nullopt;
  }
  std::optional<Frenet> best;
  double best_d2 = kInf;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double dx = path[i + 1].x - ax;
    const double dy = path[i + 1].y - ay;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) {
      continue;
    }
    // 첫 구간은 시작점 뒤로 늘린다. 시작점에 붙이면 경로 시작보다 뒤에 있는 점이 s = 0(로봇 바로 옆)으로 잡힌다 - 새 전역
    // 경로(로봇 자리에서 시작)를 받은 직후 이미 뒤로 지나간 작업자를 앞 위협으로 보고 한 번 더 비켰다
    // (정면 접근 시험 11/4 v4, 복귀 6.3 s)
    const double lo = (i == 0) ? -kInf : 0.0;
    const double t = std::clamp(((p.x - ax) * dx + (p.y - ay) * dy) / len2, lo, 1.0);
    const double qx = ax + t * dx;
    const double qy = ay + t * dy;
    const double d2 = (p.x - qx) * (p.x - qx) + (p.y - qy) * (p.y - qy);
    if (d2 < best_d2) {
      best_d2 = d2;
      const double len = std::sqrt(len2);
      Frenet f;
      f.tx = dx / len;
      f.ty = dy / len;
      f.s = s[i] + t * len;
      // 왼쪽 +: 경로 방향 × (p - q)의 z 성분 부호
      f.d = f.tx * (p.y - qy) - f.ty * (p.x - qx);
      best = f;
    }
  }
  return best;
}

double PassingManeuver::lateral(double s) const noexcept
{
  if (!active() || s < s_start_) {
    return 0.0;
  }
  if (s < s_start_ + config_.ramp_out) {
    return offset_ * smooth((s - s_start_) / config_.ramp_out);
  }
  if (s < s_return_) {
    return offset_;
  }
  if (s < s_return_ + config_.ramp_in) {
    return offset_ * (1.0 - smooth((s - s_return_) / config_.ramp_in));
  }
  return 0.0;
}

std::optional<PassingManeuver::Threat> PassingManeuver::nearestThreat(
  const std::vector<Point2D> & path, const std::vector<double> & s, double s_robot,
  const std::vector<DynamicObstacle> & obstacles, double band) const
{
  std::optional<Threat> best;
  for (const DynamicObstacle & o : obstacles) {
    const double speed = std::hypot(o.vx, o.vy);
    if (speed < config_.min_obstacle_speed) {
      continue;
    }
    const std::optional<Frenet> f = project(path, s, Point2D{o.x, o.y});
    if (!f) {
      continue;
    }
    const double along = o.vx * f->tx + o.vy * f->ty;   // 경로 방향 +
    if (along / speed > config_.max_heading_cos) {
      continue;                                          // 마주 오지 않는다 (가로지르기·같은 방향)
    }
    const double gap = f->s - s_robot;
    const double passed = -(config_.footprint_half_length + o.radius + config_.pass_margin);
    if (gap <= passed || gap > config_.max_lookahead) {
      continue;                                          // 이미 뒤로 지나갔거나 너무 멀다
    }
    if (std::abs(f->d) >= o.radius + config_.footprint_half_width + band) {
      continue;                                          // 경로를 막지 않는다
    }
    if (!best || gap < best->s - s_robot) {
      best = Threat{f->s, f->d, o.radius, -along};
    }
  }
  return best;
}

std::optional<PassingManeuver::Threat> PassingManeuver::threat(
  const std::vector<Point2D> & path, const std::vector<double> & s, double s_robot,
  const std::vector<DynamicObstacle> & obstacles, double band, double now)
{
  const std::optional<Threat> seen = nearestThreat(path, s, s_robot, obstacles, band);
  if (seen) {
    memory_ = seen;
    memory_t_ = now;
    return seen;
  }
  if (!memory_ || now - memory_t_ > config_.threat_memory) {
    return std::nullopt;
  }
  Threat predicted = *memory_;
  predicted.s -= predicted.closing_speed * (now - memory_t_);
  const double passed = -(config_.footprint_half_length + predicted.radius + config_.pass_margin);
  if (predicted.s - s_robot <= passed) {
    return std::nullopt;                                 // 예측으로도 이미 뒤로 지나갔다
  }
  return predicted;
}

bool PassingManeuver::pathFree(
  const std::vector<Point2D> & path, const std::vector<double> & s, const Grid & grid,
  double s_from, double s_to) const
{
  const double res = grid.resolution();
  const double r = config_.footprint_half_width + config_.free_margin;
  const int reach = static_cast<int>(std::ceil(r / res));
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (s[i] < s_from || s[i] > s_to) {
      continue;
    }
    const double d = lateral(s[i]);
    const Point2D t = tangentAt(path, i);
    const double px = path[i].x - t.y * d;
    const double py = path[i].y + t.x * d;
    const int cx = static_cast<int>(std::floor(px / res));
    const int cy = static_cast<int>(std::floor(py / res));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        const Cell c{cx + dx, cy + dy};
        if (!grid.inBounds(c)) {
          continue;
        }
        const double ox = (c.x + 0.5) * res - px;
        const double oy = (c.y + 0.5) * res - py;
        if (ox * ox + oy * oy <= r * r && grid.cost(c) >= config_.lethal_cost_threshold) {
          return false;
        }
      }
    }
  }
  return true;
}

std::vector<Point2D> PassingManeuver::shifted(
  const std::vector<Point2D> & path, const std::vector<double> & s) const
{
  std::vector<Point2D> out = path;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = lateral(s[i]);
    if (d == 0.0) {
      continue;
    }
    const Point2D t = tangentAt(path, i);
    out[i].x = path[i].x - t.y * d;
    out[i].y = path[i].y + t.x * d;
  }
  return out;
}

bool PassingManeuver::samePath(const std::vector<Point2D> & path) const noexcept
{
  return path.size() == path_size_ &&
         std::abs(path.front().x - path_front_.x) < 1e-6 &&
         std::abs(path.front().y - path_front_.y) < 1e-6 &&
         std::abs(path.back().x - path_back_.x) < 1e-6 &&
         std::abs(path.back().y - path_back_.y) < 1e-6;
}

std::vector<Point2D> PassingManeuver::update(
  const RobotState & robot, const std::vector<Point2D> & path, const Grid & grid,
  const std::vector<DynamicObstacle> & obstacles, double now)
{
  if (!config_.enabled || path.size() < 2) {
    reset();
    return path;
  }
  // 롤링 costmap이라 경로 좌표가 매 주기 같은 양만큼 옮겨진다 - 시작·끝 점의 차이가 같으면 같은 경로로 본다
  if (!samePath(path)) {
    const bool same_shape = path.size() == path_size_ &&
      std::abs((path.back().x - path.front().x) - (path_back_.x - path_front_.x)) < 1e-6 &&
      std::abs((path.back().y - path.front().y) - (path_back_.y - path_front_.y)) < 1e-6;
    if (!same_shape) {
      reset();
    }
    path_size_ = path.size();
    path_front_ = path.front();
    path_back_ = path.back();
  }
  const std::vector<double> s = arcLengths(path);
  const std::optional<Frenet> r = project(path, s, Point2D{robot.x, robot.y});
  if (!r) {
    return path;
  }
  const double s_robot = r->s;
  const double need = config_.footprint_half_width + config_.clearance;

  if (phase_ == PassingPhase::kIdle) {
    armed_ = false;
    const std::optional<Threat> th = threat(path, s, s_robot, obstacles, config_.clearance, now);
    if (!th) {
      return path;
    }
    const double v = std::max(robot.v, config_.min_robot_speed);
    const double closing = v + std::max(th->closing_speed, 0.0);
    const double contact_gap = th->s - s_robot - th->radius - config_.footprint_half_length;
    const double time_to_meet = contact_gap / closing;
    const bool early = time_to_meet > config_.ramp_out / v + config_.start_margin;
    const int away = (std::abs(th->d) < config_.center_band) ? config_.preferred_side :
      (th->d > 0.0 ? -1 : 1);
    for (const int side : {away, -away}) {
      const double target = th->d + side * (th->radius + need);
      if (std::abs(target) > config_.max_offset) {
        continue;
      }
      phase_ = PassingPhase::kOut;
      offset_ = target;
      s_start_ = s_robot;
      s_return_ = kInf;
      // 나가기 + 만날 때까지 + 지나칠 때까지 + 들어오기 구간이 정적 장애물에서 비어 있어야 한다
      const double hold =
        std::max(contact_gap, 0.0) + 2.0 * (th->radius + config_.footprint_half_length);
      if (pathFree(path, s, grid, s_robot, s_robot + config_.ramp_out + hold + config_.ramp_in)) {
        if (early) {
          resetManeuver();                               // 아직 이르다: 비킬 수 있다는 것만 기억한다
          armed_ = true;
          return path;
        }
        return shifted(path, s);
      }
      resetManeuver();
    }
    return path;                                         // 비킬 자리가 없다: 기존 동작(감속·정지)
  }

  // 나가는 중·유지 중: 막는 장애물이 남았는지 (비킨 만큼 넓은 띠로 본다)
  if (phase_ == PassingPhase::kOut || phase_ == PassingPhase::kHold) {
    const std::optional<Threat> th =
      threat(path, s, s_robot, obstacles, config_.clearance + std::abs(offset_), now);
    clear_count_ = th ? 0 : clear_count_ + 1;
    if (clear_count_ >= config_.clear_cycles && s_return_ == kInf) {
      s_return_ = std::max(s_robot, s_start_ + config_.ramp_out);
    }
    phase_ = (s_robot < s_start_ + config_.ramp_out) ? PassingPhase::kOut : PassingPhase::kHold;
  }
  if (s_robot >= s_return_) {
    phase_ = PassingPhase::kReturn;
  }
  if (phase_ == PassingPhase::kReturn && s_robot >= s_return_ + config_.ramp_in) {
    reset();
    return path;
  }
  return shifted(path, s);
}

}  // namespace amr_planning_core
