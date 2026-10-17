#include "amr_planning_core/dwa.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace amr_planning_core
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

Cell worldToCell(double x, double y, double resolution)
{
  return Cell{
    static_cast<int>(std::floor(x / resolution)),
    static_cast<int>(std::floor(y / resolution))};
}

// [min_val, max_val] 구간을 count개로 균등 샘플링한 index번째 값.
double sampleValue(double min_val, double max_val, int index, int count)
{
  if (count <= 1) {
    return min_val;
  }
  const double ratio = static_cast<double>(index) / static_cast<double>(count - 1);
  return min_val + (max_val - min_val) * ratio;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::size_t closestPathIndex(const std::vector<Point2D> & path, double x, double y)
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - x, path[i].y - y);
    if (d < best_distance) {
      best_distance = d;
      best_index = i;
    }
  }
  return best_index;
}

// path가 비어 있지 않고 start가 로봇에 가장 가까운 점일 때의 pathPointAhead.
PathTarget pathPointAheadFrom(
  const std::vector<Point2D> & path, std::size_t start, double robot_x, double robot_y,
  double distance)
{
  PathTarget target;
  const double to_start = std::hypot(path[start].x - robot_x, path[start].y - robot_y);
  double accumulated = 0.0;
  Point2D previous = path[start];
  for (std::size_t i = start; i < path.size(); ++i) {
    accumulated += std::hypot(path[i].x - previous.x, path[i].y - previous.y);
    if (accumulated >= distance) {
      target.point = path[i];
      target.distance_from_robot = to_start + accumulated;
      return target;
    }
    previous = path[i];
  }
  target.point = path.back();
  target.distance_from_robot = to_start + accumulated;
  target.at_path_end = true;
  return target;
}

}  // namespace

PathTarget pathPointAhead(
  const std::vector<Point2D> & path, double robot_x, double robot_y, double distance)
{
  if (path.empty()) {
    PathTarget target;
    target.point = Point2D{robot_x, robot_y};
    target.at_path_end = true;
    return target;
  }
  return pathPointAheadFrom(
    path, closestPathIndex(path, robot_x, robot_y), robot_x, robot_y, distance);
}

DwaPlanner::DwaPlanner(DwaConfig config)
: config_(config)
{
}

DynamicWindow DwaPlanner::computeDynamicWindow(const RobotState & state) const
{
  DynamicWindow window;
  window.min_v = std::max(config_.min_v, state.v - config_.acc_v * config_.dt);
  window.max_v = std::min(config_.max_v, state.v + config_.acc_v * config_.dt);
  window.min_w = std::max(-config_.max_w, state.w - config_.acc_w * config_.dt);
  window.max_w = std::min(config_.max_w, state.w + config_.acc_w * config_.dt);
  return window;
}

std::vector<TrajectoryPoint> DwaPlanner::simulateTrajectory(
  const RobotState & state, double v, double w) const
{
  std::vector<TrajectoryPoint> trajectory;
  const int steps = std::max(1, static_cast<int>(std::round(config_.sim_time / config_.dt)));
  trajectory.reserve(static_cast<std::size_t>(steps));

  double x = state.x;
  double y = state.y;
  double yaw = state.yaw;

  for (int i = 0; i < steps; ++i) {
    x += v * std::cos(yaw) * config_.dt;
    y += v * std::sin(yaw) * config_.dt;
    yaw = normalizeAngle(yaw + w * config_.dt);
    trajectory.push_back({x, y, yaw});
  }

  return trajectory;
}

bool DwaPlanner::evaluateFootprint(
  const Grid & grid, double x, double y, double yaw, double * max_cell_cost) const
{
  *max_cell_cost = 0.0;

  // 풋프린트 전체를 격자 해상도의 절반 간격으로 채워서 샘플링한다 — 모서리만 보면
  // 격자 해상도보다 작은 장애물 셀을 놓칠 수 있다.
  const double step = std::max(grid.resolution() * 0.5, 1e-3);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double half_length = config_.footprint_half_length;
  const double half_width = config_.footprint_half_width;

  for (double lx = -half_length; lx <= half_length + 1e-9; lx += step) {
    for (double ly = -half_width; ly <= half_width + 1e-9; ly += step) {
      const double wx = x + lx * cos_yaw - ly * sin_yaw;
      const double wy = y + lx * sin_yaw + ly * cos_yaw;

      const Cell cell = worldToCell(wx, wy, grid.resolution());
      if (!grid.inBounds(cell)) {
        // 로컬 코스트맵 밖은 아직 모르는 영역으로 취급한다(장애물로 가정하지 않음).
        continue;
      }

      const uint8_t cost = grid.cost(cell);
      if (cost >= config_.lethal_cost_threshold) {
        *max_cell_cost = static_cast<double>(cost);
        return true;
      }
      *max_cell_cost = std::max(*max_cell_cost, static_cast<double>(cost));
    }
  }

  return false;
}

bool DwaPlanner::footprintCollides(const Grid & grid, double x, double y, double yaw) const
{
  double unused_cost = 0.0;
  return evaluateFootprint(grid, x, y, yaw, &unused_cost);
}

namespace
{

// (px, py)에서 중심 (x, y), 회전 yaw, 반폭(half_length, half_width)인 사각형
// 표면까지의 최단 거리. 점이 사각형 안이면 0.
double rectangleSurfaceDistance(
  double x, double y, double yaw, double half_length, double half_width, double px, double py)
{
  // 사각형의 로컬 좌표계로 변환(역회전).
  const double dx = px - x;
  const double dy = py - y;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_x = dx * cos_yaw + dy * sin_yaw;
  const double local_y = -dx * sin_yaw + dy * cos_yaw;

  const double excess_x = std::max(0.0, std::abs(local_x) - half_length);
  const double excess_y = std::max(0.0, std::abs(local_y) - half_width);
  return std::hypot(excess_x, excess_y);
}

}  // namespace

bool DwaPlanner::evaluateDynamicObstacles(
  const std::vector<DynamicObstacle> & obstacles, double t, double x, double y, double yaw,
  double * min_surface_distance) const
{
  *min_surface_distance = std::numeric_limits<double>::infinity();
  const double predict_t = config_.predict_dynamic_obstacles ? t : 0.0;

  for (const DynamicObstacle & obstacle : obstacles) {
    const double obs_x = obstacle.x + obstacle.vx * predict_t;
    const double obs_y = obstacle.y + obstacle.vy * predict_t;
    const double to_rect = rectangleSurfaceDistance(
      x, y, yaw, config_.footprint_half_length, config_.footprint_half_width, obs_x, obs_y);
    const double surface_distance = to_rect - obstacle.radius;

    if (surface_distance <= config_.dynamic_obstacle_safety_margin) {
      *min_surface_distance = std::max(0.0, surface_distance);
      return true;
    }
    *min_surface_distance = std::min(*min_surface_distance, surface_distance);
  }

  return false;
}

bool DwaPlanner::dynamicObstacleCollides(
  const std::vector<DynamicObstacle> & obstacles, double t, double x, double y, double yaw) const
{
  double unused_distance = 0.0;
  return evaluateDynamicObstacles(obstacles, t, x, y, yaw, &unused_distance);
}

double DwaPlanner::dynamicObstacleCost(double min_surface_distance) const
{
  const double range = std::max(config_.dynamic_obstacle_influence_radius, 1e-6);
  const double beyond_margin = min_surface_distance - config_.dynamic_obstacle_safety_margin;
  const double cost = 1.0 - beyond_margin / range;
  return std::clamp(cost, 0.0, 1.0);
}

double DwaPlanner::admissibleVelocity(double obstacle_distance) const
{
  return std::sqrt(2.0 * std::max(config_.acc_v, 0.0) * std::max(obstacle_distance, 0.0));
}

double DwaPlanner::headingCost(const TrajectoryPoint & trajectory_end, const Point2D & goal) const
{
  const double desired_yaw = std::atan2(goal.y - trajectory_end.y, goal.x - trajectory_end.x);
  const double diff = normalizeAngle(desired_yaw - trajectory_end.yaw);
  return std::abs(diff) / kPi;  // [0, 1]
}

double DwaPlanner::evaluateCost(
  const TrajectoryPoint & trajectory_end, const Point2D & goal, double v,
  double max_cell_cost) const
{
  // v가 0에 가까우면 궤적이 사실상 제자리라 trajectory_end == 시작점이다. 그 상태에서
  // 마침 로봇이 목표 쪽을 보고 있으면 heading 비용이 0이 되어, "목표를 보고 가만히
  // 멈춰 있기"가 실제로 전진하는 선택보다 더 싸 보이는 국소최소(local minimum)에
  // 빠진다 — 장애물 앞에서 멈춘 채 영원히 못 움직이는 현상으로 실제 확인했다.
  // 정지는 목표를 향한 "방향"이라고 볼 근거가 없으므로 heading 비용을 최댓값(1.0)으로
  // 고정해 제자리 멈춤이 절대 유리해지지 않게 한다.
  constexpr double kMinMovingSpeed = 1e-3;
  const double heading = (std::abs(v) < kMinMovingSpeed) ? 1.0 : headingCost(trajectory_end, goal);
  const double clearance = max_cell_cost / 255.0;  // [0, 1]
  const double max_v = std::max(config_.max_v, 1e-6);
  const double velocity = (max_v - v) / max_v;  // [0, 1], 0 = 최고 속도

  return config_.heading_weight * heading +
         config_.clearance_weight * clearance +
         config_.velocity_weight * velocity;
}

DwaCommand DwaPlanner::rotateToGoalYaw(
  const RobotState & state, double goal_yaw, double yaw_tolerance) const
{
  DwaCommand result;
  result.v = 0.0;

  const double yaw_error = normalizeAngle(goal_yaw - state.yaw);
  if (std::abs(yaw_error) <= yaw_tolerance) {
    result.status = DwaStatus::kGoalReached;
    return result;
  }

  // speed_cap_for_stop(amr_zone_client/geometry.py)과 같은 발상: 등가감속(acc_w)으로
  // 남은 오차만큼 정확히 서는 각속도 상한을 goal_yaw 쪽 부호로 준다 - max_w로 돌다가
  // 멈출 거리(각도)보다 오차가 작아지면 자연히 느려져서 넘어가지 않는다.
  const double stopping_speed = std::sqrt(2.0 * config_.acc_w * std::abs(yaw_error));
  const double desired_w_magnitude = std::min(config_.max_w, stopping_speed);
  const double desired_w = (yaw_error > 0.0) ? desired_w_magnitude : -desired_w_magnitude;

  const double max_dw = config_.acc_w * config_.dt;
  const double dw = std::clamp(desired_w - state.w, -max_dw, max_dw);

  result.status = DwaStatus::kOk;
  result.w = state.w + dw;
  return result;
}

DwaCommand DwaPlanner::computeVelocityCommand(
  const RobotState & state, const Point2D & goal, const Grid & grid,
  const std::vector<DynamicObstacle> & dynamic_obstacles,
  const std::optional<Point2D> & final_goal) const
{
  return selectCommand(state, {goal}, false, grid, dynamic_obstacles, final_goal);
}

DwaCommand DwaPlanner::computeVelocityCommand(
  const RobotState & state, const std::vector<Point2D> & path, const Grid & grid,
  const std::vector<DynamicObstacle> & dynamic_obstacles) const
{
  if (path.empty()) {
    DwaCommand result;
    result.status = DwaStatus::kGoalReached;
    return result;
  }
  return selectCommand(state, path, true, grid, dynamic_obstacles, path.back());
}

DwaCommand DwaPlanner::selectCommand(
  const RobotState & state, const std::vector<Point2D> & path, bool follow_path,
  const Grid & grid, const std::vector<DynamicObstacle> & dynamic_obstacles,
  const std::optional<Point2D> & final_goal) const
{
  DwaCommand result;

  const std::size_t path_start = closestPathIndex(path, state.x, state.y);
  const double base_distance = follow_path ? config_.lookahead_distance : 0.0;
  const Point2D goal =
    pathPointAheadFrom(path, path_start, state.x, state.y, base_distance).point;
  const bool adaptive = follow_path && config_.adaptive_lookahead;

  const double dx = goal.x - state.x;
  const double dy = goal.y - state.y;
  if (std::hypot(dx, dy) <= config_.goal_tolerance) {
    result.status = DwaStatus::kGoalReached;
    return result;
  }

  DynamicWindow window = computeDynamicWindow(state);
  if (config_.use_goal_slowdown && final_goal.has_value()) {
    // 목표 앞 감속: acc_v로 줄여 최종 목표에 서는 속도까지만 샘플링한다. 그만큼 빨리 못
    // 줄이면(윈도우 최저 속도가 더 높으면) 최대 감속인 최저 속도만 남긴다.
    const double d_goal = std::hypot(final_goal->x - state.x, final_goal->y - state.y);
    const double goal_cap = admissibleVelocity(d_goal);
    window.max_v = std::max(window.min_v, std::min(window.max_v, goal_cap));
  }

  // [0]: 장애물에 안 닿는 궤적, [1]: 닿지만 허용 속도 조건으로 그 앞에 설 수 있는 궤적.
  // [0]이 하나라도 있으면 [1]은 보지 않는다 - 닿는 궤적까지 비용으로 겨루게 하면 heading이
  // 압도적이라 벽으로 곧장 가다 막판에 못 꺾는다(기존 폐루프 테스트가 실패했다).
  bool found[2] = {false, false};
  double best_cost[2] = {
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  double best_v[2] = {0.0, 0.0};
  double best_w[2] = {0.0, 0.0};

  for (int vi = 0; vi < config_.v_samples; ++vi) {
    const double v = sampleValue(window.min_v, window.max_v, vi, config_.v_samples);

    // heading 기준점: 고정이면 goal, adaptive면 이 속도의 궤적 끝보다 lookahead_distance 앞의 경로 점
    // (궤적 길이 + lookahead_distance). 궤적 끝과 기준점이 붙어 있으면 끝 -> 기준점 방향이 튄다(#216).
    // 기준점이 경로 끝점으로 잘리면(궤적 끝 + lookahead_distance가 경로 끝을 넘음) 끝점보다
    // lookahead_distance 앞인 궤적 점까지만 보고 heading을 잰다 - 같은 이유로, 끝에 가까운 빠른 궤적의
    // 끝과 경로 끝점이 붙어 방향이 튀지 않게 한다(지나치는 궤적은 끝점을 넘은 쪽에서 보면 뒤집힌다).
    Point2D heading_target = goal;
    std::size_t heading_steps = std::numeric_limits<std::size_t>::max();
    if (adaptive) {
      const double travel = std::abs(v) * config_.sim_time;
      const PathTarget target = pathPointAheadFrom(
        path, path_start, state.x, state.y, travel + config_.lookahead_distance);
      heading_target = target.point;
      const double step_length = std::abs(v) * config_.dt;
      const double before_end = target.distance_from_robot - config_.lookahead_distance;
      if (target.at_path_end && before_end < travel && step_length > 0.0) {
        heading_steps =
          static_cast<std::size_t>(std::max(1.0, std::floor(before_end / step_length)));
      }
    }

    for (int wi = 0; wi < config_.w_samples; ++wi) {
      const double w = sampleValue(window.min_w, window.max_w, wi, config_.w_samples);

      const std::vector<TrajectoryPoint> trajectory = simulateTrajectory(state, v, w);

      // 충돌 검사와 함께 장애물에 닿기 전까지 남은 궤적 점 개수(free_steps)를 센다 -
      // 허용 속도 조건의 dist는 |v|*dt*free_steps다(닿는 점 바로 앞까지, 보수적).
      bool collides = false;
      std::size_t free_steps = trajectory.size();
      double max_cell_cost = 0.0;
      double min_dynamic_surface_distance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const TrajectoryPoint & point = trajectory[i];
        double point_max_cost = 0.0;
        const bool static_hit =
          evaluateFootprint(grid, point.x, point.y, point.yaw, &point_max_cost);
        max_cell_cost = std::max(max_cell_cost, point_max_cost);
        if (static_hit) {
          collides = true;
          free_steps = i;
          break;
        }

        if (!dynamic_obstacles.empty()) {
          const double t = static_cast<double>(i + 1) * config_.dt;
          double point_min_distance = std::numeric_limits<double>::infinity();
          const bool dynamic_hit = evaluateDynamicObstacles(
            dynamic_obstacles, t, point.x, point.y, point.yaw, &point_min_distance);
          min_dynamic_surface_distance = std::min(min_dynamic_surface_distance, point_min_distance);
          if (dynamic_hit) {
            collides = true;
            free_steps = i;
            break;
          }
        }
      }
      if (trajectory.empty()) {
        continue;
      }
      if (config_.use_admissible_velocity) {
        // 첫 점부터 닿으면 멈출 거리가 없다. 그 밖에는 닿는 궤적도 acc_v로 그 앞에 설 수
        // 있으면 [1] 후보로 남긴다.
        const double dist = std::abs(v) * config_.dt * static_cast<double>(free_steps);
        if (free_steps == 0 || std::abs(v) > admissibleVelocity(dist)) {
          continue;
        }
      } else if (collides) {
        continue;
      }

      // 닿는 궤적은 설 수 있는 마지막 점(닿기 직전)에서 heading을 본다.
      const TrajectoryPoint & trajectory_end =
        trajectory[std::min(free_steps, heading_steps) - 1];
      const double cost = evaluateCost(trajectory_end, heading_target, v, max_cell_cost) +
        config_.dynamic_weight * dynamicObstacleCost(min_dynamic_surface_distance);
      const int tier = collides ? 1 : 0;
      if (cost < best_cost[tier]) {
        best_cost[tier] = cost;
        best_v[tier] = v;
        best_w[tier] = w;
        found[tier] = true;
      }
    }
  }

  const int tier = found[0] ? 0 : 1;
  if (!found[tier]) {
    result.status = DwaStatus::kNoValidTrajectory;
    return result;
  }

  result.status = DwaStatus::kOk;
  result.v = best_v[tier];
  result.w = best_w[tier];
  return result;
}

}  // namespace amr_planning_core
