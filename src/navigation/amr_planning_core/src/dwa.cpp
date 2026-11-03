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
// 이보다 느리면 제자리(v≈0)로 본다 - heading 비용·회전 샘플 처리 기준.
constexpr double kMinMovingSpeed = 1e-3;
// 제자리 회전 모드에서 "실제로 회전하는" 샘플로 보는 최소 각속도(rad/s).
constexpr double kMinTurnRate = 1e-3;

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
  // 나오는 각도가 들어가는 각도보다 크면 모드에 들어가자마자 나와 매 주기 바뀐다
  config_.rotate_in_place_exit_angle =
    std::min(config_.rotate_in_place_exit_angle, config_.rotate_in_place_min_angle);
  base_config_ = config_;
  avoid_config_ = config_;
  avoid_config_.dynamic_obstacle_safety_margin += config_.avoid_margin_add;
  avoid_config_.dynamic_weight *= config_.avoid_weight_scale;
  rollout_time_ = config_.sim_time;
}

void DwaPlanner::setAvoidMode(bool active)
{
  avoid_mode_ = active && base_config_.avoid_mode_enabled;
  config_ = avoid_mode_ ? avoid_config_ : base_config_;
  rollout_time_ = avoid_mode_ ?
    std::max(base_config_.sim_time, base_config_.avoid_rollout_time) : base_config_.sim_time;
}

DynamicWindow DwaPlanner::computeDynamicWindow(const RobotState & state) const
{
  DynamicWindow window;
  window.min_v = std::max(config_.min_v, state.v - config_.acc_v * config_.dt);
  window.max_v = std::min(config_.max_v, state.v + config_.acc_v * config_.dt);
  window.min_w = std::max(-config_.max_w, state.w - config_.acc_w * config_.dt);
  window.max_w = std::min(config_.max_w, state.w + config_.acc_w * config_.dt);
  // 지금 속도가 한계 밖으로 한 주기 변화 이상 벗어나 있으면(예: 처음 속도가 max_v보다 높음, w가 -max_w보다
  // 낮음) min > max로 창이 뒤집혀, 샘플이 한계값까지 한 주기 최대 변화(acc*dt)보다 더 크게 튄다. 그때는 한계
  // 쪽으로 최대로 줄인 값 하나만 남긴다 - 위로 벗어났으면 min(지금 - acc*dt), 아래로 벗어났으면 max(지금 + acc*dt).
  auto keep_reachable = [](double current, double & lo, double & hi) {
      if (lo > hi) {
        if (current > hi) {
          hi = lo;
        } else {
          lo = hi;
        }
      }
    };
  keep_reachable(state.v, window.min_v, window.max_v);
  keep_reachable(state.w, window.min_w, window.max_w);
  return window;
}

std::vector<TrajectoryPoint> DwaPlanner::simulateTrajectory(
  const RobotState & state, double v, double w) const
{
  std::vector<TrajectoryPoint> trajectory;
  const int steps = std::max(1, static_cast<int>(std::round(rollout_time_ / config_.dt)));
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
  const Grid & grid, double x, double y, double yaw, double * max_cell_cost, double margin) const
{
  *max_cell_cost = 0.0;

  // 풋프린트 전체를 격자 해상도의 절반 간격으로 채워서 샘플링한다 — 모서리만 보면
  // 격자 해상도보다 작은 장애물 셀을 놓칠 수 있다.
  const double step = std::max(grid.resolution() * 0.5, 1e-3);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double half_length = config_.footprint_half_length + margin;
  const double half_width = config_.footprint_half_width + margin;

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
  double * min_surface_distance, const std::vector<double> * start_distances) const
{
  // 이미 여유 안인 장애물에서 "지금보다 가까워졌다"를 가르는 허용 오차(m) - 나란히 가며 거리가
  // 그대로인 궤적이 계산 오차로 충돌이 되지 않게 한다.
  constexpr double kCloserTolerance = 1e-3;
  *min_surface_distance = std::numeric_limits<double>::infinity();
  const double predict_t = config_.predict_dynamic_obstacles ? t : 0.0;

  for (std::size_t j = 0; j < obstacles.size(); ++j) {
    const DynamicObstacle & obstacle = obstacles[j];
    const double obs_x = obstacle.x + obstacle.vx * predict_t;
    const double obs_y = obstacle.y + obstacle.vy * predict_t;
    const double to_rect = rectangleSurfaceDistance(
      x, y, yaw, config_.footprint_half_length, config_.footprint_half_width, obs_x, obs_y);
    const double surface_distance = to_rect - obstacle.radius;

    bool hit = surface_distance <= config_.dynamic_obstacle_safety_margin;
    if (hit && start_distances != nullptr &&
      (*start_distances)[j] <= config_.dynamic_obstacle_safety_margin)
    {
      // 지금 이미 여유 안: 지금보다 가까워지거나 닿는 궤적만 충돌로 본다(#216)
      hit = surface_distance <= 0.0 ||
        surface_distance < (*start_distances)[j] - kCloserTolerance;
    }
    if (hit) {
      *min_surface_distance = std::max(0.0, surface_distance);
      return true;
    }
    if (start_distances != nullptr &&
      (*start_distances)[j] <= config_.dynamic_obstacle_safety_margin)
    {
      // 이미 여유 안인 장애물은 궤적을 따라 잰 최소 거리에 넣지 않는다 - 늘 여유 안이라 거리비용이
      // 모든 궤적에서 1.0이 돼 멀어지는 궤적을 고르지 못한다. 대신 궤적 끝 거리로 따로 매긴다
      // (withinMarginObstacleCost).
      continue;
    }
    *min_surface_distance = std::min(*min_surface_distance, surface_distance);
  }

  return false;
}

double DwaPlanner::withinMarginObstacleCost(
  const std::vector<DynamicObstacle> & obstacles, const std::vector<double> & start_distances,
  double t, const TrajectoryPoint & point) const
{
  // 이미 안전 여유 안인 장애물: 궤적 끝(t, point)에서 표면까지 거리를 닿음(0)부터 재서
  // [0, margin + influence_radius]를 1 -> 0으로 매긴다. 나란히 가면(거리 그대로) 1.0에 가깝고 멀어질수록 작다.
  const double range = std::max(
    config_.dynamic_obstacle_safety_margin + config_.dynamic_obstacle_influence_radius, 1e-6);
  const double predict_t = config_.predict_dynamic_obstacles ? t : 0.0;
  double cost = 0.0;
  for (std::size_t j = 0; j < obstacles.size(); ++j) {
    if (start_distances[j] > config_.dynamic_obstacle_safety_margin) {
      continue;
    }
    const DynamicObstacle & obstacle = obstacles[j];
    const double surface_distance = rectangleSurfaceDistance(
      point.x, point.y, point.yaw, config_.footprint_half_length, config_.footprint_half_width,
      obstacle.x + obstacle.vx * predict_t, obstacle.y + obstacle.vy * predict_t) - obstacle.radius;
    cost = std::max(cost, std::clamp(1.0 - surface_distance / range, 0.0, 1.0));
  }
  return cost;
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

double DwaPlanner::goalStoppingVelocity(double goal_distance) const
{
  const double a = std::max(config_.acc_v, 0.0);
  const double a_dt = a * config_.dt;
  return std::max(0.0, std::sqrt(a_dt * a_dt + 2.0 * a * std::max(goal_distance, 0.0)) - a_dt);
}

double DwaPlanner::headingCost(const TrajectoryPoint & trajectory_end, const Point2D & goal) const
{
  const double desired_yaw = std::atan2(goal.y - trajectory_end.y, goal.x - trajectory_end.x);
  const double diff = normalizeAngle(desired_yaw - trajectory_end.yaw);
  return std::abs(diff) / kPi;  // [0, 1]
}

double DwaPlanner::evaluateCost(
  const TrajectoryPoint & trajectory_end, const Point2D & goal, double v,
  double max_cell_cost, bool score_stationary_heading) const
{
  // v가 0에 가까우면 궤적이 사실상 제자리라 trajectory_end == 시작점이다. 그 상태에서
  // 마침 로봇이 목표 쪽을 보고 있으면 heading 비용이 0이 되어, "목표를 보고 가만히
  // 멈춰 있기"가 실제로 전진하는 선택보다 더 싸 보이는 국소최소(local minimum)에
  // 빠진다 — 장애물 앞에서 멈춘 채 영원히 못 움직이는 현상으로 실제 확인했다.
  // 정지는 목표를 향한 "방향"이라고 볼 근거가 없으므로 heading 비용을 최댓값(1.0)으로
  // 고정해 제자리 멈춤이 절대 유리해지지 않게 한다.
  // 단 제자리 회전 모드(목표 방향 오차가 크다, score_stationary_heading)에서는 회전한 뒤 방향
  // (trajectory_end.yaw)으로 heading을 매긴다 - 그래야 목표 쪽으로 도는 회전이 크게 도는 전진 호보다
  // 싸다. 정지(w=0)는 지금 오차 그대로라 유리하지 않다.
  const double heading = (std::abs(v) < kMinMovingSpeed && !score_stationary_heading) ?
    1.0 : headingCost(trajectory_end, goal);
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

  // 지금 자세에서 동적 장애물마다 표면까지 거리 - 이미 안전 여유 안인 장애물은 멀어지는 궤적을
  // 허용하는 기준이다(evaluateDynamicObstacles).
  std::vector<double> start_distances;
  start_distances.reserve(dynamic_obstacles.size());
  for (const DynamicObstacle & obstacle : dynamic_obstacles) {
    start_distances.push_back(rectangleSurfaceDistance(
        state.x, state.y, state.yaw, config_.footprint_half_length, config_.footprint_half_width,
        obstacle.x, obstacle.y) - obstacle.radius);
  }
  bool any_within_margin = false;
  bool dynamic_near = false;
  for (const double d : start_distances) {
    any_within_margin = any_within_margin || d <= config_.dynamic_obstacle_safety_margin;
    dynamic_near = dynamic_near ||
      d <= config_.dynamic_obstacle_safety_margin + config_.dynamic_obstacle_influence_radius;
  }

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
    const double goal_cap = goalStoppingVelocity(d_goal);
    window.max_v = std::max(window.min_v, std::min(window.max_v, goal_cap));
  }

  // [0]: 장애물에 안 닿는 궤적, [1]: 닿지만 허용 속도 조건으로 그 앞에 설 수 있는 궤적.
  // [0]이 하나라도 있으면 [1]은 보지 않는다 - 닿는 궤적까지 비용으로 겨루게 하면 heading이
  // 압도적이라 벽으로 곧장 가다 막판에 못 꺾는다(기존 폐루프 테스트가 실패했다).
  // 제자리 회전 모드(히스테리시스): 지금 자세에서 기준점(lookahead_distance 앞)까지 방향 오차가
  // rotate_in_place_min_angle보다 크면 들어가고 rotate_in_place_exit_angle 이하가 되면 나온다. 이 모드에서는
  // v≈0 샘플도 회전한 뒤 방향으로 heading 비용을 매기고(평소에는 정지 함정 때문에 1.0 고정), 먼저 최대
  // 감속 속도(v=window.min_v, 서 있으면 0)만 샘플링한 회전 후보를 전체 윈도우의 최선(정지 1.0 고정, 평소와
  // 같은 채점)과 비용으로 겨뤄 더 싼 쪽을 고른다 - 회전이 막히거나(좁은 통로 안) 거의 안 도는 샘플뿐이면(w
  // 샘플은 지금 각속도 둘레라 state.w≠0이면 정확한 0이 없다) 전진 쪽이 이긴다. 동적 장애물이
  // safety_margin + influence_radius 안이면 모드에 들어가지 않는다 - 피하느라 경로에서 벗어나 오차가 45°를
  // 넘었을 때 서서 경로 쪽(장애물이 오는 쪽)으로 돌지 않게 한다.
  // 경로를 받는 버전에서만 쓴다 - 먼 목표 점 하나로 부르면(follow_path false) 장애물을 돌아가는 동안 그 점까지
  // 오차가 커져, 회전이 회피와 싸운다(ClosedLoopReachesGoalWithOneObstacle). 경로의 기준점은 장애물을 이미 돌아간다.
  const double heading_error = std::abs(normalizeAngle(
      std::atan2(goal.y - state.y, goal.x - state.x) - state.yaw));
  if (follow_path && config_.rotate_in_place_min_angle > 0.0 && !dynamic_near) {
    rotating_in_place_ = heading_error > (rotating_in_place_ ?
      config_.rotate_in_place_exit_angle : config_.rotate_in_place_min_angle);
  } else {
    rotating_in_place_ = false;
  }
  const bool rotate_mode = rotating_in_place_;

  // 제자리 회전 후보(회전 모드의 회전 탐색)의 풋프린트 여유(rotate_in_place_clearance). 회전 모드 밖 v≈0 회전과
  // 전체 윈도우 탐색은 키우지 않는다 - 전진은 여유 없이 장애물 0.05 m 앞까지 다가가므로, 거기서 모든 회전을 키워
  // 검사하면 정지만 남아 영영 못 돈다. 지금 자세에서 이미 그 여유 안에 장애물이 있으면 이번 주기는 키우지 않는다
  // (같은 이유, 안전 노드도 지금보다 가까워지는 회전만 막는다).
  double unused_cost = 0.0;
  const double turn_clearance = (config_.rotate_in_place_clearance > 0.0 &&
    !evaluateFootprint(grid, state.x, state.y, state.yaw, &unused_cost,
    config_.rotate_in_place_clearance)) ? config_.rotate_in_place_clearance : 0.0;

  struct Best
  {
    bool found[2] = {false, false};
    double cost[2] = {
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    double v[2] = {0.0, 0.0};
    double w[2] = {0.0, 0.0};
  };
  // 비용(heading)을 매기는 궤적 길이: sim_time 시점까지(회피 모드의 긴 궤적은 충돌 검사에만 쓴다)
  const std::size_t cost_steps = static_cast<std::size_t>(
    std::max(1, static_cast<int>(std::round(config_.sim_time / config_.dt))));

  // turn_search: 회전 모드의 회전 후보 탐색이면 true - v≈0 샘플을 회전한 뒤 방향으로 채점하고 회전 샘플은
  // 풋프린트를 turn_clearance만큼 키운다. 전체 윈도우 탐색(false)은 평소처럼 정지를 1.0으로 고정한다 - 아니면
  // "목표 보고 서 있기" 함정이 다시 열린다.
  auto search = [&](const DynamicWindow & sample_window, int v_count, bool turn_search) {
    Best best;
    for (int vi = 0; vi < v_count; ++vi) {
      const double v = sampleValue(sample_window.min_v, sample_window.max_v, vi, v_count);

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
        const double w = sampleValue(sample_window.min_w, sample_window.max_w, wi, config_.w_samples);

        const std::vector<TrajectoryPoint> trajectory = simulateTrajectory(state, v, w);
        const bool turning_in_place = std::abs(v) < kMinMovingSpeed && std::abs(w) > kMinTurnRate;
        const double footprint_margin = (turn_search && turning_in_place) ? turn_clearance : 0.0;

        // 충돌 검사와 함께 장애물에 닿기 전까지 남은 궤적 점 개수(free_steps)를 센다 -
        // 허용 속도 조건의 dist는 |v|*dt*free_steps다(닿는 점 바로 앞까지, 보수적).
        bool collides = false;
        std::size_t free_steps = trajectory.size();
        double max_cell_cost = 0.0;
        double min_dynamic_surface_distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
          const TrajectoryPoint & point = trajectory[i];
          double point_max_cost = 0.0;
          const bool static_hit = evaluateFootprint(
            grid, point.x, point.y, point.yaw, &point_max_cost, footprint_margin);
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
              dynamic_obstacles, t, point.x, point.y, point.yaw, &point_min_distance,
              &start_distances);
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
        if (collides && turning_in_place) {
          // 제자리 회전(v≈0, w≠0)이 궤적 중간에 닿으면 버린다 - 허용 속도 조건(v=0이면 늘 "설 수 있음")은
          // 회전 감속(acc_w)을 안 보므로, 닿기 전에 회전을 멈출 수 있다고 볼 근거가 없다. 사각
          // 풋프린트는 v=0이어도 매 스텝 yaw가 바뀐 자세로 검사한다(대각선 0.72 m > 좁은 통로 0.6 m).
          // 정지(w≈0)는 남긴다 - 다가오는 동적 장애물(예측)과 궤적 중간에 닿아도 그 자리에 서 있는 것은
          // 후보여야 한다(tier 1). 버리면 느린 전진이나 kNoValidTrajectory만 남는다.
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

        // 닿는 궤적은 설 수 있는 마지막 점(닿기 직전)에서 heading을 본다. 회피 모드에서 궤적을 sim_time보다 길게
        // 시뮬레이션해도(avoid_rollout_time) heading은 sim_time 시점까지만 본다 - 기준점(|v|*sim_time 앞)과 맞춘다.
        const TrajectoryPoint & trajectory_end =
          trajectory[std::min({free_steps, heading_steps, cost_steps}) - 1];
        // 동적 거리비용: 여유 밖 장애물은 궤적을 따라 잰 최소 거리, 이미 여유 안인 장애물은 설 수 있는
        // 마지막 점(궤적 끝)에서의 거리 - 둘 중 큰 쪽(#228 연우 리뷰)
        double dynamic_cost = dynamicObstacleCost(min_dynamic_surface_distance);
        if (any_within_margin) {
          dynamic_cost = std::max(dynamic_cost, withinMarginObstacleCost(
              dynamic_obstacles, start_distances, static_cast<double>(free_steps) * config_.dt,
              trajectory[free_steps - 1]));
        }
        const double cost =
          evaluateCost(trajectory_end, heading_target, v, max_cell_cost, turn_search) +
          config_.dynamic_weight * dynamic_cost;
        const int tier = collides ? 1 : 0;
        if (cost < best.cost[tier]) {
          best.cost[tier] = cost;
          best.v[tier] = v;
          best.w[tier] = w;
          best.found[tier] = true;
        }
      }
    }
    return best;
  };

  Best best;
  bool chosen = false;
  if (rotate_mode) {
    DynamicWindow turn_window = window;
    turn_window.max_v = turn_window.min_v;
    best = search(turn_window, 1, true);
    if (best.found[0] && std::abs(best.w[0]) > kMinTurnRate) {
      // 회전 후보의 궤적 끝 방향 오차가 지금보다 min_gain 이상 줄 때만 채택한다. w 샘플은 지금 각속도 둘레라
      // state.w≠0이면 정확한 0이 없어, 회전이 양쪽 다 막혀도 거의 안 도는 샘플(예: w=0.01)이 남는다 - 그걸
      // 채택하면 전체 윈도우를 안 보고 그 자리에 선다.
      // min_gain = acc_w * dt * sim_time / 2: 서 있다가 한 주기에 낼 수 있는 최대 각속도(acc_w*dt)로 sim_time 동안
      // 돌면 줄어드는 오차의 절반(기본값 3.2 * 0.1 * 2.0 / 2 = 0.32 rad). 정상적인 회전은 출발 주기에도 이만큼
      // 줄이고, 막혀서 거의 안 도는 샘플은 못 넘는다. 설정값에서 계산하므로 acc_w·dt·sim_time이 바뀌면 같이 바뀐다.
      // 전체 윈도우 최선과 비용으로 겨루는 안은 velocity_weight 때문에 전진 호가 늘 이겨 출발 때 크게 돌았다.
      const std::vector<TrajectoryPoint> turn = simulateTrajectory(state, best.v[0], best.w[0]);
      const TrajectoryPoint end = turn[std::min(turn.size(), cost_steps) - 1];
      const double end_error = std::abs(normalizeAngle(
          std::atan2(goal.y - end.y, goal.x - end.x) - end.yaw));
      const double min_gain = 0.5 * config_.acc_w * config_.dt * config_.sim_time;
      chosen = end_error < heading_error - min_gain;
    }
  }
  if (!chosen) {
    best = search(window, config_.v_samples, false);
  }

  const int tier = best.found[0] ? 0 : 1;
  if (!best.found[tier]) {
    result.status = DwaStatus::kNoValidTrajectory;
    return result;
  }

  result.status = DwaStatus::kOk;
  result.v = best.v[tier];
  result.w = best.w[tier];
  return result;
}

}  // namespace amr_planning_core
