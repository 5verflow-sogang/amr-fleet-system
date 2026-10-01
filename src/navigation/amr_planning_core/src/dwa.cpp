#include "amr_planning_core/dwa.hpp"

#include <algorithm>
#include <cmath>
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

}  // namespace

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

DwaCommand DwaPlanner::computeVelocityCommand(
  const RobotState & state, const Point2D & goal, const Grid & grid) const
{
  DwaCommand result;

  const double dx = goal.x - state.x;
  const double dy = goal.y - state.y;
  if (std::hypot(dx, dy) <= config_.goal_tolerance) {
    result.status = DwaStatus::kGoalReached;
    return result;
  }

  const DynamicWindow window = computeDynamicWindow(state);

  bool found = false;
  double best_cost = std::numeric_limits<double>::infinity();
  double best_v = 0.0;
  double best_w = 0.0;

  for (int vi = 0; vi < config_.v_samples; ++vi) {
    const double v = sampleValue(window.min_v, window.max_v, vi, config_.v_samples);

    for (int wi = 0; wi < config_.w_samples; ++wi) {
      const double w = sampleValue(window.min_w, window.max_w, wi, config_.w_samples);

      const std::vector<TrajectoryPoint> trajectory = simulateTrajectory(state, v, w);

      bool collides = false;
      double max_cell_cost = 0.0;
      for (const TrajectoryPoint & point : trajectory) {
        double point_max_cost = 0.0;
        if (evaluateFootprint(grid, point.x, point.y, point.yaw, &point_max_cost)) {
          collides = true;
          break;
        }
        max_cell_cost = std::max(max_cell_cost, point_max_cost);
      }
      if (collides || trajectory.empty()) {
        continue;
      }

      const double cost = evaluateCost(trajectory.back(), goal, v, max_cell_cost);
      if (cost < best_cost) {
        best_cost = cost;
        best_v = v;
        best_w = w;
        found = true;
      }
    }
  }

  if (!found) {
    result.status = DwaStatus::kNoValidTrajectory;
    return result;
  }

  result.status = DwaStatus::kOk;
  result.v = best_v;
  result.w = best_w;
  return result;
}

}  // namespace amr_planning_core
