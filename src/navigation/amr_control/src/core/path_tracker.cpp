#include "amr_control/core/path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_control
{

bool parseControllerType(const std::string & name, ControllerType & out)
{
  if (name == "pure_pursuit") {
    out = ControllerType::kPurePursuit;
    return true;
  }
  if (name == "stanley") {
    out = ControllerType::kStanley;
    return true;
  }
  return false;
}

std::string controllerTypeName(ControllerType type)
{
  return type == ControllerType::kStanley ? "stanley" : "pure_pursuit";
}

PathTracker::PathTracker(const PathTrackerConfig & config)
: config_(config)
{
}

void PathTracker::setPath(const Path & path)
{
  path_.clear();
  for (const auto & p : path) {
    if (path_.empty() || std::hypot(p.x - path_.back().x, p.y - path_.back().y) > 1e-6) {
      path_.push_back(p);
    }
  }
  cum_lengths_ = cumulativeLengths(path_);
  progress_segment_ = 0;
  first_search_ = true;
  goal_reached_ = false;
}

void PathTracker::clearPath()
{
  setPath(Path{});
}

bool PathTracker::reachedEnd(const Pose2D & pose, const PathProjection & proj) const
{
  const PathPoint & end = path_.back();
  if (std::hypot(end.x - pose.x, end.y - pose.y) <= config_.goal_tolerance) {
    return true;
  }
  if (path_.size() < 2 || proj.segment + 2 < path_.size()) {
    return false;
  }
  // 마지막 선분 위에서 끝점을 지나쳤으면 되돌아가지 않고 선다
  const PathPoint & prev = path_[path_.size() - 2];
  const double along = (pose.x - end.x) * (end.x - prev.x) + (pose.y - end.y) * (end.y - prev.y);
  return along >= 0.0;
}

TrackingResult PathTracker::compute(const Pose2D & pose, double measured_speed, double dt)
{
  TrackingResult result;
  if (path_.empty()) {
    last_v_ = 0.0;
    return result;
  }

  const double max_s = first_search_ ?
    std::numeric_limits<double>::infinity() :
    cum_lengths_[progress_segment_] + config_.search_window;
  first_search_ = false;
  result.projection = projectOntoPath(path_, cum_lengths_, pose.x, pose.y, progress_segment_, max_s);
  progress_segment_ = result.projection.segment;
  result.path_curvature =
    curvatureAt(path_, cum_lengths_, result.projection.s, config_.curvature_half_span);

  if (goal_reached_ || reachedEnd(pose, result.projection)) {
    goal_reached_ = true;
    last_v_ = 0.0;
    result.status = TrackingStatus::kGoalReached;
    result.target = path_.back();
    return result;
  }

  // 경로 방향과 크게 어긋나면 서서 돈다 (출발 직후 반대쪽을 보고 있을 때 등)
  const double heading_error = normalizeAngle(result.projection.yaw - pose.yaw);
  if (config_.rotate_in_place_angle > 0.0 &&
    std::abs(heading_error) > config_.rotate_in_place_angle)
  {
    last_v_ = 0.0;
    result.status = TrackingStatus::kRotating;
    result.w = std::copysign(
      std::min(config_.rotate_in_place_speed, config_.max_angular_speed), heading_error);
    result.target = PathPoint{result.projection.x, result.projection.y};
    return result;
  }

  // 선속도: 순항 → 끝 감속 → 가속 제한
  const double remaining = std::max(cum_lengths_.back() - result.projection.s, 0.0);
  double v = std::min(config_.desired_speed, config_.max_linear_speed);
  v = std::min(
    v, std::max(config_.min_approach_speed, std::sqrt(2.0 * config_.max_linear_decel * remaining)));
  v = std::min(v, last_v_ + config_.max_linear_accel * std::max(dt, 0.0));
  v = std::max(v, 0.0);

  double curvature = 0.0;
  if (config_.controller == ControllerType::kPurePursuit) {
    const double lookahead = purePursuitLookahead(config_.pure_pursuit, measured_speed);
    result.target = pointAtArcLength(path_, cum_lengths_, result.projection.s + lookahead);
    curvature = purePursuitCurvature(pose, result.target);
  } else {
    const Pose2D front = stanleyReferencePoint(pose, config_.stanley.front_offset);
    const PathProjection front_proj = projectOntoPath(
      path_, cum_lengths_, front.x, front.y, progress_segment_,
      cum_lengths_[progress_segment_] + config_.search_window);
    result.target = PathPoint{front_proj.x, front_proj.y};
    const double steer = stanleySteeringAngle(
      config_.stanley, front_proj.yaw - pose.yaw, front_proj.cte, measured_speed);
    curvature = stanleyCurvature(config_.stanley, steer);
  }

  // 각속도 한계에 걸리면 곡률을 지키도록 선속도를 낮춘다 (경로 모양 유지)
  double w = v * curvature;
  if (std::abs(w) > config_.max_angular_speed) {
    v = config_.max_angular_speed / std::abs(curvature);
    w = std::copysign(config_.max_angular_speed, w);
  }

  last_v_ = v;
  result.status = TrackingStatus::kTracking;
  result.v = v;
  result.w = w;
  return result;
}

}  // namespace amr_control
