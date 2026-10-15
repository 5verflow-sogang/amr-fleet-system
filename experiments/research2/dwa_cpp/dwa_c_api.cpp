// amr_planning_core DwaPlanner(#41, 수정 없음)를 Python(ctypes)에서 부르기 위한 C 함수.
// payload_nav/dwa_cpp.py가 dwa.cpp·grid.cpp와 함께 공유 라이브러리로 컴파일한다.
#include <cstdint>
#include <vector>

#include "amr_planning_core/dwa.hpp"
#include "amr_planning_core/grid.hpp"

using amr_planning_core::DwaCommand;
using amr_planning_core::DwaConfig;
using amr_planning_core::DwaPlanner;
using amr_planning_core::DwaStatus;
using amr_planning_core::DynamicObstacle;
using amr_planning_core::Grid;
using amr_planning_core::Point2D;
using amr_planning_core::RobotState;

extern "C" {

// DwaConfig 필드 (dwa.hpp 순서). Python 쪽 _Config와 같아야 한다.
struct DwaCConfig
{
  double max_v, min_v, max_w, acc_v, acc_w, dt, sim_time;
  int v_samples, w_samples;
  double footprint_half_length, footprint_half_width;
  int lethal_cost_threshold;
  double heading_weight, clearance_weight, velocity_weight, goal_tolerance;
  int predict_dynamic_obstacles;
  double dynamic_obstacle_safety_margin, dynamic_obstacle_influence_radius, dynamic_weight;
};

static DwaConfig toConfig(const DwaCConfig * c)
{
  DwaConfig cfg;
  cfg.max_v = c->max_v;
  cfg.min_v = c->min_v;
  cfg.max_w = c->max_w;
  cfg.acc_v = c->acc_v;
  cfg.acc_w = c->acc_w;
  cfg.dt = c->dt;
  cfg.sim_time = c->sim_time;
  cfg.v_samples = c->v_samples;
  cfg.w_samples = c->w_samples;
  cfg.footprint_half_length = c->footprint_half_length;
  cfg.footprint_half_width = c->footprint_half_width;
  cfg.lethal_cost_threshold = static_cast<uint8_t>(c->lethal_cost_threshold);
  cfg.heading_weight = c->heading_weight;
  cfg.clearance_weight = c->clearance_weight;
  cfg.velocity_weight = c->velocity_weight;
  cfg.goal_tolerance = c->goal_tolerance;
  cfg.predict_dynamic_obstacles = c->predict_dynamic_obstacles != 0;
  cfg.dynamic_obstacle_safety_margin = c->dynamic_obstacle_safety_margin;
  cfg.dynamic_obstacle_influence_radius = c->dynamic_obstacle_influence_radius;
  cfg.dynamic_weight = c->dynamic_weight;
  return cfg;
}

static int statusCode(DwaStatus s)
{
  return s == DwaStatus::kOk ? 0 : (s == DwaStatus::kGoalReached ? 1 : 2);
}

// state[5] = x, y, yaw, v, w (grid 로컬 좌표), goal[2], obstacles[n*5] = x, y, vx, vy, radius.
// 반환 상태 0 ok / 1 goal_reached / 2 no_valid, out[2] = v, w.
int dwa_compute_velocity_command(
  const DwaCConfig * config, const double * state, const double * goal,
  int width, int height, double resolution, const uint8_t * costs,
  const double * obstacles, int n_obstacles, double * out)
{
  const DwaPlanner planner(toConfig(config));
  RobotState s;
  s.x = state[0];
  s.y = state[1];
  s.yaw = state[2];
  s.v = state[3];
  s.w = state[4];
  Point2D g;
  g.x = goal[0];
  g.y = goal[1];
  std::vector<DynamicObstacle> obs(static_cast<std::size_t>(n_obstacles));
  for (int i = 0; i < n_obstacles; ++i) {
    obs[i].x = obstacles[5 * i];
    obs[i].y = obstacles[5 * i + 1];
    obs[i].vx = obstacles[5 * i + 2];
    obs[i].vy = obstacles[5 * i + 3];
    obs[i].radius = obstacles[5 * i + 4];
  }
  const Grid grid(static_cast<std::size_t>(width), static_cast<std::size_t>(height), resolution, costs);
  const DwaCommand cmd = planner.computeVelocityCommand(s, g, grid, obs);
  out[0] = cmd.v;
  out[1] = cmd.w;
  return statusCode(cmd.status);
}

// DwaPlanner::rotateToGoalYaw. state[5]는 위와 같다. out[2] = v(0), w.
int dwa_rotate_to_goal_yaw(
  const DwaCConfig * config, const double * state, double goal_yaw, double yaw_tolerance, double * out)
{
  const DwaPlanner planner(toConfig(config));
  RobotState s;
  s.x = state[0];
  s.y = state[1];
  s.yaw = state[2];
  s.v = state[3];
  s.w = state[4];
  const DwaCommand cmd = planner.rotateToGoalYaw(s, goal_yaw, yaw_tolerance);
  out[0] = cmd.v;
  out[1] = cmd.w;
  return statusCode(cmd.status);
}

}  // extern "C"
