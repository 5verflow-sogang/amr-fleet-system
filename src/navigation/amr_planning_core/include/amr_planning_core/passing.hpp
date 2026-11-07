#ifndef AMR_PLANNING_CORE__PASSING_HPP_
#define AMR_PLANNING_CORE__PASSING_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "amr_planning_core/dwa.hpp"
#include "amr_planning_core/grid.hpp"

namespace amr_planning_core
{

// 정면으로 마주 오는 동적 장애물을 옆으로 비켜 지나간다 (원 명세 7장 "1.0 m/s 장애물 재계획 회피", "복귀 5초").
//
// 전역 경로는 그대로 두고, 경로를 따라가는 좌표(Frenet: 경로 길이 s, 옆 거리 d, 왼쪽 +)에서 옆 이탈 d(s)를
// 나가기(S자) → 유지 → 들어오기(S자) 모양으로 정해 그 모양대로 옮긴 경로를 DWA가 따라가게 한다.
// Werling 외 "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame"
// (ICRA 2010)의 옆 이탈 계획을 규칙 하나로 줄인 것이다. 들어오는 거리(ramp_in)가 정해져 있어 복귀 시간이
// 설계값이 된다.
// - 경로를 1초마다 새로 짜면 비킨 자리에서 목표까지 대각선이 돼 처음 경로로 늦게 돌아온다 → 처음 경로를 쓴다
// - 장애물이 차체 뒤로 지나간 뒤에만 들어온다 (지나가는 중에 머리를 틀면 TTC가 다시 짧아져 안전 노드가 세운다)
// - 정면이 아닌 장애물(가로지르기)에는 켜지지 않는다. 그 경우는 기존 회피 모드·안전 노드가 맡는다
struct PassingConfig
{
  bool enabled = false;
  // 마주 온다: 장애물 속도와 그 자리 경로 방향 사이 cos이 이 값 이하 (-0.7 ≈ 135°), 속력 min_obstacle_speed 이상
  double max_heading_cos = -0.7;
  double min_obstacle_speed = 0.2;
  // 지나칠 때 차체 옆면 ↔ 장애물 표면 [m]. DWA 동적 장애물 여유(dynamic_obstacle_safety_margin)보다 커야
  // DWA가 옮긴 경로를 따라가는 궤적을 받는다
  double clearance = 0.3;
  // 옆 이탈 상한 [m] (원 명세 경로 이탈 1 m). 필요한 이탈이 이보다 크면 비키지 않는다
  double max_offset = 0.9;
  // 경로를 따라 나가는·들어오는 거리 [m]
  double ramp_out = 0.6;
  double ramp_in = 0.6;
  // 다 비켜선 뒤 장애물과 만나기까지 여유 [s]. 크면 일찍 비켜 이탈 시간이 길어지고, 작으면 비키는 중에
  // TTC가 안전 노드 정지(1 s) 아래로 떨어진다
  double start_margin = 0.8;
  // 이보다 먼(경로 길이) 장애물은 보지 않는다 [m]
  double max_lookahead = 8.0;
  // 장애물 표면이 차체 뒤 끝에서 이만큼 더 지나가면 들어온다 [m]
  double pass_margin = 0.2;
  // 막는 장애물이 이 주기 수 연속으로 안 보여야 들어온다 (추적이 한 주기 끊긴 것을 지나간 것으로 보지 않게.
  // 20 Hz 제어 주기면 6 = 0.3 s)
  int clear_cycles = 6;
  // 막는 장애물 트랙이 끊겨도(사람 다리 군집이 갈라져 ID가 바뀌면 1~1.5 s 비었다, 정면 접근 시험 11/4) 마지막
  // 위치·속도로 이 시간 [s]까지 예측해 이어 본다 - 비키기 시작과 들어오기 판단 모두
  double threat_memory = 1.5;
  // 장애물이 경로 가운데(|d| < center_band)면 이쪽으로 비킨다: -1 오른쪽, +1 왼쪽 (진행 방향 기준).
  // 로봇끼리 마주쳐도 같은 쪽으로 비켜 엇갈린다
  int preferred_side = -1;
  double center_band = 0.05;
  // 나가기 시간 계산에 쓰는 로봇 속력 하한 [m/s] (서 있어도 0으로 나누지 않게)
  double min_robot_speed = 0.3;
  // 옮긴 경로의 점이 정적 장애물(lethal_cost_threshold 이상 셀)에서 차체 반폭 + free_margin 떨어져야 비킨다.
  // Grid 밖 점은 못 보므로 비어 있다고 본다 (지역 costmap은 로봇 둘레만 있다. 가 보면 DWA가 다시 검사한다)
  double free_margin = 0.05;
  double footprint_half_length = 0.3;
  double footprint_half_width = 0.2;
  uint8_t lethal_cost_threshold = 253;
};

enum class PassingPhase { kIdle, kOut, kHold, kReturn };

class PassingManeuver
{
public:
  explicit PassingManeuver(const PassingConfig & config = PassingConfig());

  // 제어 주기마다 부른다. path: 따라갈 전역 경로, obstacles: 동적 장애물 (둘 다 Grid 로컬 좌표).
  // 비켜 지나가는 중이면 옆으로 옮긴 경로를, 아니면 path 그대로를 돌려준다.
  // now: 지금 시각 [s] (트랙이 끊긴 동안의 예측에 쓴다).
  std::vector<Point2D> update(
    const RobotState & robot, const std::vector<Point2D> & path, const Grid & grid,
    const std::vector<DynamicObstacle> & obstacles, double now);

  PassingPhase phase() const noexcept {return phase_;}
  bool active() const noexcept {return phase_ != PassingPhase::kIdle;}
  // 마주 오는 장애물이 보이고 비킬 자리가 있어 곧 비킬 예정이거나 비키는 중 (이동안 DWA 회피 모드를 끈다 -
  // 회피 모드의 넓은 여유로 먼저 늦추면 나가기 시점이 어긋난다)
  bool engaged() const noexcept {return active() || armed_;}
  // 계획한 옆 이탈 [m] (경로 왼쪽 +). 쉴 때 0
  double offset() const noexcept {return active() ? offset_ : 0.0;}
  const PassingConfig & config() const noexcept {return config_;}
  void reset() noexcept;

  // 점 p를 경로에 투영한 Frenet 좌표. s: 경로 처음부터 길이, d: 옆 거리(왼쪽 +), (tx, ty): 그 자리 경로 방향.
  // 경로 시작점보다 뒤는 첫 구간을 늘려 s < 0으로 준다 (끝점 너머는 끝점에 붙는다)
  struct Frenet
  {
    double s = 0.0;
    double d = 0.0;
    double tx = 1.0;
    double ty = 0.0;
  };
  static std::vector<double> arcLengths(const std::vector<Point2D> & path);
  static std::optional<Frenet> project(
    const std::vector<Point2D> & path, const std::vector<double> & s, const Point2D & p);

  // 계획한 옆 이탈 d(s) (테스트·분석용)
  double lateral(double s) const noexcept;

private:
  struct Threat
  {
    double s = 0.0;
    double d = 0.0;
    double radius = 0.0;
    double closing_speed = 0.0;   // 경로를 따라 로봇 쪽으로 오는 속력 [m/s]
  };
  std::optional<Threat> nearestThreat(
    const std::vector<Point2D> & path, const std::vector<double> & s, double s_robot,
    const std::vector<DynamicObstacle> & obstacles, double band) const;
  // 지금 보이는 막는 장애물, 없으면 threat_memory 안의 마지막 것을 등속으로 옮긴 예측 (지나갔으면 없음)
  std::optional<Threat> threat(
    const std::vector<Point2D> & path, const std::vector<double> & s, double s_robot,
    const std::vector<DynamicObstacle> & obstacles, double band, double now);
  bool pathFree(
    const std::vector<Point2D> & path, const std::vector<double> & s, const Grid & grid,
    double s_from, double s_to) const;
  std::vector<Point2D> shifted(
    const std::vector<Point2D> & path, const std::vector<double> & s) const;
  bool samePath(const std::vector<Point2D> & path) const noexcept;
  // 비키기 상태만 처음으로 (트랙 기억·예정은 둔다)
  void resetManeuver() noexcept;

  PassingConfig config_;
  PassingPhase phase_ = PassingPhase::kIdle;
  double offset_ = 0.0;
  double s_start_ = 0.0;    // 나가기 시작 s
  double s_return_ = 0.0;   // 들어오기 시작 s (유지 중에는 무한)
  int clear_count_ = 0;     // 막는 장애물이 연속으로 안 보인 주기 수
  bool armed_ = false;      // 곧 비킬 예정 (아직 이름)
  std::optional<Threat> memory_;   // 마지막으로 본 막는 장애물
  double memory_t_ = 0.0;
  // 경로가 바뀌면(새 전역 경로) 처음부터 다시 본다
  std::size_t path_size_ = 0;
  Point2D path_front_;
  Point2D path_back_;
};

}  // namespace amr_planning_core

#endif  // AMR_PLANNING_CORE__PASSING_HPP_
