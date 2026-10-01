#ifndef AMR_PLANNING_CORE__DWA_HPP_
#define AMR_PLANNING_CORE__DWA_HPP_

#include <vector>

#include "amr_planning_core/grid.hpp"

namespace amr_planning_core
{

// 로봇의 순간 상태. x, y, yaw는 Grid의 로컬 좌표계(미터) 기준이다 — Grid는 원점을
// 갖지 않으므로(grid.hpp 참고) 이 좌표계는 "Grid 셀 (0,0)의 모서리가 (0,0)인 평면"이다.
// odom/map 프레임에서 이 좌표계로의 변환은 2~3주차 amr_nav2_plugins가 맡는다
// (A*의 Cell이 이미 grid-index 프레임인 것과 같은 패턴).
struct RobotState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double w = 0.0;
};

// 목표 지점(전역 경로의 다음 지점). RobotState와 같은 좌표계.
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

// 시뮬레이션된 궤적 위의 한 점.
struct TrajectoryPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 1단계: 동적 윈도우(현재 속도 ± 가속 한계*dt를 로봇 한계로 자른 범위).
struct DynamicWindow
{
  double min_v = 0.0;
  double max_v = 0.0;
  double min_w = 0.0;
  double max_w = 0.0;
};

enum class DwaStatus
{
  kOk,
  kGoalReached,
  kNoValidTrajectory,
};

struct DwaCommand
{
  DwaStatus status = DwaStatus::kNoValidTrajectory;
  double v = 0.0;
  double w = 0.0;
};

// DWA 튜닝 파라미터. ROS 비의존이므로 이 struct를 생성자 인자로 받는다.
// 기본값은 amr_bringup/config/nav2_params.yaml의 DWB baseline과 로봇 스펙에
// 맞췄다(값이 다른 항목은 각 필드 주석에 이유를 적었다). 실제 값은 노드 패키지
// (amr_nav2_plugins)가 YAML에서 읽어 채운다.
struct DwaConfig
{
  // 로봇 속도 한계 (nav2_params.yaml FollowPath.max_vel_x/max_vel_theta와 동일).
  double max_v = 2.0;
  double min_v = 0.0;
  double max_w = 1.5;

  // 가속 한계 (nav2_params.yaml FollowPath.acc_lim_x/acc_lim_theta와 동일).
  double acc_v = 1.0;
  double acc_w = 3.2;

  // 궤적 시뮬레이션 간격·길이.
  double dt = 0.1;
  double sim_time = 2.0;

  // 속도 샘플 개수. w_samples는 nav2_params.yaml의 vtheta_samples(20)보다 많다 —
  // 회전 각속도 범위가 좁은 통로 회피 성공률에 더 민감하다고 보고 촘촘히 샘플링한다.
  // TODO(3주차): 실측 KPI를 보고 nav2_params.yaml 값과 맞출지 재검토.
  int v_samples = 20;
  int w_samples = 30;

  // 로봇 풋프린트 반폭(사각형, 600x400mm). 외접원이 아니라 사각형으로 충돌 검사해야
  // 0.6m 좁은 통로(로봇 폭 0.4m + 0.2m)를 지나갈 수 있다 — 외접원 지름은 0.72m라
  // 0.6m 통로를 항상 막힌 것으로 오판한다.
  double footprint_half_length = 0.3;  // x: 전후
  double footprint_half_width = 0.2;   // y: 좌우

  // 이 값 이상의 셀 비용은 통과 불가로 취급한다. amr_planning_core::AStarConfig와
  // 동일한 nav2 costmap 관례(LETHAL_OBSTACLE=254, INSCRIBED_INFLATED_OBSTACLE=253).
  uint8_t lethal_cost_threshold = 253;

  // 비용 함수 가중치. 세 항 모두 [0,1]로 정규화해서 더하므로 가중치를 직접
  // 비교할 수 있다. 기본값 0.8/0.1/0.1의 근거:
  // - heading(목표 방향): 충돌 궤적은 이미 하드 필터로 제외되므로, 남은 후보 중에서는
  //   "목표를 향해 가는가"가 주행 품질을 가장 많이 좌우한다 -> 압도적 비중(0.8).
  // - clearance(장애물 거리, Grid 비용값 기반): 충돌은 아니지만 인플레이션 비용이
  //   높은 셀(장애물에 가까움)을 약하게 피하게 하는 보조 안전 마진 -> 낮게(0.1).
  // - velocity(속도): heading·clearance가 비슷한 후보들 사이에서 더 빠른 쪽을
  //   고르게 하는 효율성 타이브레이커 -> 낮게(0.1).
  double heading_weight = 0.8;
  double clearance_weight = 0.1;
  double velocity_weight = 0.1;

  // 목표까지 이 거리(미터) 이내면 kGoalReached. 전역 경로의 다음 지점(웨이포인트)
  // 추종이 목적이라 각도 허용오차는 따로 두지 않는다(최종 도킹 정밀도는 범위 밖).
  double goal_tolerance = 0.15;
};

// ROS 비의존 DWA 로컬 컨트롤러 코어. 2~3주차에 amr_nav2_plugins에서
// nav2_core::Controller로 감쌀 때, 플러그인은 local costmap을 Grid로 감싸고
// odom<->Grid 로컬 좌표 변환만 담당하며 탐색 로직은 이 클래스에 위임한다.
//
// 이번 주 범위: 정적 장애물(Grid)만 피한다. 3주차에 TrackedObstacleArray 기반
// 동적 장애물을 cost 함수에 반영한다(docs/algorithms/navigation.md 4절 참고).
class DwaPlanner
{
public:
  explicit DwaPlanner(DwaConfig config = DwaConfig());

  // 4단계를 모두 실행해 다음 제어 주기에 낼 (v, w)를 고른다.
  DwaCommand computeVelocityCommand(
    const RobotState & state, const Point2D & goal, const Grid & grid) const;

  // 1단계: 동적 윈도우 계산. 단독으로도 테스트·검사할 수 있도록 공개한다.
  DynamicWindow computeDynamicWindow(const RobotState & state) const;

  // 2단계: 차동구동 전진 운동학으로 (v, w) 궤적을 sim_time 동안 시뮬레이션한다.
  std::vector<TrajectoryPoint> simulateTrajectory(
    const RobotState & state, double v, double w) const;

  // 사각 풋프린트가 (x, y, yaw) 자세에서 장애물과 충돌하는지 검사한다.
  bool footprintCollides(const Grid & grid, double x, double y, double yaw) const;

  const DwaConfig & config() const noexcept {return config_;}

private:
  // footprintCollides의 내부 버전. 충돌 여부와 함께(비충돌 시) 샘플링한 셀 중
  // 가장 높은 비용값을 clearance 비용 계산에 쓰도록 돌려준다.
  bool evaluateFootprint(
    const Grid & grid, double x, double y, double yaw, double * max_cell_cost) const;

  // 3단계: 비용 함수(heading + clearance + velocity, 모두 [0,1] 정규화).
  // v가 거의 0인 궤적은 heading을 계산하지 않고 최댓값(1.0)을 준다 - 그렇지 않으면
  // "목표를 바라보며 제자리에 멈추기"가 실제로 전진하는 선택보다 싸 보이는 국소
  // 최소에 빠진다(정지 궤적은 trajectory_end가 시작점과 같아서 heading이 0이 되기
  // 쉽다). dwa.cpp의 evaluateCost 구현 주석 참고.
  double evaluateCost(
    const TrajectoryPoint & trajectory_end, const Point2D & goal, double v,
    double max_cell_cost) const;

  double headingCost(const TrajectoryPoint & trajectory_end, const Point2D & goal) const;

  DwaConfig config_;
};

}  // namespace amr_planning_core

#endif  // AMR_PLANNING_CORE__DWA_HPP_
