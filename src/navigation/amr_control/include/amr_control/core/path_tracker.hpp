#ifndef AMR_CONTROL__CORE__PATH_TRACKER_HPP_
#define AMR_CONTROL__CORE__PATH_TRACKER_HPP_

// 경로 추종기: 경로를 받아 매 제어 주기마다 (v, ω)를 낸다. 조향은 Pure Pursuit 또는 Stanley,
// 선속도 프로파일(가속 제한·끝 감속)·제자리 회전·끝 정지는 두 방식이 같이 쓴다.
// ROS 비의존 코어 (CONTRIBUTING 4-4). 노드(path_tracker_node)가 config yaml로 채운다.

#include <cstddef>
#include <string>
#include <vector>

#include "amr_control/core/path_geometry.hpp"
#include "amr_control/core/pure_pursuit.hpp"
#include "amr_control/core/stanley.hpp"

namespace amr_control
{

enum class ControllerType
{
  kPurePursuit,
  kStanley,
};

// "pure_pursuit" / "stanley" (cte 기록과 하네스 meta.json `controller` 값과 같은 이름)
bool parseControllerType(const std::string & name, ControllerType & out);
std::string controllerTypeName(ControllerType type);

struct PathTrackerConfig
{
  ControllerType controller = ControllerType::kPurePursuit;

  // 순항 선속도 [m/s]. 로봇 한계(max_linear_speed)로 한 번 더 자른다
  double desired_speed = 0.5;
  // 로봇 한계 (nav2_params.yaml FollowPath max_vel_x / max_vel_theta / acc_lim_x와 같은 값)
  double max_linear_speed = 2.0;
  double max_angular_speed = 1.5;
  double max_linear_accel = 1.0;
  // 경로 끝 감속: v ≤ sqrt(2·decel·남은 호 길이)
  double max_linear_decel = 1.0;
  // 끝 감속 중 최저 속도. 0이면 목표 직전에서 기어가다 못 닿는다
  double min_approach_speed = 0.05;
  // 경로 끝점과 이 거리 안이거나 끝점을 지나치면 정지 [m]
  double goal_tolerance = 0.1;

  // 경로 방향과 로봇 방향 차이가 이 값 [rad]보다 크면 서서 돈다. 0 이하면 끔
  double rotate_in_place_angle = 1.0;
  double rotate_in_place_speed = 0.8;

  // 가장 가까운 점을 지난 위치 앞 이 호 길이 [m] 안에서만 찾는다 (projectOntoPath 주석)
  double search_window = 2.0;
  // 구간 분류(직선·곡선)용 곡률을 잴 때 앞뒤 폭 [m] (curvatureAt)
  double curvature_half_span = 0.25;

  PurePursuitConfig pure_pursuit;
  StanleyConfig stanley;
};

enum class TrackingStatus
{
  kNoPath,
  kTracking,
  kRotating,
  kGoalReached,
};

struct TrackingResult
{
  TrackingStatus status = TrackingStatus::kNoPath;
  double v = 0.0;
  double w = 0.0;
  // 로봇 중심의 투영 (CTE 기록용. Stanley도 기준점이 아닌 중심 기준으로 기록해 두 방식을 같은 잣대로 비교한다)
  PathProjection projection;
  // 투영점에서 경로 곡률 [1/m] (구간 분류용)
  double path_curvature = 0.0;
  // 조향에 쓴 점 (Pure Pursuit은 lookahead 점, Stanley는 기준점의 투영점). 시각화·디버그용
  PathPoint target;
};

class PathTracker
{
public:
  explicit PathTracker(const PathTrackerConfig & config = PathTrackerConfig());

  const PathTrackerConfig & config() const {return config_;}

  // 새 경로. 진행 상태(지난 구간, 도착)를 초기화한다. 직전 명령 속도는 그대로 둔다
  // (주행 중 재계획마다 가속 제한이 0부터 다시 걸려 멈칫하지 않게). 연속해서 겹친 점은 하나로 줄인다.
  void setPath(const Path & path);
  void clearPath();
  // 로봇이 섰을 때(odom 끊김 등 추종기 밖의 이유) 부른다. 다음 출발을 0부터 가속한다
  void resetSpeed() {last_v_ = 0.0;}
  bool hasPath() const {return !path_.empty();}
  const Path & path() const {return path_;}

  // 한 제어 주기. measured_speed는 odom 선속도 (adaptive lookahead·Stanley 분모에 쓴다),
  // dt는 직전 호출과의 간격 [s] (가속 제한에 쓴다).
  // 끝에 닿으면 kGoalReached를 내고 새 경로가 올 때까지 (0, 0)을 유지한다.
  TrackingResult compute(const Pose2D & pose, double measured_speed, double dt);

private:
  bool reachedEnd(const Pose2D & pose, const PathProjection & proj) const;

  PathTrackerConfig config_;
  Path path_;
  std::vector<double> cum_lengths_;
  std::size_t progress_segment_ = 0;
  bool first_search_ = true;
  bool goal_reached_ = false;
  double last_v_ = 0.0;
};

}  // namespace amr_control

#endif  // AMR_CONTROL__CORE__PATH_TRACKER_HPP_
