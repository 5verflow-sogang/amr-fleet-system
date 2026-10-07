#ifndef AMR_CONTROL__CORE__PATH_GEOMETRY_HPP_
#define AMR_CONTROL__CORE__PATH_GEOMETRY_HPP_

// 경로 추종 공통 기하: 가장 가까운 점, 부호 있는 CTE, 호 길이 위치, 곡률.
// ROS 비의존 코어다 (CONTRIBUTING 4-4). rclcpp·메시지 타입을 include하지 않는다.
// 좌표는 모두 같은 평면 프레임(노드가 경로 프레임으로 맞춰 넘긴다), 단위는 m·rad.

#include <cstddef>
#include <limits>
#include <vector>

namespace amr_control
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
};

using Path = std::vector<PathPoint>;

// 경로 위 가장 가까운 점. 경로는 점을 잇는 선분(폴리라인)으로 본다.
struct PathProjection
{
  bool valid = false;
  // 가장 가까운 점이 놓인 선분 [segment, segment + 1]과 그 안의 비율 t (0~1)
  std::size_t segment = 0;
  double t = 0.0;
  // 가장 가까운 점 (= cte.csv의 planned_x, planned_y)
  double x = 0.0;
  double y = 0.0;
  // 경로 시작에서 가장 가까운 점까지의 호 길이
  double s = 0.0;
  // 그 선분의 진행 방향
  double yaw = 0.0;
  // 부호 있는 횡방향 오차. 경로 진행 방향 기준 왼쪽이 +, 오른쪽이 −
  double cte = 0.0;
};

// -π~π로 정규화
double normalizeAngle(double a);

// 각 점까지의 누적 호 길이. 결과[0] = 0, 결과.back() = 전체 길이
std::vector<double> cumulativeLengths(const Path & path);

// (x, y)에서 가장 가까운 경로 위 점. 선분 first_segment부터, 시작 호 길이가 max_s 이하인
// 선분까지만 찾는다. 추종 중에는 지난 위치 앞쪽 일정 거리만 보게 해서, 되돌아오거나 교차하는
// 경로에서 이미 지난 구간이나 한참 뒤 구간으로 튀지 않게 한다.
// 점이 1개면 그 점(cte는 거리, 부호 +), 0개면 valid = false.
PathProjection projectOntoPath(
  const Path & path, const std::vector<double> & cum_lengths, double x, double y,
  std::size_t first_segment = 0,
  double max_s = std::numeric_limits<double>::infinity());

// 호 길이 s 위치의 점. 0~전체 길이로 자른다
PathPoint pointAtArcLength(const Path & path, const std::vector<double> & cum_lengths, double s);

// 호 길이 s 앞뒤 half_span 위치의 세 점을 지나는 원의 곡률 [1/m] (Menger 곡률, 부호 없음).
// 경로 점 간격이 촘촘하면(Nav2 5 cm) 이웃 점 셋으로는 잡음이 커서 호 길이 폭으로 잡는다.
// 경로 양 끝에서는 폭을 경로 안으로 자른다. 세 점이 겹치면 0.
double curvatureAt(
  const Path & path, const std::vector<double> & cum_lengths, double s, double half_span);

}  // namespace amr_control

#endif  // AMR_CONTROL__CORE__PATH_GEOMETRY_HPP_
