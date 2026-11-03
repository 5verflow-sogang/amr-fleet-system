#ifndef AMR_PLANNING_CORE__PATH_SMOOTHING_HPP_
#define AMR_PLANNING_CORE__PATH_SMOOTHING_HPP_

#include <cstdint>
#include <vector>

#include "amr_planning_core/grid.hpp"

namespace amr_planning_core
{

// A* 경로(셀 목록) 평활화 파라미터 (docs/algorithms/navigation.md 3-5절).
struct SmoothingConfig
{
  // 1단계 지름길: 앞 점에서 직선으로 보이는 가장 먼 점까지 바로 잇는다. 8방향 격자 경로의
  // 지그재그(0°/45° 계단)를 없애 길이를 줄인다.
  bool shortcut = true;

  // 2단계 경사 하강(Thrun): 꺾인 점을 둥글게 편다. weight_data는 원래(지름길) 점으로 끌어당기는
  // 힘, weight_smooth는 이웃 두 점의 가운데로 끌어당기는 힘이다. weight_data가 0이면 직선으로
  // 수렴하고, 클수록 원래 점에 가깝게 남는다.
  // 꺾인 곳이 둥글어지는 길이는 대략 point_spacing * sqrt(weight_smooth / weight_data)다.
  bool gradient = true;
  double weight_data = 0.1;
  double weight_smooth = 0.3;
  // 경사 하강을 할 점 간격(셀). 지름길 선분을 이 간격으로 나눈 점을 편다. 1셀이면 둥글어지는 길이도 1~2셀이라
  // 거의 안 펴진다 - 플러그인은 미터 값(smoothing_point_spacing_m)을 셀로 바꿔 넘긴다
  double point_spacing = 4.0;
  // 한 번 훑을 때 모든 점의 이동량 합(셀)이 tolerance 아래로 내려가거나 max_iterations에 닿으면 멈춘다
  int max_iterations = 200;
  double tolerance = 1e-3;

  // 이 값 이상의 셀은 지나갈 수 없다 (AStarConfig::lethal_cost_threshold와 같은 값으로 둔다).
  uint8_t lethal_cost_threshold = 253;
};

// A* 경로를 평활화한다. 입력·출력 좌표는 셀 단위이고 셀 (x, y)의 중심이 (x, y)다.
//
// 안전 규칙: 평활화는 경로를 원래 A* 경로보다 비용이 큰 셀로 옮기지 않는다.
// - 지름길 i→j는 그 선분이 지나는 모든 셀의 비용이 원래 경로 i..j 구간의 최대 비용 이하일 때만 쓴다.
// - 경사 하강은 점을 옮긴 자리의 셀 비용이 지름길 경로에서 그 점이 있던 셀의 비용 이하일 때만 옮긴다.
// 비용은 inflation(장애물까지 거리의 감소 함수)이라, 경로 위 최대 비용이 늘지 않으면 장애물 여유 거리의
// 최솟값도 (격자 해상도 안에서) 줄지 않는다.
//
// 출력 점 간격은 1셀 이하다(편 뒤 1셀 간격으로 다시 나눈다 - 컨트롤러가 촘촘한 경로를 기대한다). 처음·끝 점은 그대로다.
// 점이 2개 이하면 그대로 돌려준다.
std::vector<Point2D> smoothPath(
  const Grid & grid, const std::vector<Cell> & path, const SmoothingConfig & config = SmoothingConfig());

// 선분 a→b가 지나는 셀들의 최대 비용. 범위 밖 셀은 255로 본다. a·b는 셀 단위(셀 중심이 정수).
uint8_t maxCostAlongSegment(const Grid & grid, const Point2D & a, const Point2D & b);

}  // namespace amr_planning_core

#endif  // AMR_PLANNING_CORE__PATH_SMOOTHING_HPP_
