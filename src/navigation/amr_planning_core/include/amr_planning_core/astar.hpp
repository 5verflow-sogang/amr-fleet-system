#ifndef AMR_PLANNING_CORE__ASTAR_HPP_
#define AMR_PLANNING_CORE__ASTAR_HPP_

#include <cstdint>
#include <vector>

#include "amr_planning_core/grid.hpp"

namespace amr_planning_core
{

enum class HeuristicType
{
  kEuclidean,
  kOctile,
};

enum class PlanStatus
{
  kSuccess,
  kStartOutOfBounds,
  kGoalOutOfBounds,
  kStartOccupied,
  kGoalOccupied,
  kNoPathFound,
};

// A* 튜닝 파라미터. ROS 비의존이므로 이 struct를 생성자 인자로 받는다.
// 실제 값은 노드 패키지(amr_nav2_plugins)가 YAML에서 읽어 채운다.
struct AStarConfig
{
  // 8방향 이동에는 옥타일이 유클리드보다 더 팽팽한 추정을 준다(둘 다 admissible).
  HeuristicType heuristic = HeuristicType::kOctile;

  // 이 값 이상의 셀 비용은 통과 불가로 취급한다.
  // nav2 costmap 관례: LETHAL_OBSTACLE=254, INSCRIBED_INFLATED_OBSTACLE=253.
  uint8_t lethal_cost_threshold = 253;

  // 셀 비용을 이동 비용에 반영하는 가중치. 0이면 순수 거리 기반(고전 A*)이고,
  // 커질수록 비용이 높은 셀(인플레이션 영역 등)을 더 피해서 돈다.
  double cost_weight = 0.0;

  // true면 대각선 이동 시 양쪽 축 셀이 모두 통과 가능해야 이동을 허용한다
  // (장애물 모서리를 뚫고 지나가는 대각선 이동 방지).
  bool prevent_corner_cutting = true;
};

struct PlanResult
{
  PlanStatus status = PlanStatus::kNoPathFound;
  std::vector<Cell> path;
};

// ROS 비의존 A* 글로벌 플래너 코어.
// 2주차에 amr_nav2_plugins에서 nav2_core::GlobalPlanner로 감쌀 때,
// 플러그인은 costmap을 Grid로 감싸고 world<->map 좌표 변환만 담당하며
// 탐색 로직은 이 클래스에 그대로 위임한다.
class AStarPlanner
{
public:
  explicit AStarPlanner(AStarConfig config = AStarConfig());

  PlanResult plan(const Grid & grid, const Cell & start, const Cell & goal) const;

  const AStarConfig & config() const noexcept {return config_;}

private:
  double heuristic(const Cell & a, const Cell & b) const;
  double moveCost(double base_distance, uint8_t neighbor_cost) const;
  bool isTraversable(const Grid & grid, const Cell & cell) const;

  AStarConfig config_;
};

}  // namespace amr_planning_core

#endif  // AMR_PLANNING_CORE__ASTAR_HPP_
