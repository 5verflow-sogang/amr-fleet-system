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
// amr_nav2_plugins::AStarGlobalPlanner(nav2_core::GlobalPlanner)가 costmap을 Grid로 감싸고
// world<->map 좌표 변환만 담당하며, 탐색 로직은 이 클래스에 그대로 위임한다.
//
// 셀마다의 탐색 상태(g·부모·닫힘)는 호출 사이에 재사용하는 작업 버퍼에 둔다 - 창고 지도(0.025 m,
// 약 420만 셀)에서 매번 새로 잡고 채우면 짧은 경로도 그것만으로 30 ms가 넘었다. 그래서 plan()은
// const지만 같은 객체로 여러 스레드에서 동시에 부르면 안 된다(planner_server는 한 번에 하나씩 부른다).
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

  // 작업 버퍼 (셀 인덱스 = y * width + x). seen_[i] == generation_이면 g_·parent_[i]가 이번 탐색 값이고,
  // closed_[i] == generation_이면 닫힌 셀이다. 탐색마다 generation_만 올려 전체를 지우지 않는다.
  mutable std::vector<double> g_;
  mutable std::vector<uint32_t> parent_;
  mutable std::vector<uint32_t> seen_;
  mutable std::vector<uint32_t> closed_;
  mutable uint32_t generation_ = 0;
};

}  // namespace amr_planning_core

#endif  // AMR_PLANNING_CORE__ASTAR_HPP_
