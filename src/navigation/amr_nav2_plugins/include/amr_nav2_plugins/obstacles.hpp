#ifndef AMR_NAV2_PLUGINS__OBSTACLES_HPP_
#define AMR_NAV2_PLUGINS__OBSTACLES_HPP_

#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_planning_core/dwa.hpp"

namespace amr_nav2_plugins
{

// perception/obstacles 트랙 중 움직이는 것(is_dynamic)만 DWA 동적 장애물로 바꾼다.
// 정적 트랙(랙 모서리·기둥·짧은 벽 조각)은 이미 costmap(정적 지도 + 장애물 층)에 있다. 그걸
// 반경 + dynamic_obstacle_safety_margin으로 한 번 더 막으면, 랙 옆을 지날 때 랙 면 트랙이 차체와
// 겹쳐(Gazebo 3단계: 반경 0.83 m 정적 트랙이 차체와 -0.56 m) 빠져나오는 궤적까지 모두 버려진다.
// 안전 노드(amr_perception_core safety.py)도 같은 규칙으로 정적 트랙을 건너뛴다.
// 좌표는 메시지 그대로(costmap 전역 좌표계 odom) 둔다 - origin 보정은 호출하는 쪽에서 매 주기 한다.
inline std::vector<amr_planning_core::DynamicObstacle> toDynamicObstacles(
  const amr_msgs::msg::TrackedObstacleArray & msg)
{
  std::vector<amr_planning_core::DynamicObstacle> obstacles;
  obstacles.reserve(msg.obstacles.size());
  for (const amr_msgs::msg::TrackedObstacle & tracked : msg.obstacles) {
    if (!tracked.is_dynamic) {
      continue;
    }
    amr_planning_core::DynamicObstacle obstacle;
    obstacle.x = tracked.pose.pose.position.x;
    obstacle.y = tracked.pose.pose.position.y;
    obstacle.vx = tracked.twist.twist.linear.x;
    obstacle.vy = tracked.twist.twist.linear.y;
    obstacle.radius = tracked.radius;
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__OBSTACLES_HPP_
