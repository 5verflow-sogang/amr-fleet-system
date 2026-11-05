#ifndef AMR_NAV2_PLUGINS__SPEED_LIMIT_HPP_
#define AMR_NAV2_PLUGINS__SPEED_LIMIT_HPP_

#include <algorithm>

namespace amr_nav2_plugins
{

// Nav2 속도 필터(costmap_filters) 규약: speed_limit이 0 이하면 제한 없음
// (Nav2 Humble NO_SPEED_LIMIT = 0.0, nav2_costmap_2d/costmap_filters/filter_values.hpp) -
// 음수는 우리 쪽 "아직 안 받음" 기본값(DwaController::speed_limit_)도 같이 포함한다.
// percentage면 max_v에 대한 비율(%), 아니면 절대 속도[m/s]다. ROS 없이 테스트한다(#294).
inline double applySpeedLimit(double v, double max_v, double speed_limit, bool percentage)
{
  if (speed_limit <= 0.0) {
    return v;
  }
  const double limit = percentage ? max_v * speed_limit / 100.0 : speed_limit;
  return std::min(v, limit);
}

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__SPEED_LIMIT_HPP_
