// 발행 속도 줄이기 규칙 (amr_gazebo/clock_throttle.py should_publish와 같다, ROS 없이 테스트한다)
#pragma once

#include <cstdint>
#include <optional>

namespace amr_gazebo
{

// 마지막으로 보낸 시각에서 period 이상 흘렀으면 보낸다. 처음이거나 시간이 거꾸로 가면(리셋) 바로 보낸다
inline bool should_publish(std::optional<int64_t> last_sent_ns, int64_t now_ns, int64_t period_ns)
{
  if (!last_sent_ns || now_ns < *last_sent_ns) {
    return true;
  }
  return now_ns - *last_sent_ns >= period_ns;
}

}  // namespace amr_gazebo
