// joint_state_throttle 규칙 (ROS 없이)
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "amr_gazebo/throttle.hpp"

using amr_gazebo::should_publish;
constexpr int64_t MS = 1'000'000;

TEST(Throttle, FirstTickIsSent)
{
  EXPECT_TRUE(should_publish(std::nullopt, 0, 20 * MS));
}

TEST(Throttle, Input1kHzBecomes50Hz)
{
  int sent = 0;
  std::optional<int64_t> last;
  for (int64_t t = 0; t < 1000 * MS; t += MS) {  // 1초 동안 1 ms 간격
    if (should_publish(last, t, 20 * MS)) {
      ++sent;
      last = t;
    }
  }
  EXPECT_EQ(sent, 50);
}

TEST(Throttle, TimeGoingBackIsSentAtOnce)
{
  EXPECT_TRUE(should_publish(10'000 * MS, 0, 20 * MS));
}
