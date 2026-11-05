#include <gtest/gtest.h>

#include "amr_nav2_plugins/speed_limit.hpp"

namespace amr_nav2_plugins
{
namespace
{

constexpr double kMaxV = 1.0;
constexpr double kV = 0.8;  // DWA가 고른 속도 - 제한보다 항상 큰 값으로 둬서 "걸리는지"만 본다

}  // namespace

// Nav2 Humble NO_SPEED_LIMIT = 0.0 - 제한 없음으로 봐야 한다(#294)
TEST(SpeedLimitTest, ZeroMeansNoLimit)
{
  EXPECT_DOUBLE_EQ(applySpeedLimit(kV, kMaxV, 0.0, false), kV);
  EXPECT_DOUBLE_EQ(applySpeedLimit(kV, kMaxV, 0.0, true), kV);
}

// 아직 setSpeedLimit을 못 받은 기본값(음수)도 제한 없음
TEST(SpeedLimitTest, NegativeMeansNoLimit)
{
  EXPECT_DOUBLE_EQ(applySpeedLimit(kV, kMaxV, -1.0, false), kV);
}

TEST(SpeedLimitTest, PercentageLimitsToFractionOfMaxV)
{
  EXPECT_DOUBLE_EQ(applySpeedLimit(kV, kMaxV, 50.0, true), kMaxV * 0.5);
}

TEST(SpeedLimitTest, AbsoluteLimitsToThatSpeed)
{
  EXPECT_DOUBLE_EQ(applySpeedLimit(kV, kMaxV, 0.2, false), 0.2);
}

}  // namespace amr_nav2_plugins
