#include <gtest/gtest.h>

#include <limits>

#include "amr_nav2_plugins/avoid_mode.hpp"

namespace amr_nav2_plugins
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// collision_risks 10 Hz, 제어 20 Hz처럼 t0부터 duration 동안 ttc를 받으며 update를 부른다. 끝 시각을 돌려준다.
double feed(AvoidModeSwitch & avoid, double t0, double duration, double ttc)
{
  double t = t0;
  for (int i = 0; t < t0 + duration - 1e-9; ++i) {
    if (i % 2 == 0) {
      avoid.onRisks(ttc, t);
    }
    avoid.update(t);
    t += 0.05;
  }
  return t;
}

}  // namespace

TEST(AvoidModeTest, EntersBelowThresholdAndExitsAfterHold)
{
  AvoidModeSwitch avoid;
  double t = feed(avoid, 0.0, 1.0, 5.0);
  EXPECT_FALSE(avoid.active());  // TTC 5 s >= 3 s
  t = feed(avoid, t, 0.1, 2.9);
  EXPECT_TRUE(avoid.active());   // TTC < 3 s - 켬
  t = feed(avoid, t, 0.4, kInf);
  EXPECT_TRUE(avoid.active());   // 안전 0.4 s - 아직 0.5 s 안 됨
  t = feed(avoid, t, 0.3, kInf);
  EXPECT_FALSE(avoid.active());  // 0.5 s 넘게 이어짐 - 끔
}

TEST(AvoidModeTest, RiskWithinHoldRestartsHold)
{
  // 0.5 s 안에 다시 위험해지면 안 꺼지고, 안전한 시간을 처음부터 다시 센다
  AvoidModeSwitch avoid;
  double t = feed(avoid, 0.0, 0.1, 1.0);
  ASSERT_TRUE(avoid.active());
  t = feed(avoid, t, 0.3, 3.0);   // 3.0 s는 문턱 이상 - 안전
  t = feed(avoid, t, 0.1, 2.0);   // 다시 위험
  t = feed(avoid, t, 0.4, kInf);  // 위험 뒤 안전 0.4 s - 앞의 0.3 s는 안 친다
  EXPECT_TRUE(avoid.active());
  t = feed(avoid, t, 0.3, kInf);
  EXPECT_FALSE(avoid.active());
}

TEST(AvoidModeTest, KeepsStateWhileRisksAreStale)
{
  AvoidModeSwitch avoid;
  double t = feed(avoid, 0.0, 0.1, 1.0);
  ASSERT_TRUE(avoid.active());
  t = feed(avoid, t, 0.2, kInf);  // 안전 0.2 s
  // collision_risks가 끊김 - 1 s 동안 update만. 끊긴 동안은 끄지 않는다
  for (int i = 0; i < 20; ++i) {
    t += 0.05;
    EXPECT_TRUE(avoid.update(t));
  }
  EXPECT_TRUE(avoid.stale(t));
  // 다시 오면 안전한 0.5 s를 새로 채워야 끈다(끊기기 전 0.2 s는 안 친다)
  t = feed(avoid, t, 0.4, kInf);
  EXPECT_FALSE(avoid.stale(t));
  EXPECT_TRUE(avoid.active());
  t = feed(avoid, t, 0.3, kInf);
  EXPECT_FALSE(avoid.active());

  // 꺼진 채로 끊기면 꺼진 채로 둔다
  for (int i = 0; i < 20; ++i) {
    t += 0.05;
    EXPECT_FALSE(avoid.update(t));
  }
  EXPECT_TRUE(avoid.stale(t));
}

TEST(AvoidModeTest, NeverReceivedIsNotStale)
{
  // 추적 노드를 안 띄운 스택(fleet_stack 등)은 collision_risks가 처음부터 없다 - 끊김 경고를 내지 않고 꺼진 채
  AvoidModeSwitch avoid;
  EXPECT_FALSE(avoid.update(10.0));
  EXPECT_FALSE(avoid.stale(10.0));
}

TEST(AvoidModeTest, DisabledNeverActivates)
{
  AvoidModeParams params;
  params.enabled = false;
  AvoidModeSwitch avoid(params);
  feed(avoid, 0.0, 1.0, 0.5);
  EXPECT_FALSE(avoid.active());
}

}  // namespace amr_nav2_plugins
