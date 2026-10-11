#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "amr_control/core/pid.hpp"

namespace amr_control
{
namespace
{

constexpr double kDt = 0.05;  // 제어 주기 20 Hz

// 1차 속도 플랜트: v' = (gain·u − v) / tau. gain < 1이면 앞먹임만으로는 목표에 못 미친다
// (적재·바닥 마찰로 바퀴 속도 명령만큼 안 나가는 상황)
struct Plant
{
  double gain = 0.8;
  double tau = 0.2;
  double v = 0.0;

  double step(double u)
  {
    v += kDt / tau * (gain * u - v);
    return v;
  }
};

PidConfig defaultConfig()
{
  // config/path_tracker.yaml velocity_pid와 같은 값
  PidConfig config;
  config.kp = 0.5;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_limit = 0.3;
  config.derivative_filter_tau = 0.05;
  config.output_min = 0.0;
  config.output_max = 2.0;
  return config;
}

// 계단 응답: 0 → 0.5 m/s. 앞먹임만이면 0.4 m/s에 머물지만 PID는 3초 안에 ±2%로 정착하고 넘침이 작다
TEST(Pid, StepResponseSettles)
{
  Pid pid(defaultConfig());
  Plant plant;
  const double target = 0.5;
  double settle_time = -1.0;
  double peak = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double t = i * kDt;
    const double v = plant.step(pid.update(target, plant.v, kDt));
    peak = std::max(peak, v);
    if (std::abs(v - target) > 0.02 * target) {
      settle_time = -1.0;
    } else if (settle_time < 0.0) {
      settle_time = t;
    }
  }
  ASSERT_GE(settle_time, 0.0);
  EXPECT_LT(settle_time, 3.0);
  EXPECT_LT(peak, target * 1.1);
  EXPECT_NEAR(plant.v, target, 0.005);
}

// 포화 시 windup 없음: 닿을 수 없는 목표로 10초 포화시킨 뒤 목표를 낮추면 바로 포화에서 빠져나오고
// 크게 넘치지 않는다
TEST(Pid, NoWindupWhenSaturated)
{
  PidConfig config = defaultConfig();
  config.output_max = 0.5;
  config.integral_limit = 10.0;  // 상한 자르기가 아니라 조건부 적분만으로 막는지 본다
  Pid pid(config);
  Plant plant;

  for (int i = 0; i < 200; ++i) {
    const double u = pid.update(1.0, plant.v, kDt);
    EXPECT_LE(u, 0.5);
    plant.step(u);
  }
  // 플랜트는 0.8 × 0.5 = 0.4에서 멈춘다. 적분이 쌓였다면 적분항이 매우 커져 있다
  EXPECT_NEAR(plant.v, 0.4, 0.01);
  EXPECT_LT(pid.integral(), 0.5);

  // 목표를 0.3으로 낮추면 다음 주기부터 출력이 상한 아래로 내려온다
  const double first = pid.update(0.3, plant.v, kDt);
  EXPECT_LT(first, 0.5);
  plant.step(first);
  double min_v = plant.v;
  for (int i = 0; i < 100; ++i) {
    plant.step(pid.update(0.3, plant.v, kDt));
    min_v = std::min(min_v, plant.v);
  }
  EXPECT_NEAR(plant.v, 0.3, 0.01);
  // 적분이 0에서 다시 쌓이는 동안 목표 아래로 조금 내려간다(약 4 cm/s). 포화 중 적분이 쌓였다면
  // 반대로 한참 동안 상한에 붙어 목표 위로 넘친다
  EXPECT_GT(min_v, 0.3 - 0.06);
}

// 하한 쪽도 같다: 출력 0에 붙은 채 음의 적분이 쌓이지 않는다
TEST(Pid, NoWindupAtLowerLimit)
{
  Pid pid(defaultConfig());
  for (int i = 0; i < 200; ++i) {
    // 측정이 목표보다 계속 크다 (예: 내리막에서 밀림)
    EXPECT_GE(pid.update(0.0, 0.3, kDt), 0.0);
  }
  EXPECT_GT(pid.integral(), -0.1);
}

// 게인이 모두 0이면 목표를 그대로 낸다 (PID를 켜도 지금 동작과 같다)
TEST(Pid, ZeroGainsPassThrough)
{
  Pid pid;
  EXPECT_DOUBLE_EQ(pid.update(0.5, 0.0, kDt), 0.5);
  EXPECT_DOUBLE_EQ(pid.update(0.7, 0.2, kDt), 0.7);
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.4, kDt), 0.0);
  EXPECT_DOUBLE_EQ(pid.update(1.2, 1.5, 0.0), 1.2);
}

// 미분은 측정값 기준: 목표가 계단으로 바뀌어도 미분항이 튀지 않는다
TEST(Pid, DerivativeOnMeasurementNoKick)
{
  PidConfig config;
  config.kd = 1.0;
  Pid pid(config);
  pid.update(0.0, 0.2, kDt);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.2, kDt), 1.0);
  // 측정이 늘면(가속 중) 미분항이 출력을 줄인다
  EXPECT_LT(pid.update(1.0, 0.3, kDt), 1.0);
}

TEST(Pid, ResetClearsState)
{
  Pid pid(defaultConfig());
  for (int i = 0; i < 20; ++i) {
    pid.update(0.5, 0.0, kDt);
  }
  EXPECT_GT(pid.integral(), 0.0);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

}  // namespace
}  // namespace amr_control
