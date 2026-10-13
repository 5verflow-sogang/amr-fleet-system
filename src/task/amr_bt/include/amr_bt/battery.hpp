// 가상 배터리 모델 (RobotState.battery_percent). 명세에 수치가 없어 시뮬레이션 운용에 맞춘 값이다.
// 이동 거리와 시간으로 닳고, 충전소에 도킹해 충전 중(charging)이면 찬다. ROS 없이 테스트한다.
#pragma once

#include <algorithm>

namespace amr_bt
{

struct BatteryConfig
{
  double initial_percent = 100.0;
  double move_per_m = 0.05;       // 이동 1 m당 [%]. 배송 1건(약 50 m) ≈ 2.5 %
  double idle_per_s = 0.002;      // 가만히 있어도 [%/s] (센서·컴퓨터, 약 7 %/h)
  double charge_per_s = 0.5;      // 충전 [%/s]. 0 → 100 %가 200 s (시뮬레이션이라 빠르게)
};

class Battery
{
public:
  explicit Battery(const BatteryConfig & config = BatteryConfig())
  : config_(config), percent_(std::clamp(config.initial_percent, 0.0, 100.0)) {}

  // dt 동안 distance만큼 움직였을 때. 충전 중이면 소모 대신 충전한다
  void update(double dt, double distance, bool charging)
  {
    if (dt <= 0.0) {
      return;
    }
    if (charging) {
      percent_ += config_.charge_per_s * dt;
    } else {
      percent_ -= config_.move_per_m * std::max(0.0, distance) + config_.idle_per_s * dt;
    }
    percent_ = std::clamp(percent_, 0.0, 100.0);
  }

  double percent() const {return percent_;}

private:
  BatteryConfig config_;
  double percent_;
};

}  // namespace amr_bt
