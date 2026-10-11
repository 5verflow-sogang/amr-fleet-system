#include "amr_control/core/pid.hpp"

#include <algorithm>

namespace amr_control
{

Pid::Pid(const PidConfig & config)
: config_(config)
{
}

void Pid::reset()
{
  integral_ = 0.0;
  prev_measurement_ = 0.0;
  derivative_ = 0.0;
  has_prev_ = false;
}

double Pid::update(double setpoint, double measurement, double dt)
{
  const double error = setpoint - measurement;

  // 측정값 미분 (1차 저역통과)
  if (dt > 0.0 && has_prev_) {
    const double raw = (measurement - prev_measurement_) / dt;
    const double tau = config_.derivative_filter_tau;
    const double alpha = tau > 0.0 ? dt / (tau + dt) : 1.0;
    derivative_ += alpha * (raw - derivative_);
  }
  prev_measurement_ = measurement;
  has_prev_ = true;

  const double without_integral = setpoint + config_.kp * error - config_.kd * derivative_;

  // 적분 후보. 출력이 포화된 쪽으로 더 미는 적분은 받지 않는다
  if (dt > 0.0 && config_.ki != 0.0) {
    const double candidate = std::clamp(
      integral_ + config_.ki * error * dt, -config_.integral_limit, config_.integral_limit);
    const double out = without_integral + candidate;
    const bool pushes_high = out > config_.output_max && candidate > integral_;
    const bool pushes_low = out < config_.output_min && candidate < integral_;
    if (!pushes_high && !pushes_low) {
      integral_ = candidate;
    }
  }

  return std::clamp(without_integral + integral_, config_.output_min, config_.output_max);
}

}  // namespace amr_control
