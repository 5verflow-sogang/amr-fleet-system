#ifndef AMR_CONTROL__CORE__PID_HPP_
#define AMR_CONTROL__CORE__PID_HPP_

// 선속도 PID: 목표 선속도(추종기 출력)를 실제 선속도(odom)가 따라가도록 명령을 보정한다.
//   u = 목표 + kp·e + ki·∫e dt − kd·d(측정)/dt,   e = 목표 − 측정
// 목표를 그대로 앞먹임(feedforward)하고 PID는 오차만 메운다. 그래서 게인이 모두 0이면 u = 목표(통과)다.
// - 미분은 측정값 기준이라 목표가 계단으로 바뀌어도 튀지 않는다(derivative kick 없음).
//   측정 잡음을 줄이려고 1차 저역통과(시정수 derivative_filter_tau)를 거친다.
// - 적분 포화 방지(anti-windup): 출력이 상·하한에 걸린 상태에서 오차가 그 방향으로 더 미는 동안은
//   적분하지 않는다(conditional integration). 적분항 자체도 ±integral_limit로 자른다.
// ROS 비의존 코어 (CONTRIBUTING 4-4).

#include <limits>

namespace amr_control
{

struct PidConfig
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  // 출력 상·하한 [m/s]
  double output_min = -std::numeric_limits<double>::infinity();
  double output_max = std::numeric_limits<double>::infinity();
  // 적분항(ki·∫e)의 절댓값 상한 [m/s]
  double integral_limit = std::numeric_limits<double>::infinity();
  // 측정값 미분의 저역통과 시정수 [s]. 0이면 거르지 않는다
  double derivative_filter_tau = 0.0;
};

class Pid
{
public:
  explicit Pid(const PidConfig & config = PidConfig());

  const PidConfig & config() const {return config_;}

  // 한 주기. dt <= 0이면 적분·미분 없이 앞먹임 + 비례만 낸다(첫 호출 등)
  double update(double setpoint, double measurement, double dt);

  // 적분·미분 상태를 비운다 (정지했을 때, 경로가 바뀌어 멈췄다 다시 출발할 때)
  void reset();

  double integral() const {return integral_;}

private:
  PidConfig config_;
  double integral_ = 0.0;  // ki·∫e (게인을 곱한 값을 쌓아 게인 변경에도 출력이 튀지 않게)
  double prev_measurement_ = 0.0;
  double derivative_ = 0.0;
  bool has_prev_ = false;
};

}  // namespace amr_control

#endif  // AMR_CONTROL__CORE__PID_HPP_
