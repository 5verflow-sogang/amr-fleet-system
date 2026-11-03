#ifndef AMR_NAV2_PLUGINS__AVOID_MODE_HPP_
#define AMR_NAV2_PLUGINS__AVOID_MODE_HPP_

#include <optional>

namespace amr_nav2_plugins
{

// TTC 회피 모드를 언제 켜고 끌지 (navigation.md 4-10절). 시각은 초(노드 시계, sim time이면 sim time)로 받아
// ROS 없이 테스트한다. DwaController가 collision_risks 수신마다 onRisks, 제어 주기마다 update를 부른다.
struct AvoidModeParams
{
  bool enabled = true;
  double enter_ttc_s = 3.0;     // 동적 트랙 최소 TTC가 이보다 작으면 켠다
  double exit_hold_s = 0.5;     // TTC >= enter_ttc_s(또는 inf)가 이만큼 이어지면 끈다
  double risk_timeout_s = 0.3;  // collision_risks가 이만큼 안 오면 끊긴 것으로 본다
};

class AvoidModeSwitch
{
public:
  explicit AvoidModeSwitch(AvoidModeParams params = AvoidModeParams())
  : params_(params)
  {
  }

  // collision_risks 한 번 수신. min_ttc는 동적 트랙의 최소 TTC(없으면 inf).
  void onRisks(double min_ttc, double now)
  {
    received_ = true;
    last_risks_ = now;
    if (!params_.enabled) {
      return;
    }
    if (min_ttc < params_.enter_ttc_s) {
      active_ = true;
      safe_since_.reset();
    } else if (active_ && !safe_since_) {
      safe_since_ = now;
    }
  }

  // 제어 주기마다: 끌 때가 됐으면 끄고 지금 상태를 돌려준다. 끊긴 동안은 그대로 둔다(켜져 있으면 켠 채로) -
  // 끊긴 사이 위험이 왔는지 모르므로 끄지 않는다. 다시 오면 안전한 TTC가 exit_hold_s 동안 새로 이어져야 끈다.
  bool update(double now)
  {
    if (!params_.enabled) {
      active_ = false;
      return active_;
    }
    if (stale(now)) {
      safe_since_.reset();
      return active_;
    }
    if (active_ && safe_since_ && now - *safe_since_ >= params_.exit_hold_s) {
      active_ = false;
      safe_since_.reset();
    }
    return active_;
  }

  bool active() const {return active_;}

  // 한 번이라도 받은 뒤 risk_timeout_s 넘게 안 왔는지. 처음부터 안 오면(추적 노드를 안 띄운 스택) 끊김이 아니다.
  bool stale(double now) const
  {
    return received_ && now - last_risks_ > params_.risk_timeout_s;
  }

private:
  AvoidModeParams params_;
  bool active_ = false;
  bool received_ = false;
  double last_risks_ = 0.0;
  std::optional<double> safe_since_;
};

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__AVOID_MODE_HPP_
