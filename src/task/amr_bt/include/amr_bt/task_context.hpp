// BT 노드와 ROS 층(amr_bt_node)이 같이 보는 작업 상태.
// BT 노드는 rclcpp를 직접 쓰지 않고 이 구조체의 함수(now, report, nav, dock)만 부른다. 그래서 ROS 없이 테스트할 수 있다.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "amr_msgs/msg/error_info.hpp"
#include "amr_msgs/msg/task.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace amr_bt
{

// 액션 1건의 진행 상황. 실패면 code(ErrorInfo 대역)와 설명을 채운다
struct ActionStatus
{
  enum State {kRunning, kSucceeded, kFailed};
  State state = kRunning;
  uint16_t code = amr_msgs::msg::ErrorInfo::NONE;
  std::string message;
};

// 목표를 보내고(start) 진행을 묻고(poll) 취소한다(cancel). ROS 층은 액션 클라이언트로, 테스트는 가짜로 채운다.
// start는 이전 목표를 버리고 새로 보낸다. poll은 마지막 start의 결과만 돌려준다
template<typename Goal>
struct ActionPort
{
  std::function<void(const Goal &)> start;
  std::function<ActionStatus()> poll;
  std::function<void()> cancel;
};

// 시간만 흘려 흉내 내는 ActionPort (use_nav2·use_docking이 false일 때). seconds 뒤 성공
template<typename Goal>
ActionPort<Goal> make_stub_port(std::function<double()> now, double seconds)
{
  auto until = std::make_shared<std::optional<double>>();
  ActionPort<Goal> port;
  port.start = [now, seconds, until](const Goal &) {*until = now() + seconds;};
  port.poll = [now, until]() {
      ActionStatus s;
      if (until->has_value() && now() >= until->value()) {
        s.state = ActionStatus::kSucceeded;
      }
      return s;
    };
  port.cancel = [until]() {until->reset();};
  return port;
}

struct TaskContext
{
  // TaskStatus 발행: (state, phase, error, end). end면 end_time을 채우는 마지막 이벤트
  using ReportFn = std::function<void(
        uint8_t state, const std::string & phase, const amr_msgs::msg::ErrorInfo & error, bool end)>;

  // 노드 시계 [s] (use_sim_time이면 시뮬레이션 시간)
  std::function<double()> now;
  ReportFn report;

  // 이동(Nav2 navigate_to_pose, map 기준 자세)과 도킹(docking/dock, station_id)
  ActionPort<geometry_msgs::msg::Pose> nav;
  ActionPort<std::string> dock;

  // 싣기·내리기 시간 [s]. 명세상 도킹 뒤 가상 적재·하역 이벤트라 시간만 흘린다
  double dwell_s = 2.0;

  // ---- MoveTo 규칙 (amr_bt.yaml) ----
  double nav_timeout_s = 300.0;   // 멈춘 시간을 뺀 이동 시간 상한 → NAV_TIMEOUT
  int nav_retries = 2;            // Nav2가 실패(abort)하면 다시 보내는 횟수
  double pause_timeout_s = 60.0;  // 구역 때문에 한 번 멈춘 채 이만큼 지나면 실패 → TRAFFIC_LEASE_TIMEOUT
  // 위치 복구 때문에 한 번 멈춘 채 이만큼 지나면 실패 → LOCALIZATION_LOST. 복구 노드가 시도 1번에 최대 28 s
  // (정지 10 + 회전 15 + 확인 3), 3번이면 약 84 s라 구역보다 길게 둔다 (#178)
  double kidnap_pause_timeout_s = 120.0;
  double resume_hold_s = 0.3;     // 진행 상태가 이만큼 이어져야 다시 보낸다 (한 틱짜리 idle 걸러냄)

  // ---- 가상 배터리 (amr_bt_node가 battery.hpp 모델로 채운다) ----
  double battery_percent = 100.0;
  bool charging = false;          // Charge 노드가 켠다. 켜져 있으면 배터리가 찬다 (RobotState MODE_CHARGING)
  double charge_target = 100.0;   // 이만큼 차면 충전 작업 COMPLETED [%]
  double charge_timeout_s = 600.0;

  // ---- 다른 노드가 내는 상태 (amr_bt_node가 구독해 채운다). 아무도 안 내면 기본값 = 진행 ----
  std::string zone_state = "idle";        // zone_client/state (#137)
  std::string kidnap_state = "tracking";  // kidnap_recovery/state (#79)

  // ---- 작업 상태 ----
  std::optional<amr_msgs::msg::Task> task;   // 수락한 작업. 없으면 IDLE
  amr_msgs::msg::ErrorInfo failure;          // FAILED로 끝낼 때 보낼 원인 (코드 + 설명)
  std::string phase;                         // 지금 단계 (TaskStatus.phase)

  bool busy() const {return task.has_value();}

  bool is_delivery() const
  {
    return task.has_value() && task->task_type == amr_msgs::msg::Task::TYPE_DELIVERY;
  }

  void assign(const amr_msgs::msg::Task & t)
  {
    task = t;
    failure = amr_msgs::msg::ErrorInfo();
    phase.clear();
  }

  // 동작 노드가 실패 원인을 남긴다. 그 노드가 FAILURE를 돌려주면 FailTask가 이 값으로 FAILED를 보낸다.
  // code는 ErrorInfo 대역 (200 주행 / 300 교통 / 900 도킹 …). 하네스가 코드로 실패 원인을 나눈다
  void fail(uint16_t code, const std::string & message)
  {
    failure.code = code;
    failure.message = message;
  }

  void clear()
  {
    task.reset();
    phase.clear();
  }
};

}  // namespace amr_bt
