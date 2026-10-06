// BT 노드와 ROS 층(amr_bt_node)이 같이 보는 작업 상태.
// BT 노드는 rclcpp를 직접 쓰지 않고 이 구조체의 함수(now, report)만 부른다. 그래서 ROS 없이 테스트할 수 있다.
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "amr_msgs/msg/error_info.hpp"
#include "amr_msgs/msg/task.hpp"

namespace amr_bt
{

struct TaskContext
{
  // TaskStatus 발행: (state, phase, error, end). end면 end_time을 채우는 마지막 이벤트
  using ReportFn = std::function<void(
        uint8_t state, const std::string & phase, const amr_msgs::msg::ErrorInfo & error, bool end)>;

  // 노드 시계 [s] (use_sim_time이면 시뮬레이션 시간)
  std::function<double()> now;
  ReportFn report;

  // 자리채움 동작 시간 [s]. Nav2·도킹을 붙이기 전까지 이동·싣기를 시간만 흘려 흉내 낸다
  double stub_move_s = 5.0;
  double stub_dwell_s = 2.0;

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
