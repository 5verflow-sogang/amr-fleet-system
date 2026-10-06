// TaskAssignment 수신 규칙 (amr_msgs/msg/TaskAssignment.msg 주석).
// amr_fleet_manager robot_sim.AssignmentFilter(가짜 로봇 참고 구현)와 같은 동작이다.
// 다른 도메인 패키지는 import하지 않으므로(CONTRIBUTING) C++로 다시 쓴다. ROS에 의존하지 않는다.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace amr_bt
{

enum class AssignmentKind
{
  kNew,        // 처음 받은 명령: 처리하고 Ack
  kDuplicate,  // 마지막으로 처리한 명령의 재전송: 다시 처리하지 않고 같은 Ack만
  kStale,      // 마지막보다 작은 번호 (지연으로 순서가 뒤바뀜): 무시
};

// 마지막 명령에 보낸 Ack 내용. 재전송이 오면 그대로 다시 보낸다
struct AckReply
{
  uint8_t result = 0;
  uint16_t error_code = 0;
  std::string error_message;
};

class AssignmentFilter
{
public:
  // 서버 epoch가 바뀌면 번호 기준을 초기화하고 true. 처음 받은 epoch는 재시작으로 보지 않는다
  bool on_heartbeat(uint32_t epoch)
  {
    const bool restarted = epoch_.has_value() && *epoch_ != epoch;
    if (restarted) {
      last_id_ = 0;
      last_reply_.reset();
    }
    epoch_ = epoch;
    return restarted;
  }

  AssignmentKind classify(uint32_t assignment_id) const
  {
    if (assignment_id == last_id_ && last_reply_.has_value()) {
      return AssignmentKind::kDuplicate;
    }
    if (assignment_id <= last_id_) {
      return AssignmentKind::kStale;
    }
    return AssignmentKind::kNew;
  }

  void record(uint32_t assignment_id, const AckReply & reply)
  {
    last_id_ = assignment_id;
    last_reply_ = reply;
  }

  uint32_t last_id() const {return last_id_;}
  const std::optional<AckReply> & last_reply() const {return last_reply_;}

private:
  std::optional<uint32_t> epoch_;
  uint32_t last_id_ = 0;
  std::optional<AckReply> last_reply_;
};

}  // namespace amr_bt
