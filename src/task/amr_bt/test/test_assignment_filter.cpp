// 수신 규칙: amr_fleet_manager test_robot_sim.py의 AssignmentFilter 테스트와 같은 경우를 본다
#include <gtest/gtest.h>

#include "amr_bt/assignment_filter.hpp"

using amr_bt::AckReply;
using amr_bt::AssignmentFilter;
using amr_bt::AssignmentKind;

TEST(AssignmentFilter, NewThenDuplicateThenStale)
{
  AssignmentFilter f;
  EXPECT_EQ(f.classify(1), AssignmentKind::kNew);
  AckReply reply;
  reply.result = 1;
  reply.error_code = 102;
  f.record(1, reply);
  // 재전송: 같은 Ack를 다시 보낸다
  EXPECT_EQ(f.classify(1), AssignmentKind::kDuplicate);
  EXPECT_EQ(f.last_reply()->error_code, 102);
  f.record(5, AckReply{});
  // 번호는 연속이 아니어도 된다 (다른 로봇 몫을 건너뜀). 작은 번호는 뒤늦게 온 것
  EXPECT_EQ(f.classify(3), AssignmentKind::kStale);
  EXPECT_EQ(f.classify(6), AssignmentKind::kNew);
  EXPECT_EQ(f.last_id(), 5u);
}

TEST(AssignmentFilter, EpochChangeResets)
{
  AssignmentFilter f;
  // 처음 받은 epoch는 재시작이 아니다
  EXPECT_FALSE(f.on_heartbeat(3));
  f.record(10, AckReply{});
  EXPECT_FALSE(f.on_heartbeat(3));
  EXPECT_EQ(f.classify(1), AssignmentKind::kStale);
  // 서버 재시작: 번호가 1부터 다시 시작한다
  EXPECT_TRUE(f.on_heartbeat(4));
  EXPECT_EQ(f.classify(1), AssignmentKind::kNew);
  EXPECT_FALSE(f.last_reply().has_value());
}

TEST(AssignmentFilter, ZeroIsNeverNew)
{
  // assignment_id는 1부터다. 0은 기본값이라 처리하지 않는다
  AssignmentFilter f;
  EXPECT_EQ(f.classify(0), AssignmentKind::kStale);
}
