// task_cycle.xml을 ROS 없이 돌린다: 가짜 시계를 넘기며 tick하고 TaskStatus 순서를 본다
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "amr_bt/bt_nodes.hpp"
#include "amr_bt/task_context.hpp"
#include "amr_msgs/msg/task.hpp"
#include "amr_msgs/msg/task_status.hpp"

using amr_msgs::msg::ErrorInfo;
using amr_msgs::msg::Task;
using amr_msgs::msg::TaskStatus;

namespace
{

struct Event
{
  std::string task_id;
  uint8_t state;
  std::string phase;
  std::string error;
  uint16_t code;
  bool end;
};

class TaskCycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<amr_bt::TaskContext>();
    ctx->now = [this]() {return clock;};
    ctx->report = [this](uint8_t state, const std::string & phase, const ErrorInfo & error,
        bool end) {
        events.push_back(
          {ctx->task ? ctx->task->task_id : "", state, phase, error.message, error.code, end});
      };
    ctx->stub_move_s = 5.0;
    ctx->stub_dwell_s = 2.0;
    amr_bt::register_nodes(factory, ctx);
    tree = factory.createTreeFromFile(TREE_FILE);
  }

  // dt마다 tick. 작업이 끝나 IDLE이 되면 멈춘다
  void run(double seconds, double dt = 0.1)
  {
    for (double t = 0; t < seconds; t += dt) {
      tree.tickRoot();
      clock += dt;
      if (ctx->task == std::nullopt && !events.empty() && events.back().end) {
        return;
      }
    }
  }

  std::vector<std::string> phases() const
  {
    std::vector<std::string> out;
    for (const auto & e : events) {
      out.push_back(std::to_string(e.state) + ":" + e.phase);
    }
    return out;
  }

  static Task make_task(const std::string & id, uint8_t type)
  {
    Task t;
    t.task_id = id;
    t.task_type = type;
    return t;
  }

  double clock = 0.0;
  std::shared_ptr<amr_bt::TaskContext> ctx;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  std::vector<Event> events;
};

const std::string P = std::to_string(TaskStatus::STATE_IN_PROGRESS) + ":";

TEST_F(TaskCycleTest, IdleWithoutTask)
{
  run(3.0);
  EXPECT_TRUE(events.empty());
}

TEST_F(TaskCycleTest, DeliveryRunsAllPhases)
{
  run(1.0);
  ctx->assign(make_task("task_1", Task::TYPE_DELIVERY));
  run(60.0);
  const std::vector<std::string> expected = {
    P + "move_to_pickup", P + "loading", P + "move_to_dropoff", P + "unloading",
    std::to_string(TaskStatus::STATE_COMPLETED) + ":"};
  EXPECT_EQ(phases(), expected);
  EXPECT_TRUE(events.back().end);
  EXPECT_FALSE(ctx->busy());
  // 이동 5 s ×2 + 싣기·내리기 2 s ×2 = 14 s. 단계가 바뀔 때마다 tick(0.1 s)이 하나씩 더 든다
  EXPECT_NEAR(clock, 1.0 + 14.0, 1.0);
}

TEST_F(TaskCycleTest, ChargeSkipsPickup)
{
  ctx->assign(make_task("charge_1", Task::TYPE_CHARGE));
  run(60.0);
  const std::vector<std::string> expected = {
    P + "move_to_dropoff", std::to_string(TaskStatus::STATE_COMPLETED) + ":"};
  EXPECT_EQ(phases(), expected);
}

Task make_task_for_fail()
{
  Task t;
  t.task_id = "fail_1";
  t.task_type = Task::TYPE_DELIVERY;
  return t;
}

// 취소는 다음 tick을 기다리지 않고 그 자리에서 FAILED를 보내고 작업을 비운다
TEST_F(TaskCycleTest, CancelFailsImmediately)
{
  ctx->assign(make_task("task_2", Task::TYPE_DELIVERY));
  run(3.0);   // move_to_pickup 중
  ASSERT_TRUE(amr_bt::cancel_task(tree, *ctx, "task_2", amr_bt::canceled_error()));
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events.back().task_id, "task_2");
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().phase, "move_to_pickup");
  EXPECT_EQ(events.back().error, "canceled");
  EXPECT_EQ(events.back().code, ErrorInfo::ASSIGN_CANCELED);   // 서버가 다시 할당하지 않는 코드 (#107)
  EXPECT_TRUE(events.back().end);
  EXPECT_FALSE(ctx->busy());

  // 이후 tick에서 취소된 작업에 아무것도 더 보내지 않는다
  const auto n = events.size();
  run(10.0);
  EXPECT_EQ(events.size(), n);
}

// 하연 리뷰 1: 취소 직후(다음 tick 전) 들어온 새 할당을 받을 수 있어야 한다
TEST_F(TaskCycleTest, NewTaskRightAfterCancel)
{
  ctx->assign(make_task("old", Task::TYPE_DELIVERY));
  run(3.0);
  ASSERT_TRUE(amr_bt::cancel_task(tree, *ctx, "old", amr_bt::canceled_error()));
  ASSERT_FALSE(ctx->busy());   // amr_bt_node는 busy()면 ALREADY_BUSY로 거부한다
  ctx->assign(make_task("new", Task::TYPE_CHARGE));
  run(30.0);

  // 하연 리뷰 2: 취소된 작업(old)의 FAILED 뒤에는 old 이벤트가 하나도 없어야 한다
  const std::string failed = std::to_string(TaskStatus::STATE_FAILED) + ":";
  const std::string done = std::to_string(TaskStatus::STATE_COMPLETED) + ":";
  std::vector<std::string> seq;
  for (const auto & e : events) {
    seq.push_back(e.task_id + "/" + std::to_string(e.state) + ":" + e.phase);
  }
  const std::vector<std::string> expected = {
    "old/" + P + "move_to_pickup", "old/" + failed + "move_to_pickup",
    "new/" + P + "move_to_dropoff", "new/" + done};
  EXPECT_EQ(seq, expected);
}

// 싣기(Dwell) 중 취소도 같은 단계 이름으로 FAILED
TEST_F(TaskCycleTest, CancelDuringLoading)
{
  ctx->assign(make_task("task_3", Task::TYPE_DELIVERY));
  run(6.0);   // 이동 5 s 뒤 loading 중
  ASSERT_EQ(ctx->phase, "loading");
  ASSERT_TRUE(amr_bt::cancel_task(tree, *ctx, "task_3", amr_bt::canceled_error()));
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().phase, "loading");
}

// 하던 작업이 아닌 취소(이미 끝났거나 다른 작업)는 무시하고 하던 작업을 계속한다
TEST_F(TaskCycleTest, CancelOfOtherTaskIgnored)
{
  ctx->assign(make_task("mine", Task::TYPE_RETURN));
  run(1.0);
  EXPECT_FALSE(amr_bt::cancel_task(tree, *ctx, "someone_else", amr_bt::canceled_error()));
  EXPECT_TRUE(ctx->busy());
  run(30.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_COMPLETED);
}

// 하연 리뷰 3: 동작 노드가 ctx.fail(code, …)로 남긴 오류 코드가 FAILED 보고에 실린다
TEST(FailTaskTest, FailureCarriesErrorCode)
{
  auto ctx = std::make_shared<amr_bt::TaskContext>();
  std::vector<ErrorInfo> errors;
  ctx->now = []() {return 0.0;};
  ctx->report = [&errors](uint8_t state, const std::string &, const ErrorInfo & error, bool) {
      if (state == TaskStatus::STATE_FAILED) {
        errors.push_back(error);
      }
    };
  BT::BehaviorTreeFactory factory;
  amr_bt::register_nodes(factory, ctx);
  // 실패하는 동작을 AlwaysFailure로 흉내 낸다 (작업 9에서 Nav2·도킹 노드가 이 자리에 온다)
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="T"><BehaviorTree ID="T">
      <Sequence><WaitForTask/>
        <Fallback>
          <Sequence><ReportPhase phase="move_to_pickup"/><AlwaysFailure/></Sequence>
          <FailTask/>
        </Fallback>
      </Sequence>
    </BehaviorTree></root>)");
  ctx->assign(make_task_for_fail());
  ctx->fail(ErrorInfo::NAV_TIMEOUT, "goal not reached in time");
  tree.tickRoot();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].code, ErrorInfo::NAV_TIMEOUT);
  EXPECT_EQ(errors[0].message, "goal not reached in time");
  EXPECT_FALSE(ctx->busy());
}

TEST_F(TaskCycleTest, NextTaskAfterCompletion)
{
  ctx->assign(make_task("a", Task::TYPE_RETURN));
  run(30.0);
  ctx->assign(make_task("b", Task::TYPE_RETURN));
  run(30.0);
  const std::string done = std::to_string(TaskStatus::STATE_COMPLETED) + ":";
  const std::vector<std::string> expected = {P + "move_to_dropoff", done, P + "move_to_dropoff", done};
  EXPECT_EQ(phases(), expected);
}

}  // namespace
