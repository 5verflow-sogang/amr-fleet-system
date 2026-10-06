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
  uint8_t state;
  std::string phase;
  std::string error;
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
        bool end) {events.push_back({state, phase, error.message, end});};
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

TEST_F(TaskCycleTest, CancelFailsWithReason)
{
  ctx->assign(make_task("task_2", Task::TYPE_DELIVERY));
  run(3.0);   // move_to_pickup 중
  ctx->request_cancel("canceled");
  run(10.0);
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().phase, "move_to_pickup");
  EXPECT_EQ(events.back().error, "canceled");
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
