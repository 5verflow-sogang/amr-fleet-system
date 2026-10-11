// task_cycle.xml을 ROS 없이 돌린다: 가짜 시계를 넘기며 tick하고 TaskStatus 순서를 본다.
// 이동·도킹은 가짜 ActionPort(FakeAction)로 결과·걸리는 시간을 정한다
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "amr_bt/battery.hpp"
#include "amr_bt/bt_nodes.hpp"
#include "amr_bt/task_context.hpp"
#include "amr_msgs/action/dock.hpp"
#include "amr_msgs/msg/task.hpp"
#include "amr_msgs/msg/task_status.hpp"

using amr_bt::ActionStatus;
using amr_msgs::msg::ErrorInfo;
using amr_msgs::msg::Task;
using amr_msgs::msg::TaskStatus;
using DockResult = amr_msgs::action::Dock::Result;

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

// 가짜 액션: start 뒤 duration 초가 지나면 outcomes[시도 번호]를 돌려준다 (목록을 넘으면 성공)
struct FakeAction
{
  double duration = 1.0;
  std::vector<ActionStatus> outcomes;
  int starts = 0;
  int cancels = 0;
  bool active = false;
  double started_at = 0.0;

  template<typename Goal>
  amr_bt::ActionPort<Goal> port(const double * clock)
  {
    amr_bt::ActionPort<Goal> p;
    p.start = [this, clock](const Goal &) {
        ++starts;
        active = true;
        started_at = *clock;
      };
    p.poll = [this, clock]() {
        ActionStatus s;
        if (active && *clock - started_at >= duration) {
          const size_t i = static_cast<size_t>(starts - 1);
          s = i < outcomes.size() ? outcomes[i] : ActionStatus{ActionStatus::kSucceeded, 0, ""};
        }
        return s;
      };
    p.cancel = [this]() {
        if (active) {
          ++cancels;
        }
        active = false;
      };
    return p;
  }
};

ActionStatus failed(uint16_t code, const std::string & message)
{
  return ActionStatus{ActionStatus::kFailed, code, message};
}

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
    nav.duration = 5.0;
    dock.duration = 1.0;
    ctx->nav = nav.port<geometry_msgs::msg::Pose>(&clock);
    ctx->dock = dock.port<std::string>(&clock);
    ctx->dwell_s = 2.0;
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

  static Task make_task(
    const std::string & id, uint8_t type, const std::string & pickup = "",
    const std::string & dropoff = "")
  {
    Task t;
    t.task_id = id;
    t.task_type = type;
    t.pickup_station_id = pickup;
    t.dropoff_station_id = dropoff;
    return t;
  }

  double clock = 0.0;
  FakeAction nav;
  FakeAction dock;
  std::shared_ptr<amr_bt::TaskContext> ctx;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  std::vector<Event> events;
};

const std::string P = std::to_string(TaskStatus::STATE_IN_PROGRESS) + ":";
const std::string DONE = std::to_string(TaskStatus::STATE_COMPLETED) + ":";
const std::string FAILED = std::to_string(TaskStatus::STATE_FAILED) + ":";

// 명세 KPI: BT 노드 15개 이상 (서브트리를 펼친 실제 트리 기준)
TEST_F(TaskCycleTest, TreeHasAtLeast15Nodes)
{
  const size_t n = tree.nodes.size();   // v3: 서브트리 안 노드까지 모두 담긴다
  std::cout << "BT nodes: " << n << std::endl;
  EXPECT_GE(n, 15u);
}

TEST_F(TaskCycleTest, IdleWithoutTask)
{
  run(3.0);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(nav.starts, 0);
}

// architecture.md 5-1절: 이동 → 도킹 → 싣기 → 이동 → 도킹 → 내리기 → COMPLETED
TEST_F(TaskCycleTest, DeliveryRunsAllPhases)
{
  run(1.0);
  ctx->assign(make_task("task_1", Task::TYPE_DELIVERY, "dock_1", "rack_a1"));
  run(60.0);
  const std::vector<std::string> expected = {
    P + "move_to_pickup", P + "docking", P + "loading",
    P + "move_to_dropoff", P + "docking", P + "unloading", DONE};
  EXPECT_EQ(phases(), expected);
  EXPECT_TRUE(events.back().end);
  EXPECT_FALSE(ctx->busy());
  EXPECT_EQ(nav.starts, 2);
  EXPECT_EQ(dock.starts, 2);
  // (이동 5 s + 도킹 1 s + 싣기 2 s) ×2 = 16 s. 단계가 바뀔 때마다 tick(0.1 s)이 하나씩 더 든다
  EXPECT_NEAR(clock, 1.0 + 16.0, 1.5);
}

// 스테이션 이름 없이 자세만 받은 작업은 도킹을 건너뛴다
TEST_F(TaskCycleTest, DeliveryWithoutStationsSkipsDocking)
{
  ctx->assign(make_task("task_1", Task::TYPE_DELIVERY));
  run(60.0);
  const std::vector<std::string> expected = {
    P + "move_to_pickup", P + "loading", P + "move_to_dropoff", P + "unloading", DONE};
  EXPECT_EQ(phases(), expected);
  EXPECT_EQ(dock.starts, 0);
}

TEST_F(TaskCycleTest, ChargeDocksAtCharger)
{
  ctx->assign(make_task("charge_1", Task::TYPE_CHARGE, "", "charger_c1"));
  run(60.0);
  const std::vector<std::string> expected = {P + "move_to_dropoff", P + "docking", P + "charging", DONE};
  EXPECT_EQ(phases(), expected);
  EXPECT_EQ(dock.starts, 1);
}

// 충전: 도킹 뒤 charging 단계에서 배터리가 charge_target까지 찰 때까지 기다린다 (ctx.charging 켬)
TEST_F(TaskCycleTest, ChargeWaitsUntilFull)
{
  ctx->battery_percent = 30.0;
  ctx->charge_target = 90.0;
  ctx->assign(make_task("charge_1", Task::TYPE_CHARGE, "", "charger_c1"));
  bool saw_charging = false;
  for (int i = 0; i < 2000 && ctx->busy(); ++i) {
    tree.tickRoot();
    clock += 0.1;
    if (ctx->charging) {
      saw_charging = true;
      ctx->battery_percent += 0.05;   // 0.5 %/s
    }
  }
  EXPECT_TRUE(saw_charging);
  EXPECT_FALSE(ctx->charging);
  EXPECT_GE(ctx->battery_percent, 90.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_COMPLETED);
  // 이동 5 s + 도킹 1 s + 충전 60 %/0.5 = 120 s
  EXPECT_NEAR(clock, 126.0, 2.0);
}

TEST_F(TaskCycleTest, ChargeTimeout)
{
  ctx->battery_percent = 30.0;
  ctx->charge_timeout_s = 10.0;
  ctx->assign(make_task("charge_1", Task::TYPE_CHARGE, "", "charger_c1"));
  run(60.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().phase, "charging");
  EXPECT_FALSE(ctx->charging);
}

// 충전 중 취소: charging을 끈다
TEST_F(TaskCycleTest, CancelDuringCharging)
{
  ctx->battery_percent = 30.0;
  ctx->assign(make_task("charge_1", Task::TYPE_CHARGE, "", "charger_c1"));
  run(10.0);
  ASSERT_TRUE(ctx->charging);
  ASSERT_TRUE(amr_bt::cancel_task(tree, *ctx, "charge_1", amr_bt::canceled_error()));
  EXPECT_FALSE(ctx->charging);
  EXPECT_EQ(events.back().phase, "charging");
}

TEST_F(TaskCycleTest, ReturnDoesNotDock)
{
  ctx->assign(make_task("return_1", Task::TYPE_RETURN, "", "wait_1"));
  run(60.0);
  const std::vector<std::string> expected = {P + "move_to_dropoff", DONE};
  EXPECT_EQ(phases(), expected);
  EXPECT_EQ(dock.starts, 0);
}

// ---- 도킹 재시도 (명세: 3회 실패 시 대체 작업) ----

TEST_F(TaskCycleTest, DockRetriesThenSucceeds)
{
  dock.outcomes = {failed(DockResult::FAILED_TO_DETECT_DOCK, "no marker"),
    failed(DockResult::FAILED_TO_CONTROL, "error 3 cm")};
  ctx->assign(make_task("charge_1", Task::TYPE_CHARGE, "", "charger_c1"));
  run(60.0);
  EXPECT_EQ(dock.starts, 3);
  const std::vector<std::string> expected = {P + "move_to_dropoff", P + "docking", P + "charging", DONE};
  EXPECT_EQ(phases(), expected);   // 재시도해도 docking 보고는 한 번
}

// 3번 다 실패하면 마지막 Dock.action 코드로 FAILED → 서버가 다른 로봇에 다시 할당한다
TEST_F(TaskCycleTest, DockFailsThreeTimes)
{
  const auto miss = failed(DockResult::FAILED_TO_DETECT_DOCK, "no marker");
  dock.outcomes = {miss, miss, miss};
  ctx->assign(make_task("task_1", Task::TYPE_DELIVERY, "dock_1", "rack_a1"));
  run(60.0);
  EXPECT_EQ(dock.starts, 3);
  EXPECT_EQ(nav.starts, 1);   // 픽업에서 끝났다
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().phase, "docking");
  EXPECT_EQ(events.back().code, DockResult::FAILED_TO_DETECT_DOCK);
  EXPECT_NE(events.back().error.find("dock_1 failed 3 times"), std::string::npos)
    << events.back().error;
  EXPECT_FALSE(ctx->busy());
}

// 다음 작업은 시도 횟수를 새로 센다
TEST_F(TaskCycleTest, DockAttemptsResetPerTask)
{
  const auto miss = failed(DockResult::FAILED_TO_DETECT_DOCK, "no marker");
  dock.outcomes = {miss, miss, miss, miss};   // 다음 작업 첫 시도도 실패, 두 번째에 성공
  ctx->assign(make_task("a", Task::TYPE_CHARGE, "", "charger_c1"));
  run(60.0);
  ctx->assign(make_task("b", Task::TYPE_CHARGE, "", "charger_c1"));
  run(60.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_COMPLETED);
  EXPECT_EQ(dock.starts, 5);
}

// ---- 이동: Nav2 실패·타임아웃 ----

TEST_F(TaskCycleTest, NavRetriesThenFails)
{
  ctx->nav_retries = 2;
  const auto abort = failed(ErrorInfo::NAV_NO_PATH, "navigate_to_pose aborted");
  nav.outcomes = {abort, abort, abort};
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(60.0);
  EXPECT_EQ(nav.starts, 3);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().phase, "move_to_dropoff");
  EXPECT_EQ(events.back().code, ErrorInfo::NAV_NO_PATH);
  EXPECT_NE(events.back().error.find("3 attempts"), std::string::npos) << events.back().error;
}

TEST_F(TaskCycleTest, NavRetrySucceeds)
{
  nav.outcomes = {failed(ErrorInfo::NAV_NO_PATH, "aborted")};
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(60.0);
  EXPECT_EQ(nav.starts, 2);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_COMPLETED);
}

TEST_F(TaskCycleTest, NavTimeout)
{
  nav.duration = 1000.0;
  ctx->nav_timeout_s = 10.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(30.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().code, ErrorInfo::NAV_TIMEOUT);
  EXPECT_EQ(nav.cancels, 1);
  EXPECT_NEAR(clock, 10.0, 0.5);
}

// ---- 이동: 구역(zone_client/state)·위치 복구(kidnap_recovery/state)로 멈춤과 재개 ----

TEST_F(TaskCycleTest, ZoneWaitingPausesAndResumes)
{
  nav.duration = 1000.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(2.0);
  ASSERT_EQ(nav.starts, 1);
  ctx->zone_state = "waiting";
  run(5.0);
  EXPECT_EQ(nav.cancels, 1);   // 멈춤: Nav2 목표 취소
  EXPECT_EQ(nav.starts, 1);
  EXPECT_TRUE(ctx->busy());    // MoveTo는 RUNNING으로 기다린다
  nav.duration = 3.0;
  ctx->zone_state = "approaching";
  run(1.0);
  EXPECT_EQ(nav.starts, 2);    // resume_hold_s(0.3 s) 뒤 같은 목표를 다시 보낸다
  run(10.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_COMPLETED);
}

// 멈춤 중 진행 상태가 한 틱만 나오면(후진 → 반납 → 재요청 사이 idle) 다시 보내지 않는다
TEST_F(TaskCycleTest, ResumeHoldFiltersBlip)
{
  nav.duration = 1000.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(1.0);
  ctx->zone_state = "backing";
  run(2.0);
  ctx->zone_state = "idle";
  run(0.1);
  ctx->zone_state = "waiting";
  run(2.0);
  EXPECT_EQ(nav.starts, 1);
  EXPECT_EQ(nav.cancels, 1);
}

// 같은 진행 상태가 1초마다 다시 와도 진행 중이면 목표를 다시 보내지 않는다
TEST_F(TaskCycleTest, RepeatedGoStateDoesNotResend)
{
  nav.duration = 1000.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  for (int i = 0; i < 5; ++i) {
    ctx->zone_state = "approaching";
    run(1.0);
  }
  EXPECT_EQ(nav.starts, 1);
  EXPECT_EQ(nav.cancels, 0);
}

TEST_F(TaskCycleTest, ZonePauseTimeout)
{
  nav.duration = 1000.0;
  ctx->pause_timeout_s = 20.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(1.0);
  ctx->zone_state = "blocked";
  run(30.0);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().code, ErrorInfo::TRAFFIC_LEASE_TIMEOUT);
}

// 멈춘 시간은 이동 타임아웃에 넣지 않는다
TEST_F(TaskCycleTest, PausedTimeNotCountedInNavTimeout)
{
  nav.duration = 1000.0;
  ctx->nav_timeout_s = 10.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(6.0);
  ctx->zone_state = "waiting";
  run(20.0);
  ASSERT_TRUE(ctx->busy());
  ctx->zone_state = "inside";
  run(3.0);
  EXPECT_TRUE(ctx->busy());   // 진행 6 s + 3 s < 10 s
  run(5.0);
  EXPECT_EQ(events.back().code, ErrorInfo::NAV_TIMEOUT);
}

// zone_client가 후진 안전 한도로 멈춘 상태(failed)는 기다려도 안 풀려 바로 실패한다
TEST_F(TaskCycleTest, ZoneFailedAborts)
{
  nav.duration = 1000.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(1.0);
  ctx->zone_state = "backing";
  run(1.0);
  ctx->zone_state = "failed";
  run(0.3);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_FAILED);
  EXPECT_EQ(events.back().code, ErrorInfo::TRAFFIC_REVOKED);
}

TEST_F(TaskCycleTest, KidnapPausesUntilTracking)
{
  nav.duration = 1000.0;
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(1.0);
  ctx->kidnap_state = "rotating";
  run(5.0);
  EXPECT_EQ(nav.cancels, 1);
  ctx->kidnap_state = "tracking";
  run(1.0);
  EXPECT_EQ(nav.starts, 2);
}

// 작업 시작 때 이미 멈춤 상태면 목표를 보내지 않고 기다린다
TEST_F(TaskCycleTest, StartWhilePaused)
{
  ctx->kidnap_state = "settling";
  ctx->assign(make_task("r", Task::TYPE_RETURN));
  run(3.0);
  EXPECT_EQ(nav.starts, 0);
  ctx->kidnap_state = "tracking";
  run(20.0);
  EXPECT_EQ(nav.starts, 1);
  EXPECT_EQ(events.back().state, TaskStatus::STATE_COMPLETED);
}

// ---- 취소 ----

// 취소는 다음 tick을 기다리지 않고 그 자리에서 FAILED를 보내고 작업을 비운다. 진행 중 Nav2 목표도 취소한다
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
  EXPECT_EQ(nav.cancels, 1);

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
  ctx->assign(make_task("new", Task::TYPE_RETURN));
  run(30.0);

  // 하연 리뷰 2: 취소된 작업(old)의 FAILED 뒤에는 old 이벤트가 하나도 없어야 한다
  std::vector<std::string> seq;
  for (const auto & e : events) {
    seq.push_back(e.task_id + "/" + std::to_string(e.state) + ":" + e.phase);
  }
  const std::vector<std::string> expected = {
    "old/" + P + "move_to_pickup", "old/" + FAILED + "move_to_pickup",
    "new/" + P + "move_to_dropoff", "new/" + DONE};
  EXPECT_EQ(seq, expected);
}

// 도킹 중 취소: Dock.action 목표도 취소한다
TEST_F(TaskCycleTest, CancelDuringDocking)
{
  dock.duration = 100.0;
  ctx->assign(make_task("task_3", Task::TYPE_DELIVERY, "dock_1", "rack_a1"));
  run(7.0);
  ASSERT_EQ(ctx->phase, "docking");
  ASSERT_TRUE(amr_bt::cancel_task(tree, *ctx, "task_3", amr_bt::canceled_error()));
  EXPECT_EQ(events.back().phase, "docking");
  EXPECT_EQ(dock.cancels, 1);
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

TEST_F(TaskCycleTest, NextTaskAfterCompletion)
{
  ctx->assign(make_task("a", Task::TYPE_RETURN));
  run(30.0);
  ctx->assign(make_task("b", Task::TYPE_RETURN));
  run(30.0);
  const std::vector<std::string> expected = {P + "move_to_dropoff", DONE, P + "move_to_dropoff", DONE};
  EXPECT_EQ(phases(), expected);
}

TEST(BatteryTest, DrainsAndCharges)
{
  amr_bt::BatteryConfig cfg;
  cfg.initial_percent = 50.0;
  cfg.move_per_m = 0.1;
  cfg.idle_per_s = 0.01;
  cfg.charge_per_s = 1.0;
  amr_bt::Battery b(cfg);
  b.update(10.0, 20.0, false);            // 20 m × 0.1 + 10 s × 0.01 = 2.1 %
  EXPECT_NEAR(b.percent(), 47.9, 1e-9);
  b.update(10.0, 0.0, true);              // 충전 10 %
  EXPECT_NEAR(b.percent(), 57.9, 1e-9);
  b.update(100.0, 0.0, true);
  EXPECT_DOUBLE_EQ(b.percent(), 100.0);   // 상한
  b.update(1.0, 5000.0, false);
  EXPECT_DOUBLE_EQ(b.percent(), 0.0);     // 하한
  b.update(-1.0, 10.0, false);            // 시간이 거꾸로면 무시
  EXPECT_DOUBLE_EQ(b.percent(), 0.0);
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
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="T"><BehaviorTree ID="T">
      <Sequence><WaitForTask/>
        <Fallback>
          <Sequence><ReportPhase phase="move_to_pickup"/><AlwaysFailure/></Sequence>
          <FailTask/>
        </Fallback>
      </Sequence>
    </BehaviorTree></root>)");
  Task t;
  t.task_id = "fail_1";
  ctx->assign(t);
  ctx->fail(ErrorInfo::NAV_TIMEOUT, "goal not reached in time");
  tree.tickRoot();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].code, ErrorInfo::NAV_TIMEOUT);
  EXPECT_EQ(errors[0].message, "goal not reached in time");
  EXPECT_FALSE(ctx->busy());
}

}  // namespace
