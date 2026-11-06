"""Task 메시지 → amr_fleet_core TaskInfo (fleet_manager_node.task_info)."""

from amr_fleet_core.assignment.types import TYPE_COLLECT
from amr_fleet_manager.fleet_manager_node import task_info
from amr_msgs.msg import Task


def make(task_type, pickup=(1.0, 2.0), dropoff=(5.0, 6.0)):
    msg = Task(task_id='t', task_type=task_type, priority=200)
    msg.pickup_pose.position.x, msg.pickup_pose.position.y = pickup
    msg.dropoff_pose.position.x, msg.dropoff_pose.position.y = dropoff
    msg.created_time.sec = 10
    return msg


def test_collect_constant_matches_core():
    assert Task.TYPE_COLLECT == TYPE_COLLECT == 3


def test_collect_keeps_pickup_like_delivery():
    # 빈 그릇 회수: 로봇이 먼저 테이블(pickup)로 가야 하므로 할당 비용도 픽업 기준이다
    for task_type in (Task.TYPE_DELIVERY, Task.TYPE_COLLECT):
        info = task_info(make(task_type))
        assert info.pickup_xy == (1.0, 2.0)
        assert info.start_xy == (1.0, 2.0)
        assert info.dropoff_xy == (5.0, 6.0)


def test_charge_and_return_have_no_pickup():
    for task_type in (Task.TYPE_CHARGE, Task.TYPE_RETURN):
        info = task_info(make(task_type))
        assert info.pickup_xy is None
        assert info.start_xy == (5.0, 6.0)
