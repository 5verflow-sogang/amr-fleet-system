"""clock_throttle 규칙 (rclpy 없이)."""

from amr_gazebo.clock_throttle import should_publish

MS = 1_000_000


def test_first_tick_is_sent():
    assert should_publish(None, 0, 5 * MS)


def test_1khz_input_becomes_200hz():
    sent, last = 0, None
    for t in range(0, 1000 * MS, MS):  # 1초 동안 1 ms 간격
        if should_publish(last, t, 5 * MS):
            sent, last = sent + 1, t
    assert sent == 200


def test_time_going_back_is_sent_at_once():
    assert should_publish(10_000 * MS, 0, 5 * MS)
