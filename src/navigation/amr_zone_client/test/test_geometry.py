"""경계까지 남은 거리, 정지 속도 프로파일 (ROS 비의존 순수 함수)."""

import math

import pytest

from amr_zone_client.geometry import (
    combine_caps, distance, distance_to_rect, point_in_rect, ramp_cap_down, ramp_speed,
    remaining_distance_to_rect,
    speed_cap_for_stop,
)

RECT = (5.0, 6.0, -1.0, 1.0)


def straight_path(x_from, x_to, step=0.1, y=0.0):
    n = int(round((x_to - x_from) / step))
    return [(x_from + i * step, y) for i in range(1, n + 1)]


def test_point_in_rect():
    assert point_in_rect((5.5, 0.0), RECT)
    assert point_in_rect((5.0, -1.0), RECT)  # 경계 포함
    assert not point_in_rect((4.9, 0.0), RECT)
    assert not point_in_rect((5.5, 1.1), RECT)


def test_remaining_distance_straight_path():
    path = straight_path(2.0, 10.0)
    assert math.isclose(remaining_distance_to_rect(path, (2.0, 0.0), RECT), 3.0, abs_tol=1e-6)


def test_remaining_distance_already_inside():
    path = straight_path(5.5, 10.0)
    assert remaining_distance_to_rect(path, (5.5, 0.0), RECT) == 0.0


def test_remaining_distance_path_never_reaches_zone():
    path = straight_path(0.0, 3.0, y=5.0)  # y=5는 RECT(y -1..1)와 안 만난다
    assert remaining_distance_to_rect(path, (0.0, 5.0), RECT) is None


def test_remaining_distance_diagonal_path():
    # (0,0) -> (10,0)을 y=0으로 가로지르는 대각선 경로. x=5에서 들어간다
    path = [(10.0, 0.0)]
    assert math.isclose(remaining_distance_to_rect(path, (0.0, 0.0), RECT), 5.0, abs_tol=1e-6)


def test_remaining_distance_ignores_points_behind_robot():
    # 경로가 로봇 뒤쪽 점도 담고 있으면, 그 점부터 거꾸로 거리를 쌓지 않고 로봇과 가장 가까운
    # 점부터 잰다 (#66 하연 리뷰)
    far_rect = (8.0, 9.0, -1.0, 1.0)
    path = straight_path(0.0, 10.0)  # 0.1 ~ 10.0, 로봇(5.0) 뒤쪽 점(0.1~4.9)도 섞여 있다
    assert math.isclose(remaining_distance_to_rect(path, (5.0, 0.0), far_rect), 3.0, abs_tol=1e-6)


def test_remaining_distance_max_distance_cutoff():
    path = straight_path(2.0, 10.0)
    # 실제 거리는 3.0m. max_distance를 그보다 짧게 주면 못 찾은 것으로 본다
    assert remaining_distance_to_rect(path, (2.0, 0.0), RECT, max_distance=2.0) is None
    assert math.isclose(
        remaining_distance_to_rect(path, (2.0, 0.0), RECT, max_distance=3.0), 3.0, abs_tol=1e-6)


def test_speed_cap_for_stop_profile():
    # 경계 3 m 앞(트리거 지점), stop_margin 0.5, decel 1.0 -> sqrt(2*1*2.5)
    assert math.isclose(speed_cap_for_stop(3.0, 0.5, 1.0), math.sqrt(5.0), abs_tol=1e-6)
    # stop_margin 지점 = 속도 0
    assert speed_cap_for_stop(0.5, 0.5, 1.0) == 0.0
    # 이미 stop_margin보다 가까우면 0 (음수 거리로 내려가지 않는다)
    assert speed_cap_for_stop(0.0, 0.5, 1.0) == 0.0
    # 단조 감소
    assert speed_cap_for_stop(3.0, 0.5, 1.0) > speed_cap_for_stop(1.0, 0.5, 1.0) > speed_cap_for_stop(0.6, 0.5, 1.0)


def test_combine_caps_picks_smaller():
    assert combine_caps(1.0, 0.5) == 0.5
    assert combine_caps(None, 0.5) == 0.5
    assert combine_caps(None, None) is None
    assert combine_caps() is None


def test_ramp_cap_down_relaxation_is_immediate():
    # 완화(상한이 올라가거나 그대로)는 바로 반영한다 - 부드럽게 할 필요 없음
    assert ramp_cap_down(1.0, 0.5, dt=1.0, decel=1.0, no_limit_value=2.0) == 1.0
    assert ramp_cap_down(None, 0.5, dt=1.0, decel=1.0, no_limit_value=2.0) is None


def test_ramp_cap_down_limits_drop_rate():
    # decel=1.0 m/s^2, dt=0.1s -> 0.1 m/s 이상 못 내려간다
    assert ramp_cap_down(0.0, 0.5, dt=0.1, decel=1.0, no_limit_value=2.0) == pytest.approx(0.4)


def test_ramp_cap_down_zero_dt_holds_previous_value():
    assert ramp_cap_down(0.0, 0.5, dt=0.0, decel=1.0, no_limit_value=2.0) == 0.5


def test_ramp_cap_down_from_no_limit_ramps_from_max_velocity():
    # prev가 None(제한 없음)이면 no_limit_value(로봇 최고속)에서부터 내려간다
    result = ramp_cap_down(0.0, None, dt=0.1, decel=1.0, no_limit_value=2.0)
    assert result == pytest.approx(1.9)


def test_ramp_cap_down_reaching_no_limit_value_returns_none():
    assert ramp_cap_down(2.0, 1.9, dt=1.0, decel=1.0, no_limit_value=2.0) is None


def test_ramp_cap_down_starts_from_current_speed_when_lower_than_prev():
    # prev가 None(최고속 2.0으로 다룸)이어도, 로봇이 실제로는 0.5m/s로만 가고 있으면
    # 그 0.5에서부터 내려간다 - 가짜로 높은 2.0에서부터 내려가면 감속이 실제 속도를
    # 따라잡을 때까지 미뤄진다(#93 연우 리뷰).
    result = ramp_cap_down(
        0.0, None, dt=0.1, decel=1.0, no_limit_value=2.0, current_speed_mps=0.5)
    assert result == pytest.approx(0.4)


def test_ramp_cap_down_current_speed_never_raises_the_ramp():
    # 현재 속도가 prev보다 빠르면(이미 더 낮은 상한으로 램프 중인데 명령 속도가 그보다
    # 높은 경우) min()이라 영향이 없다 - 올리는 방향으로는 안 쓴다.
    result = ramp_cap_down(
        0.0, 0.3, dt=0.1, decel=1.0, no_limit_value=2.0, current_speed_mps=1.5)
    assert result == pytest.approx(0.2)  # current_speed_mps 없을 때와 동일


def test_ramp_cap_down_current_speed_none_keeps_old_behavior():
    # 안 주면(기본값 None) 기존처럼 no_limit_value에서부터 내려간다
    result = ramp_cap_down(0.0, None, dt=0.1, decel=1.0, no_limit_value=2.0)
    assert result == pytest.approx(1.9)


# --- distance, ramp_speed (BACKING 후진 속도 가감속, 10/11 결정) ---


def test_distance_basic():
    assert distance((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)


def test_ramp_speed_increases_within_rate():
    # rate(1.0) x dt(0.1) = 0.1 이내로만 올라간다
    assert ramp_speed(0.0, 0.15, dt=0.1, rate=1.0) == pytest.approx(0.1)


def test_ramp_speed_reaches_target_when_within_step():
    # 남은 차이(0.05)가 이번 틱 최대 변화량(0.1)보다 작으면 바로 target
    assert ramp_speed(0.05, 0.15, dt=0.1, rate=1.0) == pytest.approx(0.15)


def test_ramp_speed_decreases_within_rate_symmetric():
    # 내려갈 때도 같은 rate로 - ramp_cap_down과 달리 올릴 때도 같게 묶는다(후진 시작도
    # 느닷없이가 아니라 가속해야 해서)
    assert ramp_speed(0.15, 0.0, dt=0.05, rate=1.0) == pytest.approx(0.1)


def test_ramp_speed_dt_zero_keeps_current():
    assert ramp_speed(0.05, 0.15, dt=0.0, rate=1.0) == 0.05


def test_distance_to_rect():
    rect = (5.0, 6.0, -1.0, 1.0)
    assert distance_to_rect((5.5, 0.0), rect) == 0.0      # 안
    assert distance_to_rect((2.0, 0.0), rect) == pytest.approx(3.0)   # 옆
    assert distance_to_rect((2.0, 5.0), rect) == pytest.approx(5.0)   # 모서리 (3, 4)
    assert distance_to_rect((5.5, 3.0), rect) == pytest.approx(2.0)   # 위
