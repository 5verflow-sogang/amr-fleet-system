"""카메라 이미지 시각 → 영상 프레임 (amr_gazebo.demo_video). ROS 없이 돈다."""

from amr_gazebo.demo_video import Resampler

MS = 10**6


def frames_used(stamps_ms, fps=15.0):
    """이미지 시각들 → 영상 프레임마다 쓴 이미지 번호."""
    r, out, prev = Resampler(fps), [], None
    for i, t in enumerate(stamps_ms):
        n = r.push(int(t * MS))
        if n < 0:
            continue
        if prev is not None:
            out += [prev] * n
        prev = i
    out += [prev] * r.finish()
    assert r.frames == len(out)
    return out


def test_gazebo_15hz_uses_every_image_once():
    # Gazebo는 시각을 1 ms로 반올림한다: 0, 67, 133, 200, ... (15 Hz)
    stamps = [round(k * 1000 / 15) for k in range(30)]
    assert frames_used(stamps) == list(range(30))


def test_missing_image_repeats_previous():
    stamps = [round(k * 1000 / 15) for k in range(10)]
    del stamps[4]                       # 4번째 이미지가 빠짐
    used = frames_used(stamps)
    assert len(used) == 10              # 영상 길이는 시뮬레이션 시간대로
    assert used[3] == used[4] == 3      # 빠진 자리에 앞 장을 한 번 더


def test_extra_images_are_skipped_and_time_stays():
    stamps = [k * 1000 / 30 for k in range(60)]   # 30 Hz로 2초
    used = frames_used(stamps)
    assert 30 <= len(used) <= 31                  # 15 fps x 2 s
    assert used == sorted(used)


def test_out_of_order_image_dropped():
    r = Resampler(15.0)
    assert r.push(0) == 0
    assert r.push(67 * MS) == 1
    assert r.push(50 * MS) == -1
