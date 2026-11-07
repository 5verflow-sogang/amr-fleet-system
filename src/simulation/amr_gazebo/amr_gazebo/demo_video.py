"""녹화한 카메라 이미지 → 일정 간격(fps) 영상 프레임. ROS 없이 돈다 (scripts/bag_to_mp4.py가 쓴다).

영상 시간은 이미지 header의 시뮬레이션 시각이다. 시뮬레이터가 느려도(RTF 0.07) 영상은 실제 속도로 나온다.
프레임 k(시각 t0 + k/fps)에는 그 시각 + 반 프레임 안에 들어온 가장 늦은 이미지를 쓴다. 이미지가 빠지면 앞 장을
한 번 더 쓰고, 많으면 건너뛴다. 반 프레임 여유는 Gazebo 시각이 1 ms 단위로 반올림돼(15 Hz면 67·66 ms 간격)
"딱 그 시각 이하"로 고르면 한 장씩 밀려 빠지는 것을 막는다.
"""


class Resampler:
    """이미지 시각(ns)을 차례로 받아, 직전 이미지를 영상 프레임으로 몇 번 내보낼지 알려 준다."""

    def __init__(self, fps: float):
        self.period_ns = 1e9 / fps
        self.t0 = None
        self.k = 0             # 다음에 채울 프레임 번호
        self.last = None       # 직전 이미지 시각
        self.frames = 0

    def _frame_time(self, k: int) -> float:
        return self.t0 + k * self.period_ns

    def push(self, stamp_ns: int) -> int:
        """새 이미지 시각. 직전 이미지를 내보낼 횟수를 돌려준다. 시각이 뒤로 간 이미지는 -1 (버린다)."""
        if self.t0 is None:
            self.t0 = self.last = stamp_ns
            return 0
        if stamp_ns <= self.last:
            return -1
        n = 0
        while self._frame_time(self.k) + self.period_ns / 2 < stamp_ns:
            n += 1
            self.k += 1
        self.last = stamp_ns
        self.frames += n
        return n

    def finish(self) -> int:
        """끝: 마지막 이미지를 내보낼 횟수."""
        if self.t0 is None:
            return 0
        n = 0
        while self._frame_time(self.k) <= self.last + self.period_ns / 2:
            n += 1
            self.k += 1
        self.frames += n
        return n
