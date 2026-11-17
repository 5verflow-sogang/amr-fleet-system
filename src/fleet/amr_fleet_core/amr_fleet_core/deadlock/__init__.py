"""교착 탐지·해소. 설계는 amr_fleet_core README '교착 탐지' 절."""

from .graph import choose_victim, find_deadlocks, would_deadlock
from .standoff import BACK, FORWARD, StandoffConfig, StandoffDetector, YieldOrder

__all__ = [
    'BACK',
    'FORWARD',
    'StandoffConfig',
    'StandoffDetector',
    'YieldOrder',
    'choose_victim',
    'find_deadlocks',
    'would_deadlock',
]
