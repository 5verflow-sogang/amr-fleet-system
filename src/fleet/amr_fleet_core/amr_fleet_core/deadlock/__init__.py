"""교착 탐지·해소. 설계는 amr_fleet_core README '교착 탐지' 절."""

from .graph import choose_victim, find_deadlocks, would_deadlock

__all__ = [
    'choose_victim',
    'find_deadlocks',
    'would_deadlock',
]
