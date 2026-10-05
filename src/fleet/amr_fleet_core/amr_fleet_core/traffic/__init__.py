"""Traffic Management. 설계는 amr_fleet_core README '구역 허가' 절."""

from .zones import (GRANT_GRANTED, GRANT_QUEUED, GRANT_REJECTED, GRANT_REVOKED, REASON_DEADLOCK_RESOLUTION,
                    REASON_NONE, REASON_OCCUPIED, REASON_PRIORITY_YIELD, Decision, ZoneManager, ZoneSpec)

__all__ = [
    'Decision',
    'GRANT_GRANTED',
    'GRANT_QUEUED',
    'GRANT_REJECTED',
    'GRANT_REVOKED',
    'REASON_DEADLOCK_RESOLUTION',
    'REASON_NONE',
    'REASON_OCCUPIED',
    'REASON_PRIORITY_YIELD',
    'ZoneManager',
    'ZoneSpec',
]
