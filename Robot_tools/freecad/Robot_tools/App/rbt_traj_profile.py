"""
rbt_traj_profile.py
time parameterisation: maps elapsed time to path params
"""
from __future__ import annotations

from typing import Protocol


class TimeProfile(Protocol):
    """
    shape of normalised path, s,  over one segment \n
    independent of speeds or limits
    """
    duration: float  # seconds

    def s_at(self, t_sec: float) -> float:
        """
        in: elapsed seconds
        out: path parameter, clamped to 0..1
        """


class TimeProfile_Constant:
    """
    Constant velocity travel: s(t) = t / duration. \n
    Contains: \n
        - duration: segement time (sec)
    """
    def __init__(self, duration: float) -> None:
        self.duration: float = max(duration, 0.0)

    def s_at(self, t_sec: float) -> float:
        """
        in: elapsed seconds
        out: path parameter, clamped to 0..1
        """
        if self.duration <= 0.0:
            return 1
        return min(max(t_sec/self.duration, 0.0), 1.0)


def make_profile(duration: float, **kw) -> TimeProfile:
    """
    in: segment duration (sec) \n
    out: TimeProfile
    """
    return TimeProfile_Constant(duration)


# class TrapezoidProfile:
#     """
#     constant-speed cruise with linear
#     acc. and de-acc. ramps.
#     Contains:
#         - travel: segment travel (deg | mm)
#         - speed: cruise speed (deg/s | mm/s)
#         - ramp_frac: fraction of travel used in ramping
#                     at each end
#     """

#     def __init__(self, travel:float, speed:float,
#                  ramp_frac: float = 0.2) -> None:
#         ...

#     @classmethod
#     def from_duration(cls, travel: float, duration:float,
#                       ramp_frac: float = 0.2) -> "TrapezoidProfile":
#         """
#         calculate travel speed for robot from
#         the time-duration provided by user
#         in: travel (deg | mm), user duration (sec)
#         out: profile with speed
#         """

#     def s_at(self, t_sec: float) -> float:
#         ...
