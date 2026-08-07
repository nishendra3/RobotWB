"""
rbt_traj_types.py
trajectory data-structures for waypoints and motion segments
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import (
    Dict, List, Literal, Optional, Protocol,
    TypeAlias
)

import FreeCAD as App  # type: ignore

Placement: TypeAlias = App.Placement
TargetMode: TypeAlias = Literal["joint", "cartesian"]
MotionType: TypeAlias = Literal["ptp", "lin"]
TimingMode: TypeAlias = Literal["speed", "duration"]

V3: TypeAlias = App.Vector
DocObj: TypeAlias = App.DocumentObject

# target modes
JOINT: TargetMode = "joint"
CARTESIAN: TargetMode = "cartesian"

# motion types
PTP: MotionType = "ptp"
LIN: MotionType = "lin"

# timing modes
BY_SPEED: TimingMode = "speed"  # calculate time from speed
BY_DURATION: TimingMode = "duration"  # time user provided

# reference frames
FRAME_ASM: str = "asm"  # robot asm coords


def new_uid() -> str:
    """
    out: 8-char hex id for a new waypoint
    """
    return uuid.uuid4().hex[:8]


@dataclass
class Waypoint:
    """
    One taught trajectory point. Stores both joint values
    and tcp pose. 'mode' picks which one is taken as the reference. \n
    Contains:
        - uid: stable id to avoid rename & reorder clashes
        - name: user label
        - mode: JOINT/CARTESIAN
        - q_doc: joint values in doc units (deg / mm)
        - tcp_in_asm: tcp pose in robot-asm coords
        - frame: reference frame key
        - motion: PTP/LIN
        - speed: per-segment speed override
        - blend: corner blend radius (mm)
        - duration: per-segment time override (sec)
    """
    uid: str
    name: str
    mode: TargetMode = JOINT
    q_doc: List[float] = field(default_factory=list)
    tcp_in_asm: Placement = field(default_factory=App.Placement)
    frame: str = FRAME_ASM
    motion: MotionType = PTP
    speed: Optional[float] = None
    blend: Optional[float] = None
    duration: Optional[float] = None

    def to_dict(self) -> Dict:
        """
        out: plain-json dict for the Waypoints_json property
        """
        base = self.tcp_in_asm.Base
        qx, qy, qz, qw = self.tcp_in_asm.Rotation.Q
        return {
            "uid": self.uid,
            "name": self.name,
            "mode": self.mode,
            "q_doc": list(self.q_doc),
            "tcp_in_asm": [base.x, base.y, base.z,
                           qx, qy, qz, qw],
            "frame": self.frame,
            "motion": self.motion,
            "speed": self.speed,
            "blend": self.blend,
            "duration": self.duration,
        }

    @staticmethod
    def from_dict(data: Dict) -> "Waypoint":
        """
        in: one Waypoints_json entry
        out: Waypoint (missing key fallbacks to default vals)
        """
        x, y, z, qx, qy, qz, qw = data.get(
            "tcp_in_asm", [0, 0, 0, 0, 0, 0, 1])
        return Waypoint(
            uid=data.get("uid") or new_uid(),
            name=data.get("name", ""),
            mode=data.get("mode", JOINT),
            q_doc=list(data.get("q_doc", [])),
            tcp_in_asm=App.Placement(App.Vector(x, y, z),
                                     App.Rotation(qx, qy, qz, qw)),
            frame=data.get("frame", FRAME_ASM),
            motion=data.get("motion", PTP),
            speed=data.get("speed"),
            blend=data.get("blend"),
            duration=data.get("duration"),
        )


@dataclass(frozen=True)
class SolvedWaypoint:
    """
    Waypoint after mode resolution (non-mutable). \n
    contains:
        - wp: source waypoint
        - q_doc: joint values to move to (None when not reachable)
        - err_msg: reason when q_doc is None else ""
    """
    wp: Waypoint
    q_doc: Optional[List[float]]
    err_msg: str = ""


class MotionSegment(Protocol):
    """
    One waypoint-to-waypoint motion
    travel_joints paces PTP motion (slowest joint on the robot)
    tavel_tcp paces LIN motion (tcp travel speed in mm/s)
    """
    travel_tcp: float  # tcp path length mm
    travel_joints: List[float]  # |dq| per joint (deg|mm)

    def q_at(self, s: float) -> List[float]:
        """
        in : s, normalised path parameter 0..1
        out: joint values in doc units
        """


@dataclass
class PtpSegment:
    """
    joint-space linear interpolation between two joint poses
    """
    q_from: List[float]
    q_to: List[float]

    # travel_tcp is set as 0 here just
    # for cosmetic reasons to satisfy MotionSegment
    travel_tcp: float = 0.0

    @property
    def travel_joints(self) -> List[float]:
        """
        out: |q_to - q_from| per joint (deg|mm)
        """
        return [abs(b-a) for a, b in zip(self.q_from, self.q_to)]

    def q_at(self, s: float) -> List[float]:
        """
        in: s, normalised path parameter 0..1
        out: q_from + s * (q_to - q_from)
        """
        s = min(max(s, 0.0), 1.0)
        return [a + s*(b-a) for a, b in zip(self.q_from, self.q_to)]


@dataclass(frozen=True)
class TimingRequest:
    """
    Robot timing mode wished by the user (non-mutable). \n
    Contains:
        - mode: BY_SPEED | BY_DURATION
        - speed: % of full joint speed (ptp) | tcp mm/s (lin)
        - duration: wanted total run time sec (only in BY_DURATION)
    """
    mode: TimingMode
    speed: float = 100.0
    duration: float = 0.0  # sec


@dataclass(frozen=True)
class TimingReport:
    """
    Feasibility check for the TimingRequest by user (non-mutable)
    & drives the motion panel status line & play/pause btn states. \n
    Contains:
        - duration: total runtime taken by trajectory plan (sec)
        - speed: applied speed value (should not exceed > 100% of max speed)
        - feasible: True when the speed/timing is valid
        - min_duration: best possible time when running at max robot speed
        - pace_sec / pace_joint: the slowest joint determining motion time
        - err_msg: error message when the timing is infeasible
    """
    duration: float
    speed: float
    feasible: bool = True
    min_duration: float = 0.0  # total time at 100% speed
    pace_seg: int = -1
    pace_joint: int = -1  # slowest joint to determine time
    err_msg: str = ""
