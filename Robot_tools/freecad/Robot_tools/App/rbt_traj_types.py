"""
rbt_traj_types.py
trajectory data-structures for waypoints and motion segments
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import (
    Dict, List, Literal, Optional, TypeAlias
    )

import FreeCAD as App  # type: ignore

Placement: TypeAlias = App.Placement
TargetMode: TypeAlias = Literal["joint", "cartesian"]
MotionType: TypeAlias = Literal["ptp", "lin"]

V3: TypeAlias = App.Vector
DocObj: TypeAlias = App.DocumentObject

# target modes
JOINT: TargetMode = "joint"
CARTESIAN: TargetMode = "cartesian"

# motion types
PTP: MotionType = "ptp"
LIN: MotionType = "lin"

# reference frames
FRAME_WORLD: str = "world"  # FreeCAD world coords (WCS)


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
        - tcp_in_world: tcp pose in world coords (WCS)
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
    tcp_in_world: Placement = field(default_factory=App.Placement)
    motion: MotionType = PTP
    speed: Optional[float] = None
    blend: Optional[float] = None
    duration: Optional[float] = None

    def to_dict(self) -> Dict:
        """
        out: plain-json dict for the WaypointsJson property
        """
        base = self.tcp_in_world.Base
        qx, qy, qz, qw = self.tcp_in_world.Rotation.Q
        return {
            "uid": self.uid,
            "name": self.name,
            "mode": self.mode,
            "q_doc": list(self.q_doc),
            "tcp_in_world": [base.x, base.y, base.z,
                             qx, qy, qz, qw],
            "motion": self.motion,
            "speed": self.speed,
            "blend": self.blend,
            "duration": self.duration,
        }

    @staticmethod
    def from_dict(data: Dict) -> "Waypoint":
        """
        in: one WaypointsJson entry
        out: Waypoint (missing key fallbacks to default vals)
        """
        x, y, z, qx, qy, qz, qw = data.get(
            "tcp_in_world", [0, 0, 0, 0, 0, 0, 1])
        return Waypoint(
            uid=data.get("uid") or new_uid(),
            name=data.get("name", ""),
            mode=data.get("mode", JOINT),
            q_doc=list(data.get("q_doc", [])),
            tcp_in_world=App.Placement(App.Vector(x, y, z),
                                       App.Rotation(qx, qy, qz, qw)),
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


@dataclass
class PathSegment:
    """
    polyline of joint step points between two waypoints
    PTP -> 2 pts (plain joint lerp), travel_tcp/rot = 0
    LIN -> IK-solved step points along the cartesian line
    """
    q_step_pts: List[List[float]]   # step points joint vals bw two endpoints

    travel_tcp: float  # straight line tcp motion (mm)
    travel_rot: float  # shortest arc bw endpoints (deg)

    @property
    def travel_joints(self):
        """
        joint travel for display in gui
        """
        return [sum(abs(q_b[j] - q_a[j])
                    for q_a, q_b in zip(self.q_step_pts, self.q_step_pts[1:]))
                for j in range(len(self.q_step_pts[0]))]

    def q_at(self, s):
        s = min(max(s, 0), 1)  # normalised factor ( 0 .. 1)

        # find the idx of sample point bw the
        # two endpoints we are closest to
        sample_pos = s * (len(self.q_step_pts) - 1)
        idx = min(int(sample_pos), len(self.q_step_pts)-2)

        # remaining fraction from the nearest point
        frac = sample_pos - idx

        q_a, q_b = self.q_step_pts[idx], self.q_step_pts[idx+1]
        return [v_a + frac*(v_b-v_a) for v_a, v_b in zip(q_a, q_b)]

    def min_time_possible(self, max_speeds: List[float]) -> tuple[float, int]:
        """
        Min time possible when all joints are
        within limit. \n
        - in: max_speeds - max speed of the rob joints
        - out: (min time possible, joint idx pacing the motion)
        """
        n_steps = len(self.q_step_pts) - 1
        t100, pace_joint = 0, -1
        for q_a, q_b in zip(self.q_step_pts, self.q_step_pts[1:]):
            for j, vmax in enumerate(max_speeds):
                if vmax > 0 and n_steps*abs(q_b[j]-q_a[j])/vmax > t100:
                    t100, pace_joint = n_steps*abs(q_b[j]-q_a[j])/vmax, j
        return t100, pace_joint


@dataclass(frozen=True)
class SpeedSettings:
    """
    Robot speed settings by the user (defaults used otherwise). \n
    Contains:
        - lin_speed: tcp mm/s (lin)
        - ptp_speed: % of max possible robot speed for that segment (ptp)
        - override: global speed scaling (1-100 %)
    """
    lin_speed_default: float = 0.0  # mm/sec  (set in load_speed_settings)
    ptp_speed_default: float = 0.0  # % (set in load_speed_settings)
    override: float = 100.0         # %


@dataclass(frozen=True)
class PlanTiming:
    """
    Feasibility check for the SpeedSettings by user (non-mutable)
    & drives the motion panel status line & play/pause btn states. \n
    Contains:
        - duration: total runtime taken by trajectory plan (sec)
        - ptp_speed: applied speed value (<= 100% (of max speed))
        - feasible: True when the speed/timing is valid
        - min_duration: best possible time when running at max robot speed
        - pace_sec / pace_joint: the slowest joint determining motion time
        - err_msg: error message when the timing is infeasible
    """
    duration: float
    ptp_speed: float
    feasible: bool = True
    min_duration: float = 0.0  # total time at 100% speed
    pace_seg: int = -1
    pace_joint: int = -1  # slowest joint to determine time
    err_msg: str = ""
    bad_segs: tuple = ()  # (segment index, error message)
