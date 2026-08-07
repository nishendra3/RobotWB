"""
rbt_traj_plan.py
build, time and make a travel plan from waypoints
"""
from __future__ import annotations

from typing import (
    Callable, List, Optional, Tuple, TypeAlias)

import FreeCAD as App  # type: ignore


from freecad.Robot_tools.App import rbt_kine, rbt_traj
from freecad.Robot_tools.App.rbt_traj_profile import TimeProfile, make_profile
from freecad.Robot_tools.App.rbt_traj_types import (
    BY_DURATION, JOINT, MotionSegment, PtpSegment, SolvedWaypoint,
    TimingReport, TimingRequest, Waypoint, DocObj
)

V3: TypeAlias = App.Vector

FULL_SPEED_PCT = 100
MIN_SPEED_PCT = 1e-3
SPEED_EPS_PCT = 1E-3

LIM_EPS = 1e-6


class TrajectoryPlan:
    """
    Time-parameterised motion over full waypoint list
    Contains:
        - segments/profiles: one per waypoint pair
        - wp_start_times: time each waypoint is reached
            len = len(segments) + 1, [0th] = 0
        - duration: total run time, seconds
        - timing: TimingReport for this plan
    """

    def __init__(self, segments: List[MotionSegment],
                 profiles: List[TimeProfile], timing: TimingReport
                 ) -> None:
        self.segments = segments
        self.profiles = profiles
        self.timing = timing
        # assuming first waypoint is start pt at t = 0 sec
        self.wp_start_times: List[float] = [0.0]
        for prf in profiles:
            self.wp_start_times.append(
                self.wp_start_times[-1] + prf.duration)
        self.duration: float = self.wp_start_times[-1]  # total runtime

    def seg_idx_at_time(self, t_sec: float) -> int:
        """
        in: elapsed seconds
        out: index of the active segment
        """
        # end time of segment i -> wp_start_times[i+1]
        for i in range(len(self.segments)):
            if t_sec < self.wp_start_times[i+1]:
                return i

        # (time elapsed > total runtime) -> robot at last wp
        return len(self.segments) - 1

    def q_at_time(self, t_sec: float) -> List[float]:
        """
        in: elapsed seconds
        out: joint values in doc units
        """
        i = self.seg_idx_at_time(t_sec)
        s = self.profiles[i].s_at(t_sec - self.wp_start_times[i])
        return self.segments[i].q_at(s)


def solve_waypoints(rbt_obj: DocObj,
                    wps: List[Waypoint]) -> List[SolvedWaypoint]:
    """
    Turn each waypoint into required joint values to reach it
    joint -> stored directly (verified with joint limits)
    cartesian -> converted to joints using IK

    in: robot fpo, waypoints
    out: solved & valid waypoint per input
    """
    n_jnts = len(rbt_obj.Robot_joints)
    solved: List[SolvedWaypoint] = []

    for wp in wps:
        if wp.mode == JOINT:
            q = list(wp.q_doc)
            if len(q) != n_jnts:
                s_wp = SolvedWaypoint(wp, None,
                                      f"'{wp.name}': joint count changed"
                                      f"({len(q)} stored, robot has {n_jnts})")
                solved.append(s_wp)
                continue

            err = check_joint_lims(rbt_obj, q)
            s_wp = SolvedWaypoint(wp, None if err else q, err)
            solved.append(s_wp)
        else:
            # ik solves in world pose, we are storing in asm coords
            q = rbt_kine.ik_tcp_in_asm(rbt_obj, wp.tcp_in_asm,
                                       q_seed_deg=list(wp.q_doc) or None)
            err = "" if q else f"'{wp.name}': point cannot be reached"
            s_wp = SolvedWaypoint(wp, q, err)
            solved.append(s_wp)

    return solved


def check_joint_lims(rbt_obj, q_doc: List[float]) -> str:
    """
    in: robot fpo, joint values in doc units
    out: "" when inside limits, else error msg mentioning joint
    """
    for j, val in enumerate(q_doc):
        low, high = rbt_kine.joint_limits_q_deg(rbt_obj, j)
        if val < low - LIM_EPS or val > high + LIM_EPS:
            return (f"J{j + 1} at {val:.1f} outside "
                    f"[{low:.1f}, {high:.1f}]")
    return ""


def seg_time_full_speed(seg: MotionSegment,
                        max_speeds: List[float]) -> Tuple[float, int]:
    """
    Time the segment needs at 100% of individual joint speeds
    it is limited by the joint that is slowest to arrive
    in: segment, per-joint max speeds (deg/s (revolute) | mm/s (prismatic))
    out: time in sec (at 100% speed, pacing joint idx)
    """
    t100, pacer = 0.0, -1
    for j, (dq, vmax) in enumerate(zip(seg.travel_joints,
                                       max_speeds)):
        if vmax <= 0.0:
            continue
        t = dq / vmax
        if t > t100:
            t100, pacer = t, j
    return t100, pacer


def build_plan(rbt_obj: DocObj,
               wps: List[Waypoint],
               timing: TimingRequest,
               profile_factory: Callable = make_profile
               ) -> Tuple[Optional[TrajectoryPlan],
                          List[SolvedWaypoint]]:
    """
    segment duration = (time at 100%) / (speed% /100)

    BY_SPEED -> durations from timing.speed
    BY_DURATION -> calc. speeds to meet the target duration
    (if duration is not reachable, still return for user info)

    in: robot_fpo, waypoints, timing request, motion profile
    out: (plan, solved) -> None for <2 waypoints
    """
    solved = solve_waypoints(rbt_obj, wps)
    if len(solved) < 2 or any(sw.q_doc is None for sw in solved):
        return None, solved

    segments: List[MotionSegment] = [
        PtpSegment(a.q_doc, b.q_doc)
        for a, b in zip(solved, solved[1:])]

    max_speeds = rbt_kine.joint_max_speeds(rbt_obj)
    seg_t100: List[float] = []
    seg_pace: List[int] = []

    for seg in segments:
        t100, pace = seg_time_full_speed(seg, max_speeds)
        seg_t100.append(t100)
        seg_pace.append(pace)

    min_duration = sum(seg_t100)
    pace_seg_idx = seg_t100.index(max(seg_t100)) if min_duration > 0 else -1
    pace_joint = seg_pace[pace_seg_idx] if pace_seg_idx >= 0 else -1

    if timing.mode == BY_DURATION and timing.duration > 0.0:
        speed_pct = FULL_SPEED_PCT * min_duration / timing.duration
    else:
        speed_pct = timing.speed

    # clamp negative and null speeds
    speed_pct = max(speed_pct, MIN_SPEED_PCT)

    durations = [t / (speed_pct/FULL_SPEED_PCT) for t in seg_t100]
    total = sum(durations)

    feasible = speed_pct <= FULL_SPEED_PCT + SPEED_EPS_PCT

    err = "" if feasible else f"min time possible is {min_duration:.2f}s"

    report = TimingReport(total, speed_pct, feasible,
                          min_duration, pace_seg_idx,
                          pace_joint, err)

    profiles = [profile_factory(d) for d in durations]
    return TrajectoryPlan(segments, profiles, report), solved


def sample_tcp_path_asm(rbt_obj: DocObj,
                        plan: TrajectoryPlan,
                        n_per_seg: int) -> List[List[V3]]:
    """
    FK sample the true tcp path for view provider display
    in: robot_fpo, plan, samples per segment
    out: one point list per segment, robot-asm coords
    """
    n = max(n_per_seg, 2)
    paths: List[List[V3]] = []
    for seg in plan.segments:
        pts: List[V3] = []
        for k in range(n + 1):
            plc = rbt_kine.fk_tcp_in_asm(rbt_obj, seg.q_at(k / n))
            if plc is None:
                return []
            pts.append(plc.Base)
        paths.append(pts)
    return paths


PlanResult: TypeAlias = Tuple[Optional[TrajectoryPlan],
                              List[SolvedWaypoint], str]


def make_plan(traj_obj: DocObj) -> PlanResult:
    """
    make a trajectory plan
    out: plan | None
    """
    wps = rbt_traj.load_waypoints(traj_obj)
    if len(wps) < 2:
        return None, [], "needs 2+ waypoints"
    plan, solved = build_plan(traj_obj.Robot, wps,
                              rbt_traj.load_timing(traj_obj))
    if plan is None:
        return None, solved, "IK failure"
    return plan, solved, "" if plan.timing.feasible else plan.timing.err_msg


def get_plan(traj_obj: DocObj) -> PlanResult:
    """
    cached trajectory plan, reset on property change
    """
    proxy = traj_obj.Proxy
    if getattr(proxy, "plan_cache", None) is None:
        proxy.plan_cache = make_plan(traj_obj)
    return proxy.plan_cache
