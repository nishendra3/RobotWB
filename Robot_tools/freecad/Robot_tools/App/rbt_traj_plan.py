"""
rbt_traj_plan.py
build, time and make a travel plan from waypoints
"""
from __future__ import annotations

import math

from typing import (
    Callable, List, Optional, Tuple, TypeAlias)

import FreeCAD as App  # type: ignore


from freecad.Robot_tools.App import rbt_kine, rbt_traj
from freecad.Robot_tools.App.rbt_traj_profile import TimeProfile, make_profile
from freecad.Robot_tools.App.rbt_traj_types import (
    LIN, JOINT, CARTESIAN, PathSegment,
    SolvedWaypoint, SpeedSettings, PlanTiming, Waypoint, DocObj)
from freecad.Robot_tools.App.rbt_global_constants import (
    LIN_IK_STEP_DEG, LIN_IK_STEP_MM, LIN_IK_STEPS_MAX,
    DEFAULT_LIN_SPEED_ORI, LIM_EPS)
from freecad.Robot_tools.App.rbt_helpers_math import lerp_plc, rot_delta_deg

V3: TypeAlias = App.Vector
Placement: TypeAlias = App.Placement

FULL_SPEED_PCT = 100
MIN_SPEED_PCT = 1e-3
SPEED_EPS_PCT = 1E-3


class TrajectoryPlan:
    """
    Time-parameterised motion over full waypoint list
    Contains:
        - segments/profiles: one per waypoint pair
        - wp_start_times: time each waypoint is reached
            len = len(segments) + 1, [0th] = 0
        - duration: total run time, seconds
        - timing: PlanTiming for this plan
    """

    def __init__(self, segments: List[PathSegment],
                 profiles: List[TimeProfile], timing: PlanTiming
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
    n_jnts = len(rbt_obj.RobotJoints)
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
            q = rbt_kine.ik_tcp_in_world(rbt_obj,
                                         wp.tcp_in_world,
                                         q_seed_deg=list(wp.q_doc) or None)
            err = "" if q else f"'{wp.name}': point cannot be reached"
            s_wp = SolvedWaypoint(wp, q, err)
            solved.append(s_wp)

    return solved


def goto_waypoint(traj_obj, wp_idx: int, clamp: bool = False,
                  preview: bool = False) -> Optional[List[float]]:
    """
    move the robot to waypoint wp_idx
    """
    wps = rbt_traj.load_waypoints(traj_obj)
    solved = solve_waypoints(traj_obj.Robot, [wps[wp_idx]])
    q = solved[0].q_doc if solved else None
    if q is None:
        return None
    return rbt_kine.set_q(traj_obj.Robot, q, clamp=clamp,
                          preview=preview)


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


def build_plan(rbt_obj: DocObj,
               wps: List[Waypoint],
               speed_settings: SpeedSettings,
               motion_profile: Callable = make_profile
               ) -> Tuple[Optional[TrajectoryPlan],
                          List[SolvedWaypoint]]:
    """
    PTP: base duration = (time at 100% joint speed) / (ptp speed % / 100)
    LIN: base duration = max(
                (tcp_travel/lin_speed), -> user spec. linear speed
                (rot_travel/LIN_ORIENTATION_SPEED), -> limit to tool rot
                (joint limits), -> time below which joints exceed max limits
                )

    override % -> scales down all motion (PTP or LIN)

    per point, the solver priority is as follows:
    1. User specified time bw two points
    2. User specified speed bw two points (ptp/lin)
    3. Default speeds

    in: robot_fpo, waypoints, speed settings, motion profile
    out: (plan, solved) -> None for <2 waypoints
    """
    EPS = 1e-6
    solved = solve_waypoints(rbt_obj, wps)
    if len(solved) < 2 or any(sw.q_doc is None for sw in solved):
        return None, solved

    # get how to arrive at destination wp : LIN/PTP
    segments: List[PathSegment] = []
    for i, (sw_from, sw_to) in enumerate(zip(solved, solved[1:])):
        if sw_to.wp.motion == LIN:
            seg, err = lin_build_segment(rbt_obj, sw_from.q_doc,
                                         sw_to.wp, sw_to.q_doc)

            # if faulty segment, replace with a copy contating error
            if seg is None:
                solved[i+1] = SolvedWaypoint(sw_to.wp, sw_to.q_doc, err)
                return None, solved
        else:  # sw_to.wp.motion == PTP:
            seg = PathSegment([sw_from.q_doc, sw_to.q_doc], 0.0, 0.0)

        segments.append(seg)

    # per segment time requirement
    max_speeds = rbt_kine.joint_max_speeds(rbt_obj)
    seg_t, seg_t_min, pacer_joint_idxs = [], [], []

    for seg, sw_to in zip(segments, solved[1:]):
        wp = sw_to.wp
        t_min, pacer_joint_idx = seg.min_time_possible(max_speeds)

        # case: user provided time duration
        if wp.duration and wp.duration > 0:
            t = wp.duration

        # case: user provided linear motion speed
        # (use default linear speed otherwise)
        elif wp.motion == LIN:
            v_tcp = wp.speed or speed_settings.lin_speed_default
            t = max(seg.travel_tcp/max(v_tcp, EPS),
                    seg.travel_rot/DEFAULT_LIN_SPEED_ORI)

        # case: user provided ptp motion speed
        # (use default joint speed otherwise)
        else:  # wp.motion == PTP
            pct = wp.speed or speed_settings.ptp_speed_default
            t = t_min / (max(pct, MIN_SPEED_PCT) / FULL_SPEED_PCT)

        seg_t.append(t)
        seg_t_min.append(t_min)
        pacer_joint_idxs.append(pacer_joint_idx)

    # global override
    override = min(max(speed_settings.override, MIN_SPEED_PCT), FULL_SPEED_PCT)
    durations = [t / (override/FULL_SPEED_PCT) for t in seg_t]

    report = check_timing_limits(durations, seg_t_min,
                                 pacer_joint_idxs, solved, override)
    profiles = [motion_profile(d) for d in durations]

    return TrajectoryPlan(segments, profiles, report), solved


def check_timing_limits(durations: List[float], seg_t_min: List[float],
                        pacer_joint_idxs: List[int],
                        solved: List[SolvedWaypoint],
                        override: float) -> PlanTiming:
    """
    Check if no joint is exceeding its specified speed limit.
    """
    total = sum(durations)
    min_duration = sum(seg_t_min)

    # bad segments error
    def seg_err(i):
        wp = solved[i+1].wp
        return (f"'{wp.name}': J{pacer_joint_idxs[i]+1} over max speed, "
                f"segment needs >= {seg_t_min[i]:.2f} s, "
                f"planned {durations[i]:.2f} s "
                f"(override {override:.0f}%)")

    # find segments in violation of joint max speed limits
    bad_segs = tuple((i, seg_err(i)) for i, (duration, min_allowed)
                     in enumerate(zip(durations, seg_t_min))
                     if duration < min_allowed*(1 - LIM_EPS))
    if bad_segs:
        pace_seg_idx, err = bad_segs[0]
    else:
        err = ""
        pace_seg_idx = durations.index(max(durations)) if total > 0 else -1

    # get the pace determining joint for violating segment
    pace_joint_idx = (pacer_joint_idxs[pace_seg_idx]
                      if pace_seg_idx >= 0 else -1)

    return PlanTiming(total, override, not bad_segs, min_duration,
                      pace_seg_idx, pace_joint_idx, err, bad_segs)


def sample_tcp_path_world(rbt_obj: DocObj,
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
            plc = rbt_kine.fk_tcp_in_world(rbt_obj, seg.q_at(k / n))
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
    plan, solved = build_plan(traj_obj.Robot, wps,
                              rbt_kine.load_speed_settings(traj_obj.Robot))
    if plan is None:
        msg = "needs 2+ waypoints" if len(wps) < 2 else "IK failure"
        return None, solved, msg
    return plan, solved, "" if plan.timing.feasible else plan.timing.err_msg


def get_plan(traj_obj: DocObj) -> PlanResult:
    """
    cached trajectory plan, reset on property change
    """
    proxy = traj_obj.Proxy
    if getattr(proxy, "plan_cache", None) is None:
        proxy.plan_cache = make_plan(traj_obj)
    return proxy.plan_cache


# ------- Linear Trajectroy Points ------------

def lin_endpoint_poses(rbt_obj, q_from: List[float],
                       wp_to: Waypoint, q_to: List[float]
                       ) -> Optional[Tuple[Placement, Placement]]:
    """
    get the start and endpose of the line
    """
    plc_from = rbt_kine.fk_tcp_in_world(rbt_obj, q_from)  # start pose
    plc_to = (wp_to.tcp_in_world                          # end pose
              if (wp_to.mode == CARTESIAN)
              else rbt_kine.fk_tcp_in_world(rbt_obj, q_to))

    if (plc_from is None
            or plc_to is None):
        return None
    return plc_from, plc_to


def lin_step_count(travel_mm: float, travel_deg: float) -> int:
    """
    number of IK solves needer for the given travel segment
    """
    steps = max(travel_mm / LIN_IK_STEP_MM,
                travel_deg / LIN_IK_STEP_DEG,)

    return max(1, min(LIN_IK_STEPS_MAX, math.ceil(steps)))


def lin_step_pts_ik(rbt_obj, plc_from: List[float],
                    plc_to: List[float], n_steps: int,
                    q_from: List[float], q_to: List[float]):
    q_step_pts = [list(q_from)]  # add start pt
    for step in range(1, n_steps):
        q_sol = rbt_kine.ik_tcp_in_world(
            rbt_obj, lerp_plc(plc_from, plc_to, step / n_steps),
            q_seed_deg=q_step_pts[-1])
        if q_sol is None:
            return None, f"lin: unreachable after {100*step//n_steps}%"
        q_step_pts.append(q_sol)
    q_step_pts.append(list(q_to))  # add ending pt
    return q_step_pts, ""


# TODO
# def lin_config_jump_check(rbt_obj, q_step_pts):
#     # todo: implement this
#     # check & stop configuration flips when
#     # linear motion is happening
#     pass

def lin_build_segment(rbt_obj, q_from: List[float], wp_to: Waypoint,
                      q_to: List[float]) -> Tuple[Optional[PathSegment], str]:
    """

    """
    poses = lin_endpoint_poses(rbt_obj, q_from, wp_to, q_to)
    if poses is None:
        return None, "lin: FK failed"
    plc_from, plc_to = poses

    travel_mm = (plc_to.Base - plc_from.Base).Length
    travel_deg = rot_delta_deg(plc_from.Rotation, plc_to.Rotation)
    n_steps = lin_step_count(travel_mm, travel_deg)

    q_step_pts, err = lin_step_pts_ik(rbt_obj, plc_from, plc_to,
                                      n_steps, q_from, q_to)

    if q_step_pts is None:
        return None, err

    # skip if IK leads robot through config
    # changes or singularities. TODO
    # err = lin_config_jump_check(rbt_obj, q_step_pts)
    # if err:
    #     return None, err

    return PathSegment(q_step_pts, travel_mm, travel_deg), ""
