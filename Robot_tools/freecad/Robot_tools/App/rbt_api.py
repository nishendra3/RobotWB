"""
rbt_api.py - API for Robot_tools
"""
from __future__ import annotations

from typing import List, Tuple

from freecad.Robot_tools.App import rbt_kine
from freecad.Robot_tools.App.rbt_creator import RobotCreator
from freecad.Robot_tools.App.rbt_errors import (
    RbtDocError, RbtError, RbtInputError)
from freecad.Robot_tools.App.rbt_kine_types import (
    FIXED, PRISMATIC, REVOLUTE)
from freecad.Robot_tools.App.rbt_robot import (
    all_robots, find_robot, is_robot, find_doc)
from freecad.Robot_tools.App.rbt_tool import create_tool, import_shape
from freecad.Robot_tools.App.rbt_traj import (
    add_cartesian_waypoint, create_trajectory, is_trajectory,
    load_waypoints, rbt_trajectories, save_waypoints, teach_waypoint)
from freecad.Robot_tools.App.rbt_traj_plan import (
    check_joint_lims, get_plan, goto_waypoint, sample_tcp_path_world)
from freecad.Robot_tools.App.rbt_traj_types import (
    LIN, PTP, Waypoint)
from freecad.Robot_tools.backends import KIN_LIB_NAMES

__all__ = [
    "get_robot", "get_robots", "RobotApi", "RobotBuilder",
    "TrajectoryApi", "RbtError", "RbtInputError", "RbtDocError",
    "REVOLUTE", "PRISMATIC", "FIXED", "PTP", "LIN", "Waypoint",
    "is_robot", "is_trajectory",
]


def get_robots(doc=None) -> List["RobotApi"]:
    """all robots in the doc (active doc when None)"""
    return [RobotApi(o) for o in all_robots(find_doc(doc))]


def get_robot(hint=None, doc=None) -> "RobotApi":
    """one robot: the sole one, or by Name/Label, or wrap a FPO"""
    return RobotApi(find_robot(hint, doc))


class RobotApi:
    """
    wrapper over Robot_FPO
    """

    def __init__(self, fpo):
        if not is_robot(fpo):
            raise RbtInputError("not a robot FPO")
        self.fpo = fpo

    def __repr__(self):
        return (f"<RobotApi '{self.fpo.Label}' "
                f"joints={self.n_joints}>")

    # ------- structure -------
    @property
    def doc(self):
        return self.fpo.Document

    @property
    def assembly(self):
        return self.fpo.RobotAssembly

    @property
    def n_joints(self) -> int:
        return len(self.fpo.RobotJoints)

    @property
    def joint_types(self) -> List[str]:
        """
        joint types of the kinematic chain
        """
        chain = rbt_kine.get_chain(self.fpo)
        return [j.type for j in chain.joints] if chain else []

    @property
    def dof_mask(self) -> List[bool]:
        return rbt_kine.dof_mask(self.fpo)

    @property
    def limits(self) -> List[Tuple[float, float]]:
        return [rbt_kine.joint_limits_q_deg(self.fpo, i)
                for i in range(self.n_joints)]

    @property
    def max_speeds(self) -> List[float]:
        return rbt_kine.joint_max_speeds(self.fpo)

    # ------- joints -------
    @property
    def q(self) -> List[float]:
        return rbt_kine.curr_joint_vals_doc(self.fpo)

    @q.setter
    def q(self, q_deg):
        rbt_kine.set_q(self.fpo, q_deg)

    def set_q(self, q_deg, clamp=False, preview=False):
        return rbt_kine.set_q(self.fpo, q_deg,
                              clamp=clamp, preview=preview)

    def set_joint(self, j_idx, value, clamp=False, preview=False):
        """single-joint set_q; returns the applied value"""
        q = self.q
        q[j_idx] = float(value)  # IndexError on bad idx
        return self.set_q(q, clamp=clamp, preview=preview)[j_idx]

    def jog(self, q_deg):
        """FK preview only; a recompute reverts it"""
        rbt_kine.set_q(self.fpo, q_deg, preview=True)

    def check_limits(self, q_deg=None) -> str:
        """'' when ok, else the violation text"""
        q = self.q if q_deg is None else q_deg
        return check_joint_lims(self.fpo, q)

    # ------- home / zero -------
    @property
    def home(self) -> List[float]:
        return rbt_kine.home_q_deg(self.fpo)

    def go_home(self, clamp=True):
        return self.set_q(self.home, clamp=clamp)

    def save_home(self):
        rbt_kine.save_home(self.fpo)

    def save_zero(self):
        rbt_kine.set_zero_pose(self.fpo)

    # ------- fk / ik / motion -------
    def fk(self, q_deg=None):
        """
        world TCP for q (current q when None)
        """
        return rbt_kine.fk_tcp_in_world(self.fpo, q_deg)

    @property
    def tcp(self):
        """
        world TCP now (flange when no tool)
        """
        return rbt_kine.fk_tcp_in_world(self.fpo)

    def ik(self, target, q_seed=None,
           pos_tol_mm=0.01, rot_tol_deg=0.5):
        """
        ik solve, None when no solution found
        """
        return rbt_kine.ik_tcp_in_world(
            self.fpo, target, q_seed_deg=q_seed,
            pos_tol_mm=pos_tol_mm, rot_tol_deg=rot_tol_deg)

    def move_to(self, target, q_seed=None, clamp=False,
                preview=False, pos_tol_mm=0.01, rot_tol_deg=0.5):
        """
        ik + move joints
        """
        return rbt_kine.move_to(
            self.fpo, target, q_seed=q_seed, clamp=clamp,
            preview=preview, pos_tol_mm=pos_tol_mm,
            rot_tol_deg=rot_tol_deg)

    # ------- base -------
    @property
    def base(self):
        return self.fpo.BasePlacement

    @base.setter
    def base(self, plc):
        self.fpo.BasePlacement = plc  # onChanged pushes to asm

    # ------- tool -------
    @property
    def tool(self):
        return self.fpo.ActiveTool

    @tool.setter
    def tool(self, tool_fpo):
        if tool_fpo not in list(self.fpo.Tools):
            raise RbtInputError("tool is not in robot.Tools")
        self.fpo.ActiveTool = tool_fpo  # onChanged invalidates

    @property
    def tools(self):
        return list(self.fpo.Tools)

    def add_tool(self, name="Tool", shape_file=None):
        """
        add a new active tool
        """
        tool_fpo = create_tool(self.fpo, name)
        if shape_file:
            tool_fpo.Tool_shape = import_shape(self.fpo, shape_file)
        return tool_fpo

    # ------- kinematics lib -------
    @property
    def kinematics_lib(self) -> str:
        return self.fpo.KinematicsLib

    @kinematics_lib.setter
    def kinematics_lib(self, name):
        if name not in KIN_LIB_NAMES:
            raise RbtInputError(
                f"unknown lib, use one of {KIN_LIB_NAMES}")
        self.fpo.KinematicsLib = name  # onChanged invalidates

    # ------- trajectories -------
    @property
    def trajectories(self) -> List["TrajectoryApi"]:
        return [TrajectoryApi(t)
                for t in rbt_trajectories(self.fpo)]

    def add_trajectory(self) -> "TrajectoryApi":
        return TrajectoryApi(create_trajectory(self.fpo))

    # ------- escape hatches -------
    def recompute(self):
        """
        recompute assembly & tool docs
        """
        rbt_kine.recompute_asm(self.fpo)
        rbt_kine.recompute_tool(self.fpo)

    def invalidate_cache(self):
        """force chain/backend rebuild"""
        rbt_kine.invalidate(self.fpo)


class TrajectoryApi:
    """
    wrapper around Trajectory FPO
    """

    def __init__(self, fpo):
        if not is_trajectory(fpo):
            raise RbtInputError("not a trajectory FPO")
        self.fpo = fpo

    def __repr__(self):
        return (f"<TrajectoryApi '{self.fpo.Label}' "
                f"wps={len(self.waypoints)}>")

    @property
    def robot(self) -> "RobotApi":
        return RobotApi(self.fpo.Robot)

    @property
    def waypoints(self) -> List[Waypoint]:
        return load_waypoints(self.fpo)

    @waypoints.setter
    def waypoints(self, wps):
        save_waypoints(self.fpo, wps)  # onChanged drops plan_cache

    def teach(self, name="", motion=PTP) -> Waypoint:
        """
        record the current pose as a JOINT waypoint
        """
        return teach_waypoint(self.fpo, name, motion)

    def add_cartesian(self, target, name="", motion=PTP):
        """
        CARTESIAN waypoint at target
        """
        return add_cartesian_waypoint(self.fpo, target, name, motion)

    def plan(self):
        """
        get trajectory plan & validated waypoints
        """
        return get_plan(self.fpo)

    @property
    def duration(self):
        plan = self.plan()[0]
        return plan.timing.duration if plan else None

    def q_at(self, t_sec):
        """
        q at time t along the plan
        """
        plan = self.plan()[0]
        return plan.q_at_time(t_sec) if plan else None

    def goto(self, idx, clamp=False, preview=False):
        """
        move the robot to waypoint idx
        """
        return goto_waypoint(self.fpo, idx,
                             clamp=clamp, preview=preview)

    def sample_path(self, n_per_seg=20):
        """TCP polyline per segment (world); [] on plan failure"""
        plan = self.plan()[0]
        if plan is None:
            return []
        return sample_tcp_path_world(self.fpo.Robot, plan,
                                     n_per_seg)


class RobotBuilder:
    """
    DefineRobot: parts -> base -> joints -> finish
    """

    def __init__(self, doc=None):
        self.doc = find_doc(doc)
        self.creator = RobotCreator()
        self.creator.asm_doc = self.doc

    def new_robot(self) -> "RobotBuilder":
        """RobotAssembly + Robot_FPO into the doc"""
        self.creator.build_assembly(self.doc)
        return self

    def attach(self, hint=None) -> "RobotBuilder":
        """bind to an existing robot for edits"""
        if self.creator.resolve(hint) is None:
            raise RbtDocError("no robot assembly found")
        return self

    @property
    def robot(self) -> "RobotApi":
        return RobotApi(self.creator.fpo)

    def import_parts(self, path) -> list:
        """
        merge a parts .FCStd
        """
        return self.creator.import_parts_file(path)

    def add_parts(self, objs) -> list:
        """
        App::Links into the assembly, build order
        """
        return self.creator.insert_parts(objs)

    def add_part(self, obj):
        return self.creator.insert_parts([obj])[0]

    def add_base(self, ref, jtype=FIXED, label=""):
        """
        ref=(link, "FaceN")
        FIXED | REVOLUTE (turntable) | PRISMATIC (rail)
        """
        return self.creator.insert_base(jtype, ref, label)

    def add_joint(self, jtype, ref1, ref2, label=""):
        """
        refs=(link, "FaceN"), parent side first
        """
        return self.creator.insert_joint(jtype, [ref1, ref2],
                                         label)

    def flip_joint(self, joint):
        self.creator.flip_joint(joint)

    def delete_joint(self, joint):
        self.creator.delete_joint(joint)

    def remove_link(self, link):
        self.creator.remove_link(link)

    def finish(self) -> "RobotApi":
        """
        validate, seed BasePlacement, recompute
        """
        return RobotApi(self.creator.finalize())
