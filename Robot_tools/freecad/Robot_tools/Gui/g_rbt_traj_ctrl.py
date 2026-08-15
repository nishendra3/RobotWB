"""
g_rbt_traj_ctrl.py
controller for trajectory editing & playback
"""
from __future__ import annotations

from typing import List, Optional, Generator
from contextlib import contextmanager

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App import rbt_kine, rbt_traj, rbt_traj_plan
from freecad.Robot_tools.App.rbt_traj_types import (
    DocObj, Placement, TargetMode, MotionType,
    V3, Waypoint)


class TrajectoryController:
    """
    Controller for the trajectory panel GUI
    Contains:
        - traj: trajectory fpo
        - robot: robot fpo
    """

    def __init__(self, traj_obj: DocObj) -> None:
        self.traj = traj_obj
        self.robot = traj_obj.Robot

    # ---------- reads ----------

    @property
    def wps(self) -> List[Waypoint]:
        """
        read the waypoints from doc itself
        """
        return rbt_traj.load_waypoints(self.traj)

    def get_plan(self):
        """
        out: (plan | None, solved list for the table rows)
        """
        plan, solved, _ = rbt_traj_plan.get_plan(self.traj)
        return plan, solved

    # ---------- mutations ----------

    @contextmanager
    def openAndCommitTransaction(self,
                                 label: str) -> Generator[None, None, None]:
        doc = self.traj.Document
        doc.openTransaction(label)
        try:
            yield
            doc.commitTransaction()
        except Exception:
            doc.abortTransaction()
            raise

    def commit(self, label: str, wps: List[Waypoint]) -> None:
        """
        save the created waypoints all at once
        in: label for undoing operation, waypoints
        """
        with self.openAndCommitTransaction(label):
            rbt_traj.save_waypoints(self.traj, wps)

    def add_wp_at_pose(self, tcp_in_world: Placement,
                       name: str = "") -> Optional[Waypoint]:
        """
        Add catesian wp at the given pose
        """
        with self.openAndCommitTransaction("Add waypoint"):
            return rbt_traj.add_cartesian_waypoint(self.traj,
                                                   tcp_in_world, name)

    def teach_wp(self, name: str = "") -> Waypoint:
        """
        capture the robot's current pose as a JOINT waypoint
        """
        with self.openAndCommitTransaction("Teach waypoint"):
            return rbt_traj.teach_waypoint(self.traj, name)

    def update_wp(self, i: int) -> None:
        """
        re-teach the selected waypoint from the current pose
        """
        with self.edit_wps("Update waypoint") as wps:
            q = rbt_kine.curr_joint_vals_doc(self.robot)
            pose = rbt_kine.fk_tcp_in_world(self.robot, q) or App.Placement()
            wps[i].q_doc = q
            wps[i].tcp_in_world = pose

    @contextmanager
    def edit_wps(self, undo_label: str) -> Generator[List[Waypoint],
                                                     None, None]:
        wps = self.wps
        yield wps
        self.commit(undo_label, wps)

    def rename_wp(self, i: int, name: str) -> None:
        with self.edit_wps("Rename waypoint") as wps:
            wps[i].name = name

    def set_wp_mode(self, i: int, mode: TargetMode) -> None:
        with self.edit_wps("Change waypoint mode") as wps:
            wps[i].mode = mode

    def set_wp_motion(self, i: int, motion: MotionType) -> None:
        with self.edit_wps("Change waypoint motion") as wps:
            # set motion type PTP/LIN
            wps[i].motion = motion

            # set correct default speed for that motion
            if wps[i].duration is None:  # not fix time-based motion
                wps[i].speed = rbt_kine.default_speed(self.robot, motion)

    def set_wp_pace(self, i: int, value: Optional[float],
                    by_duration: bool) -> None:
        """
        set the pace (speed or duration) to reach the point
        by_duration -> time (seconds) else -> speed value (mm/s or %)
        """
        with self.edit_wps("Change waypoint pace") as wps:
            wps[i].speed = None if by_duration else value
            wps[i].duration = value if by_duration else None

    def move_wp(self, i: int, delta: int) -> None:
        """
        in: row index, delta -1 (up) / +1 (down)
        """
        j = i + delta
        if not (0 <= i < len(self.wps) and 0 <= j < len(self.wps)):
            return
        with self.edit_wps("Reorder waypoints") as wps:
            wps[i], wps[j] = wps[j], wps[i]

    def delete_wp(self, i: int) -> None:
        with self.edit_wps("Delete waypoint") as wps:
            wps.pop(i)

    # ---------- robot motion ----------

    def goto_wp(self, wp_idx: int) -> bool:
        """
        move the robot to one waypoint
        out: False when the point is unreachable
        """
        return rbt_traj_plan.goto_waypoint(self.traj,
                                           wp_idx) is not None
    # ---------- helpers ----------

    def pose_from_world_point(self, point_world: V3) -> Placement:
        """
        picked 3d point + current tcp orientation -> asm pose
        in: picked point in world coords
        out: tcp pose in asm coords
        """
        q = rbt_kine.curr_joint_vals_doc(self.robot)
        tcp_in_world = (rbt_kine.fk_tcp_in_world(self.robot, q) or
                        App.Placement())
        return App.Placement(point_world, tcp_in_world.Rotation)
