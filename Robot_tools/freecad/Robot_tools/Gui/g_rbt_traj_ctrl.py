"""
g_rbt_traj_ctrl.py
controller for trajectory editing & playback
"""
from __future__ import annotations

from typing import List, Optional, Iterator
from contextlib import contextmanager

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App import rbt_kine, rbt_traj, rbt_traj_plan
from freecad.Robot_tools.App.rbt_traj_types import (
    CARTESIAN, DocObj, JOINT, Placement, TargetMode, MotionType,
    V3, Waypoint, new_uid)


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

    def commit(self, label: str, wps: List[Waypoint]) -> None:
        """
        save the created waypoints all at once
        in: label for undoing operation, waypoints
        """
        doc = self.traj.Document
        doc.openTransaction(label)
        try:
            rbt_traj.save_waypoints(self.traj, wps)
            doc.commitTransaction()
        except Exception:
            doc.abortTransaction()
            raise

    def add_wp_at_pose(self, tcp_in_world: Placement,
                       name: str = "") -> Optional[Waypoint]:
        """
        Add catesian wp at the given pose
        in: tcp pose in asm coords, optional label
        out: appended waypoint, None when ik fails to find robot joints
        """
        q = rbt_kine.ik_tcp_in_world(self.robot,
                                     tcp_in_world)
        if q is None:
            return None
        wps = self.wps
        wp = Waypoint(new_uid(), name or self.next_name(wps),
                      CARTESIAN, q, tcp_in_world)
        wps.append(wp)
        self.commit("Add waypoint", wps)
        return wp

    def teach_wp(self, name: str = "") -> Waypoint:
        """
        capture the robot's current pose as a JOINT waypoint
        in: optional user label ("" -> next free P<n>)
        out: the appended waypoint
        """
        q = rbt_kine.curr_joint_vals_doc(self.robot)
        pose = rbt_kine.fk_tcp_in_world(self.robot, q) or App.Placement()
        wps = self.wps
        wp = Waypoint(new_uid(), name or self.next_name(wps),
                      JOINT, q, pose)
        wps.append(wp)
        self.commit("Teach waypoint", wps)
        return wp

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
    def edit_wps(self, undo_label: str) -> Iterator[List[Waypoint]]:
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
            wps[i].motion = motion

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
        solved = rbt_traj_plan.solve_waypoints(
            self.robot, [self.wps[wp_idx]])[0]
        if solved.q_doc is None:
            return False
        rbt_kine.jog_q_deg(self.robot, solved.q_doc)
        rbt_kine.resolve_offsets(self.robot, solved.q_doc)
        return True

    # ---------- helpers ----------

    def next_name(self, wps: List[Waypoint]) -> str:
        """
        first unused P<n>  for waypoint
        """
        used = {wp.name for wp in wps}
        i = len(wps) + 1
        while f"P{i}" in used:
            i += 1
        return f"P{i}"

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

    def set_prop(self, prop: str, value: float) -> None:
        """
        write trajectory property with an undo transaction
        """
        if getattr(self.traj, prop) == value:
            return

        undo_name = "changed " + prop

        doc = self.traj.Document
        doc.openTransaction(undo_name)
        try:
            setattr(self.traj, prop, value)
        finally:
            doc.commitTransaction()
