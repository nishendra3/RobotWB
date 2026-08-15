"""
g_rbt_traj_vp.py
view provider for the trajectory object
"""
import os
from typing import Dict, List, Optional

from pivy import coin  # type: ignore

from freecad.Robot_tools import rbt_locator
from freecad.Robot_tools.App import rbt_traj_plan, rbt_kine
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_global_constants import ap_clr
from freecad.Robot_tools.App.rbt_traj_types import (
    DocObj, SolvedWaypoint, V3)
from freecad.Robot_tools.App.rbt_traj_plan import TrajectoryPlan
from freecad.Robot_tools.Gui import so_helpers as so

# trajectory colors
COLOR_PATH = ap_clr["V_Orange"][0]  # tcp path
COLOR_WP = ap_clr["B_Yellow"][0]  # default waypoint
COLOR_WP_SEL = ap_clr["B_Green"][0]  # selected waypoint

# trajectroy visual dimensions
PATH_WIDTH = 2.0
WP_RADIUS_MM = 6.0


class ViewProviderTrajectory:
    """
    Draws the FK sampled TCP paths and markers per waypoint
    Scenegraph:
        root -> [path_sep, marker_sep]   (world coords)
    """

    def __init__(self, vobj):
        vobj.Proxy = self

    def dumps(self): return None
    def loads(self, state): return None

    def getDisplayModes(self, vobj): return ["Standard"]
    def getDefaultDisplayMode(self): return "Standard"

    def attach(self, vobj) -> None:
        self.vobj = vobj
        self.Object = vobj.Object

        # path & wp marker separators
        self.path_sep = coin.SoSeparator()
        self.marker_sep = coin.SoSeparator()

        # gui nodes for waypoints
        self.wp_mats: Dict[str, coin.SoMaterial] = {}
        self.wp_seps: Dict[str, coin.SoSeparator] = {}

        root = so.sep(self.path_sep, self.marker_sep)
        vobj.addDisplayMode(root, "Standard")

        self.resample(vobj.Object)

    def updateData(self, fp: DocObj, prop: str) -> None:
        """
        resample when waypoints, num of prev samples, robot change
        """
        if prop in ("WaypointsJson", "PreviewSamples"):
            self.resample(fp)
        elif prop == "Robot":
            self.resample(fp)

    def resample(self, fp: DocObj) -> None:
        """
        rebuild path polylines + waypoint markers from the doc
        """
        self.clear_display()
        robot = fp.Robot
        if robot is None:
            return
        try:
            plan, solved, _ = rbt_traj_plan.get_plan(fp)
        except Exception as e:   # chain not ready
            fcl_warn(f"{fp.Name}: path visualisation skipped - {e}\n")
            return
        if not solved:
            return
        self.build_markers(robot, solved)
        self.build_path(fp, robot, plan)

    def clear_display(self) -> None:
        self.path_sep.removeAllChildren()
        self.marker_sep.removeAllChildren()
        self.wp_mats.clear()
        self.wp_seps.clear()

    def marker_pos(self, robot: DocObj,
                   solved_wp: SolvedWaypoint) -> V3:
        """
        actual reachable point: FK of the solved q,
        taught pose when unsolved
        """
        if solved_wp.q_doc is not None:
            plc = rbt_kine.fk_tcp_in_world(robot, solved_wp.q_doc)
            if plc is not None:
                return plc.Base
        return solved_wp.wp.tcp_in_world.Base

    def make_marker(self, robot: DocObj,
                    solved_wp: SolvedWaypoint) -> coin.SoSeparator:
        mat = so.material(COLOR_WP)
        sphere = coin.SoSphere()
        sphere.radius.setValue(WP_RADIUS_MM)
        b = self.marker_pos(robot, solved_wp)
        mk = so.sep(so.transform((b.x, b.y, b.z)), mat, sphere)
        self.wp_mats[solved_wp.wp.uid] = mat
        self.wp_seps[solved_wp.wp.uid] = mk
        return mk

    def build_markers(self, robot: DocObj,
                      solved: List[SolvedWaypoint]) -> None:
        for solved_wp in solved:
            self.marker_sep.addChild(self.make_marker(robot, solved_wp))

    def build_path(self, fp: DocObj, robot: DocObj,
                   plan: Optional[TrajectoryPlan]) -> None:
        """
        build tcp path needs if the points are valid
        """
        if plan is None:
            return
        path_points = rbt_traj_plan.sample_tcp_path_world(
            robot, plan, fp.PreviewSamples)
        style = coin.SoDrawStyle()
        style.lineWidth = PATH_WIDTH
        for pts in path_points:
            self.path_sep.addChild(
                so.polyline([(p.x, p.y, p.z) for p in pts],
                            style, so.color(COLOR_PATH)))

    def highlight_wp(self, uid: str) -> None:
        """
        in: waypoint uid
        recolor the marker for panel row sync
        """
        for wp_uid, mat in self.wp_mats.items():
            clr = COLOR_WP_SEL if wp_uid == uid else COLOR_WP
            mat.diffuseColor.setValue(*clr)

    def doubleClicked(self, vobj) -> bool:
        from freecad.Robot_tools.Gui import g_rbt_traj_tpanel
        g_rbt_traj_tpanel.run(vobj.Object)
        return True

    def getIcon(self) -> str:
        return os.path.join(os.path.dirname(rbt_locator.__file__),
                            "resources/icons/rbt_trajectory.svg")
