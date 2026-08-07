"""
g_rbt_traj_vp.py
view provider for the trajectory object
"""
import os
from typing import List, Dict

from pivy import coin  # type: ignore

from freecad.Robot_tools import rbt_locator
from freecad.Robot_tools.App import rbt_traj, rbt_traj_plan
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_global_constants import ap_clr
from freecad.Robot_tools.App.rbt_placement import p_asm_in_world
from freecad.Robot_tools.App.rbt_traj_types import DocObj, Waypoint
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
        root -> asm_transformation (asm to world) -> [path_sep, marker_sep]
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

        # init asm to world pose
        self.asm_tx = coin.SoTransform()

        # path & wp marker separators
        self.path_sep = coin.SoSeparator()
        self.marker_sep = coin.SoSeparator()

        # gui nodes for waypoints
        self.wp_mats: Dict[str, coin.SoMaterial] = {}
        self.wp_seps: Dict[str, coin.SoSeparator] = {}

        root = so.sep(self.asm_tx, self.path_sep, self.marker_sep)
        vobj.addDisplayMode(root, "Standard")

        self.refresh_frame(vobj.Object)
        self.resample(vobj.Object)

    def updateData(self, fp: DocObj, prop: str) -> None:
        """
        resample when waypoints, num of prev samples, robot change
        """
        if prop in ("Waypoints_json", "Preview_samples"):
            self.resample(fp)
        elif prop == "Robot":
            self.refresh_frame(fp)
            self.resample(fp)

    def resample(self, fp: DocObj) -> None:
        """
        rebuild path polylines + waypoint markers from the doc
        """
        self.clear_display()
        robot = fp.Robot
        wps = rbt_traj.load_waypoints(fp)
        if robot is None or not wps:
            return
        self.build_markers(wps)
        self.build_path(fp, robot, wps)

    def clear_display(self) -> None:
        self.path_sep.removeAllChildren()
        self.marker_sep.removeAllChildren()
        self.wp_mats.clear()
        self.wp_seps.clear()

    def make_marker(self, wp: Waypoint) -> coin.SoSeparator:
        """
        in: waypoint
        out: sphere sep at the stored pose
        """
        mat = so.material(COLOR_WP)
        sphere = coin.SoSphere()
        sphere.radius.setValue(WP_RADIUS_MM)
        b = wp.tcp_in_asm.Base
        mk = so.sep(so.transform((b.x, b.y, b.z)), mat, sphere)

        # store the marker information
        self.wp_mats[wp.uid] = mat
        self.wp_seps[wp.uid] = mk

        return mk

    def build_markers(self, wps: List[Waypoint]) -> None:
        for wp in wps:
            self.marker_sep.addChild(self.make_marker(wp))

    def build_path(self, fp: DocObj,
                   robot: DocObj, wps: List[Waypoint]) -> None:
        """
        build tcp path needs if the points are valid
        """
        try:
            plan, _, _ = rbt_traj_plan.get_plan(fp)
        except Exception as e:   # chain not ready
            fcl_warn(f"{fp.Name}: path visualisation skipped - {e}\n")
            return
        if plan is None:
            return

        path_points = rbt_traj_plan.sample_tcp_path_asm(
            robot, plan, fp.Preview_samples)

        style = coin.SoDrawStyle()
        style.lineWidth = PATH_WIDTH
        for pts in path_points:
            self.path_sep.addChild(
                so.polyline([(p.x, p.y, p.z) for p in pts],
                            style, so.color(COLOR_PATH)))

    def refresh_frame(self, fp: DocObj) -> None:
        """
        re-read the asm->world transform after base move
        """
        robot = fp.Robot
        if robot is None:
            return
        plc = p_asm_in_world(robot)
        self.asm_tx.translation.setValue(plc.Base.x, plc.Base.y,
                                         plc.Base.z)
        self.asm_tx.rotation.setValue(*plc.Rotation.Q)

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
