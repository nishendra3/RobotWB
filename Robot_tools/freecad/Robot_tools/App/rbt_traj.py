"""
rbt_traj.py
Robot trajectory document object
"""
import json
from typing import List

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App import rbt_kine
from freecad.Robot_tools.App.rbt_properties import (
    TRAJ_SCHEMA, TRAJ_PROPS, SPEED_PROPS)
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_TRAJ_SAMPLES, TRAJ_JSON_VERSION, WB_NAME)
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_traj_types import (
    Waypoint, DocObj, PTP, CARTESIAN, JOINT, new_uid)

_TYPE_ = f"{WB_NAME}::Trajectory"

EMPTY_JSON = json.dumps({"version": TRAJ_JSON_VERSION, "waypoints": []})


class Trajectory:
    def __init__(self, obj):
        self.Type = _TYPE_
        self.add_properties(obj)
        obj.Proxy = self
        obj.PreviewSamples = DEFAULT_TRAJ_SAMPLES
        obj.WaypointsJson = EMPTY_JSON
        self.set_editor_modes(obj)

    def add_properties(self, obj):
        """
        add missing properties to the objects
        helps old documents keep up with new property addition
        """
        self.Type = _TYPE_
        for name, ptype, group, doc in TRAJ_SCHEMA:
            if name in obj.PropertiesList:
                continue
            obj.addProperty(ptype, name, group, doc)

    def drop_speed_props(self, fp):
        """
        remove props from older docs
        """
        rob = getattr(fp, "Robot", None)
        for p in SPEED_PROPS:
            if p not in fp.PropertiesList:
                continue
            if rob is not None and hasattr(rob, p):
                setattr(rob, p, getattr(fp, p))
            fp.removeProperty(p)

    def set_editor_modes(self, fp):
        """
        hide json waypoints & counts set to read-only
        """
        READ_ONLY = 1
        HIDDEN = 2
        fp.setEditorMode("WaypointsJson", HIDDEN)
        fp.setEditorMode("WaypointCount", READ_ONLY)

    def onDocumentRestored(self, fp):
        self.add_properties(fp)
        self.drop_speed_props(fp)
        self.safe_load_json(fp)
        self.set_editor_modes(fp)

    def safe_load_json(self, fp):
        """
        read the json file & repair + update
        if neeeded
        """
        try:
            json.loads(fp.WaypointsJson or EMPTY_JSON)
        except ValueError:
            fcl_warn(f"{fp.Name}: unreadable waypoint json data\n")
            fp.WaypointsJson = EMPTY_JSON

    def onChanged(self, fp, prop):
        if "Restore" in fp.State:
            return

        if prop == "WaypointsJson":
            n = len(load_waypoints(fp))
            if fp.WaypointCount != n:
                fp.WaypointCount = n

        if prop in TRAJ_PROPS:
            self.plan_cache = None

    # bypass default freecad methods
    def execute(self, fp): pass
    def dumps(self): return None
    def loads(self, state): return None


def is_trajectory(obj) -> bool:
    """
    in: any fc doc object
    out: True if it has the traj schema
    """
    proxy = getattr(obj, "Proxy", None)
    if getattr(proxy, "Type", "") == _TYPE_:
        return True
    props = getattr(obj, "PropertiesList", [])
    return "WaypointsJson" in props


def next_waypoint_name(wps: List[Waypoint]) -> str:
    """
    first unused P<n>  for waypoint
    """
    used = {wp.name for wp in wps}
    i = len(wps) + 1
    while f"P{i}" in used:
        i += 1
    return f"P{i}"


def load_waypoints(traj_obj) -> List[Waypoint]:
    """
    in: trajectory fpo
    out: waypoints, empty list on missing/bad json
    """
    try:
        data = json.loads(traj_obj.WaypointsJson or EMPTY_JSON)
        wps = data.get("waypoints", [])
        return [Waypoint.from_dict(w) for w in wps]
    except (ValueError, TypeError) as e:
        fcl_warn(f"{traj_obj.Name}: bad waypoint json - {e}\n")
        return []


def save_waypoints(traj_obj, wps: List[Waypoint]) -> None:
    """
    single json write & count sync
    in: trajectory fpo, waypoints
    """
    traj_obj.WaypointsJson = json.dumps({
        "version": TRAJ_JSON_VERSION,
        "waypoints": [wp.to_dict() for wp in wps]})


def teach_waypoint(traj_obj, name: str = "", motion=PTP) -> Waypoint:
    """
    append a JOINT waypoint at the robot's current pose
    """
    rob = traj_obj.Robot
    wps = load_waypoints(traj_obj)
    tcp = rbt_kine.fk_tcp_in_world(rob) or App.Placement()
    wp = Waypoint(uid=new_uid(),
                  name=name or next_waypoint_name(wps),
                  mode=JOINT,
                  q_doc=rbt_kine.curr_joint_vals_doc(rob),
                  tcp_in_world=tcp,
                  motion=motion,
                  speed=rbt_kine.default_speed(rob, motion))
    save_waypoints(traj_obj, wps + [wp])
    return wp


def add_cartesian_waypoint(traj_obj, target, name: str = "",
                           motion=PTP):
    """
    CARTESIAN waypoint at target
    None when unreachable
    """
    rob = traj_obj.Robot
    q = rbt_kine.ik_tcp_in_world(rob, target)
    if q is None:
        return None
    wps = load_waypoints(traj_obj)
    wp = Waypoint(uid=new_uid(),
                  name=(name or next_waypoint_name(wps)),
                  mode=CARTESIAN,
                  q_doc=q,
                  tcp_in_world=target,
                  motion=motion,
                  speed=rbt_kine.default_speed(rob, motion))
    save_waypoints(traj_obj, wps + [wp])
    return wp


def rbt_trajectories(rbt_obj) -> List:
    """
    in: robot fpo
    out: robot's trajectory objects
    """
    return list(getattr(rbt_obj, "Trajectories", None) or [])


def create_trajectory(rbt_obj) -> DocObj:
    """
    new Trajectory FPO attached to a robot
    in: robot fpo
    out: the new trajectory fpo
    """
    doc = rbt_obj.Document
    fpo = doc.addObject("App::FeaturePython", "Trajectory")
    Trajectory(fpo)
    fpo.Robot = rbt_obj
    rbt_obj.Trajectories = list(rbt_obj.Trajectories) + [fpo]
    if App.GuiUp:
        from freecad.Robot_tools.Gui.g_rbt_traj_vp import (
            ViewProviderTrajectory)
        ViewProviderTrajectory(fpo.ViewObject)
    return fpo
