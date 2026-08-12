"""
rbt_traj.py
Robot trajectory document object
"""
import json
from typing import List

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App import rbt_kine
from freecad.Robot_tools.App.rbt_kine import (
    fk_tcp_in_world, ik_tcp_in_world
)
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_TRAJ_SAMPLES, DEFAULT_LIN_SPEED_TCP, TRAJ_JSON_VERSION,
    DEFAULT_PTP_SPEED, WB_NAME)
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_traj_types import (
    SpeedSettings, Waypoint, DocObj, PTP,
    CARTESIAN, JOINT, new_uid)


_TYPE_ = f"{WB_NAME}::Trajectory"

TRAJ_SCHEMA = [
    ("Robot", "App::PropertyLinkGlobal", "Trajectory",
     "Robot this trajectory drives"),

    ("Waypoints_json", "App::PropertyString", "Trajectory",
     "(hidden) serialised waypoints, edit via the trajectory panel"),

    ("Waypoint_count", "App::PropertyInteger", "Trajectory",
     "(read only) number of waypoints"),

    ("Lin_speed_default", "App::PropertyFloat", "Timing",
     "default LIN TCP speed (mm/s)"),

    ("Ptp_speed_default", "App::PropertyFloat", "Timing",
     "default PTP speed (% of maximum possible robot speed)"),

    ("Speed_override", "App::PropertyFloat", "Timing",
     "global speed scaling 1-100 %"),

    ("Preview_samples", "App::PropertyInteger", "Display",
     "Path preview samples per segment between two taught points"),
]

SCHEMA_PROPS = {n for n, *_ in TRAJ_SCHEMA}

EMPTY_JSON = json.dumps({"version": TRAJ_JSON_VERSION, "waypoints": []})

LIM_EPS = 1e-6

CLAMPS = {
    "Ptp_speed_default": (0.0, 100.0),
    "Lin_speed_default": (LIM_EPS, float("inf")),
    "Speed_override": (1.0, 100.0),
}


class Trajectory:
    def __init__(self, obj):
        self.Type = _TYPE_
        self.add_properties(obj)
        obj.Proxy = self
        obj.Preview_samples = DEFAULT_TRAJ_SAMPLES
        obj.Waypoints_json = EMPTY_JSON
        self.set_editor_modes(obj)

        obj.Ptp_speed_default = DEFAULT_PTP_SPEED
        obj.Lin_speed_default = DEFAULT_LIN_SPEED_TCP

        obj.Speed_override = 100

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

    def set_editor_modes(self, fp):
        """
        hide json waypoints & counts set to read-only
        """
        READ_ONLY = 1
        HIDDEN = 2
        fp.setEditorMode("Waypoints_json", HIDDEN)
        fp.setEditorMode("Waypoint_count", READ_ONLY)

    def onDocumentRestored(self, fp):
        self.add_properties(fp)
        self.safe_load_json(fp)
        self.set_editor_modes(fp)

    def safe_load_json(self, fp):
        """
        read the json file & repair + update
        if neeeded
        """
        try:
            json.loads(fp.Waypoints_json or EMPTY_JSON)
        except ValueError:
            fcl_warn(f"{fp.Name}: unreadable waypoint json data\n")
            fp.Waypoints_json = EMPTY_JSON

    def onChanged(self, fp, prop):
        if "Restore" in fp.State:
            return

        if prop == "Waypoints_json":
            n = len(load_waypoints(fp))
            if fp.Waypoint_count != n:
                fp.Waypoint_count = n

        elif prop in CLAMPS:
            low, high = CLAMPS[prop]
            val = getattr(fp, prop)
            clamped = min(max(val, low), high)
            if val != clamped:
                setattr(fp, prop, clamped)

        if prop in SCHEMA_PROPS:
            self.plan_cache = None

    # bypass default freecad methods
    def execute(self, fp): pass
    def dumps(self): return None
    def loads(self, state): return None


def load_speed_settings(traj_obj) -> SpeedSettings:
    """
    in: trajectory fpo
    out: SpeedSettings
    """
    return SpeedSettings(traj_obj.Lin_speed_default,
                         traj_obj.Ptp_speed_default,
                         traj_obj.Speed_override)


def is_trajectory(obj) -> bool:
    """
    in: any fc doc object
    out: True if it has the traj schema
    """
    proxy = getattr(obj, "Proxy", None)
    if getattr(proxy, "Type", "") == _TYPE_:
        return True
    props = getattr(obj, "PropertiesList", [])
    return "Waypoints_json" in props and "Ptp_speed_default" in props


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
        data = json.loads(traj_obj.Waypoints_json or EMPTY_JSON)
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
    traj_obj.Waypoints_json = json.dumps({
        "version": TRAJ_JSON_VERSION,
        "waypoints": [wp.to_dict() for wp in wps]})


def teach_waypoint(traj_obj, name: str = "", motion=PTP) -> Waypoint:
    """
    append a JOINT waypoint at the robot's current pose
    """
    rob = traj_obj.Robot
    wps = load_waypoints(traj_obj)
    tcp = fk_tcp_in_world(rob) or App.Placement()
    wp = Waypoint(new_uid(), name or next_waypoint_name(wps), JOINT,
                  rbt_kine.curr_joint_vals_doc(rob), tcp, motion)
    save_waypoints(traj_obj, wps + [wp])
    return wp


def add_cartesian_waypoint(traj_obj, target, name: str = "",
                           motion=PTP):
    """
    CARTESIAN waypoint at target
    None when unreachable
    """
    q = ik_tcp_in_world(traj_obj.Robot, target)
    if q is None:
        return None
    wps = load_waypoints(traj_obj)
    wp = Waypoint(new_uid(), name or next_waypoint_name(wps), CARTESIAN,
                  q, target, motion)
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
