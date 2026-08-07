"""
rbt_traj.py
Robot trajectory document object
"""
import json
from typing import List

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_TRAJ_SAMPLES, DEFAULT_TRAJ_SPEED, TRAJ_JSON_VERSION)
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_traj_types import (
    BY_DURATION, BY_SPEED, TimingRequest, Waypoint, DocObj)

TRAJ_SCHEMA = [
    ("Robot", "App::PropertyLinkGlobal", "Trajectory",
     "Robot this trajectory drives"),

    ("Waypoints_json", "App::PropertyString", "Trajectory",
     "(hidden) serialised waypoints, edit via the trajectory panel"),

    ("Waypoint_count", "App::PropertyInteger", "Trajectory",
     "(read only) number of waypoints"),

    ("Travel_mode", "App::PropertyEnumeration", "Timing",
     "Speed: percent for joint motion, mm/sec for linear | "
     "Duration: hit a given fixed time, speeds solved"),

    ("Speed", "App::PropertyFloat", "Timing",
     "Speed mode: % of full joint speed (ptp) | tcp mm/s (lin)"),

    ("Target_duration", "App::PropertyFloat", "Timing",
     "Duration mode: wanted total run time, seconds"),

    ("Preview_samples", "App::PropertyInteger", "Display",
     "Path preview samples per segment between two taught points"),
]

SCHEMA_PROPS = {n for n, *_ in TRAJ_SCHEMA}

TRAVEL_MODES = ["Speed", "Duration"]

EMPTY_JSON = json.dumps({"version": TRAJ_JSON_VERSION, "waypoints": []})


class Trajectory:
    def __init__(self, obj):
        self.add_properties(obj)
        obj.Proxy = self
        obj.Speed = DEFAULT_TRAJ_SPEED
        obj.Preview_samples = DEFAULT_TRAJ_SAMPLES
        obj.Waypoints_json = EMPTY_JSON
        self.set_editor_modes(obj)

    def add_properties(self, obj):
        """
        add missing properties to the objects
        helps old documents keep up with new property addition
        """
        for name, ptype, group, doc in TRAJ_SCHEMA:
            if name in obj.PropertiesList:
                continue
            obj.addProperty(ptype, name, group, doc)

            if name == "Travel_mode":
                # enum for the dropdown list creation
                obj.Travel_mode = TRAVEL_MODES

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

        elif prop == "Speed" and fp.Travel_mode == "Speed":
            # clamp PTP speed to 0 - 100 %
            # TODO: update this when we add LIN motion
            clamped = min(max(fp.Speed, 0), 100)
            if fp.Speed != clamped:
                fp.Speed = clamped

        if prop in SCHEMA_PROPS:
            self.plan_cache = None

    def execute(self, fp):
        pass


def load_timing(traj_obj) -> TimingRequest:
    """
    in: trajectory fpo
    out: TimingRequest from Travel_mode
    """
    if traj_obj.Travel_mode == "Duration":
        mode = BY_DURATION
        t = traj_obj.Target_duration
    else:
        mode = BY_SPEED
        t = 0

    return TimingRequest(mode, traj_obj.Speed, t)


def is_trajectory(obj) -> bool:
    """
    in: any fc doc object
    out: True if it has the traj schema
    """
    props = getattr(obj, "PropertiesList", [])
    return "Waypoints_json" in props and "Travel_mode" in props


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
