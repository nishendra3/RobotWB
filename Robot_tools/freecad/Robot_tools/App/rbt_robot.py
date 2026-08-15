"""Robot FreeCAD Python Object

Name: rbt_robot.py

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_properties import (
    ROBOT_SCHEMA, KINE_PROPS, RENAMED_PROPS)
from freecad.Robot_tools.App.rbt_kine import (
    invalidate, ensure_speed_props)
from freecad.Robot_tools.App.rbt_kine_joints import (
    JointCfg, save_cfg_map)
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_KIN_LIB, WB_NAME)
from freecad.Robot_tools.App.rbt_placement import (
    ensure_sync_observer, push_base_placement, pull_base_placement)
from freecad.Robot_tools.App.rbt_errors import RbtInputError
from freecad.Robot_tools.backends import KIN_LIB_NAMES
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_LIN_SPEED_TCP, DEFAULT_PTP_SPEED)

_TYPE_ = f"{WB_NAME}::Robot"

# ------------------------------------------------
#                   Robot Base Class
# ------------------------------------------------


class Robot:
    def __init__(self, obj):
        self.Type = _TYPE_
        self.add_properties(obj)
        obj.KinematicsLib = KIN_LIB_NAMES
        obj.KinematicsLib = DEFAULT_KIN_LIB

        # default speeds & limits & properties panel
        self.set_speed_settings(obj)
        self.set_editor_modes(obj)

        obj.Proxy = self
        ensure_sync_observer()

    def add_properties(self, obj):
        self.Type = _TYPE_
        for name, ptype, group, doc in ROBOT_SCHEMA:
            if not hasattr(obj, name):
                obj.addProperty(ptype, name, group, doc)

    def set_speed_settings(self, obj):
        for name, default, lo, hi in (
                ("PtpSpeedDefault", DEFAULT_PTP_SPEED, 0.0, 100.0),
                ("LinSpeedDefault", DEFAULT_LIN_SPEED_TCP, 0.001, 9999.0),
                ("SpeedOverride", 100.0, 1.0, 100.0)):
            val = getattr(obj, name) or default
            setattr(obj, name, (val, lo, hi, 1.0))

    def add_joint_prop(self, obj):
        if (hasattr(obj, "Robot_joints_dir")
                and "RobotJointsCfg" not in obj.PropertiesList):
            obj.addProperty("App::PropertyString", "RobotJointsCfg",
                            "Robot", "per-joint config")
            js = list(obj.RobotJoints)
            dirs = list(obj.Robot_joints_dir) + [1] * len(js)
            zeros = list(obj.Robot_zero_pose) + [0.0] * len(js)
            homes = list(obj.Robot_home_pos)
            homes += zeros[len(homes):]  # unset home falls back to zero
            save_cfg_map(obj, {j.Name: JointCfg(dirs[i], zeros[i], homes[i])
                               for i, j in enumerate(js)})
            for p in ("Robot_joints_dir", "Robot_zero_pose",
                      "Robot_home_pos"):
                obj.removeProperty(p)

    def migrate_rename_props(self, obj):
        """
        renames the per-joint list props for old docs
        """
        # migrate renamed props
        for old, new in RENAMED_PROPS.items():
            if old not in obj.PropertiesList:
                continue
            val = getattr(obj, old)
            obj.removeProperty(old)
            if new == "KinematicsLib":
                obj.KinematicsLib = KIN_LIB_NAMES
            setattr(obj, new, val)

    def migrate_retire_props(self, obj):
        """
        retire the per-joint list props for old docs
        """
        # remove stale property names
        speeds = list(getattr(obj, "Robot_joints_max_speed", []))
        for jnt, v in zip(obj.RobotJoints, speeds):
            if v > 0 and "MaxSpeed" in jnt.PropertiesList:
                jnt.MaxSpeed = v

        for p in ("Robot_joints_max_speed", "Robot_joints_min_lim",
                  "Robot_joints_max_lim"):
            if p in obj.PropertiesList:
                obj.removeProperty(p)

    def onChanged(self, obj, prop):
        '''Do something when a property has changed'''

        if "Restore" in obj.State:
            return

        if prop == "BasePlacement":
            # whole assembly moves
            push_base_placement(obj)
            return

        if prop == "BaseOffset":
            # asm stays, only frame moves
            pull_base_placement(obj)

        if prop in ("RobotJoints", "RobotJointsCfg",
                    "ActiveTool", "KinematicsLib"):
            try:
                invalidate(obj)
            except Exception:
                pass

        if prop in KINE_PROPS:
            for traj in (obj.Trajectories or []):
                traj.Proxy.plan_cache = None

        if prop == "RobotJoints":
            ensure_speed_props(obj)

    def onDocumentRestored(self, obj):
        # check and repair saved doc when its reopened
        self.add_properties(obj)
        self.migrate_rename_props(obj)
        self.add_joint_prop(obj)
        self.check_kin_libs(obj)
        self.check_default_tool(obj)
        ensure_speed_props(obj)
        self.migrate_retire_props(obj)

        # speed limits
        self.set_speed_settings(obj)

        # robot placement
        ensure_sync_observer()

        # migrates old docs & handle properties editor
        pull_base_placement(obj)
        self.migrate_traj_links(obj)
        self.set_editor_modes(obj)

    def set_editor_modes(self, obj):
        for p in ("RobotJointsCfg", "RobotLinks", "Trajectories"):
            obj.setEditorMode(p, 2)   # hidden
        for p in ("RobotJoints", "RobotAssembly", "Tools"):
            obj.setEditorMode(p, 1)   # read-only

    def migrate_traj_links(self, obj):
        """
        re-add Trajectories when an old doc has it non-hidden
        """
        if (obj.getTypeIdOfProperty("Trajectories")
                == "App::PropertyLinkListHidden"):
            return
        trajs = list(obj.Trajectories)
        obj.removeProperty("Trajectories")
        obj.addProperty("App::PropertyLinkListHidden", "Trajectories",
                        "Trajectory", "Trajectories attached to this robot")
        obj.Trajectories = trajs

    def check_kin_libs(self, obj):
        """
        Check if kin lib is attached to the obj
        """
        if (set(obj.getEnumerationsOfProperty("KinematicsLib"))
                != set(KIN_LIB_NAMES)):
            obj.KinematicsLib = KIN_LIB_NAMES
            obj.KinematicsLib = DEFAULT_KIN_LIB

    def check_default_tool(self, obj):
        """
        robots restored without a tool get default
        """
        # TODO: check with Carlo how to handle this UX
        pass

    def execute(self, obj):
        '''Do something when doing a recomputation, this method is mandatory'''
        pass

    def dumps(self): return None
    def loads(self, state): return None

# --------------------------------
#         HELPERS
# --------------------------------


def is_robot(obj) -> bool:
    """
    True if an obj is of type Robot FPO
    """
    proxy = getattr(obj, "Proxy", None)
    if getattr(proxy, "Type", "") == _TYPE_:
        return True
    return (hasattr(obj, "RobotJoints") and
            hasattr(obj, "RobotAssembly"))


def all_robots(doc=None):
    """
    Returns list of all robots in current doc
    defaults to current active document
    """
    doc = doc or App.ActiveDocument
    return [o for o in doc.Objects
            if is_robot(o)] if doc else []


def find_robot(hint=None, doc=None):
    """
    find robot fpo by Name/Label, or in a given doc
    """
    if hint is not None and not isinstance(hint, str):
        if not is_robot(hint):
            raise RbtInputError("not a robot FPO")
        return hint
    doc = doc or App.ActiveDocument
    if doc is None:
        raise RbtInputError("no active document; pass doc=")
    robs = all_robots(doc)
    if hint is not None:
        robs = [r for r in robs if hint in (r.Name, r.Label)]
    if len(robs) != 1:
        raise RbtInputError("robot not found or not unique: "
                            f"{[r.Label for r in robs]}")
    return robs[0]


def find_doc(doc=None):
    doc = doc or App.ActiveDocument
    if doc is None:
        raise RbtInputError("no active document; pass doc=")
    return doc
