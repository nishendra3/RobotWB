"""Robot FreeCAD Python Object

Name: rbt_robot.py

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_kine import invalidate
from freecad.Robot_tools.App.rbt_kine_joints import (
    JointCfg, save_cfg_map)
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_KIN_LIB, WB_NAME)
from freecad.Robot_tools.App.rbt_placement import (
    ensure_sync_observer, push_base_placement, pull_base_placement)
from freecad.Robot_tools.App.rbt_errors import RbtInputError
from freecad.Robot_tools.backends import KIN_LIB_NAMES

_TYPE_ = f"{WB_NAME}::Robot"

ROBOT_SCHEMA = [

    # core properties
    ("Robot_assembly", "App::PropertyLinkGlobal",
     "Robot", "Robot assembly"),
    ("Robot_joints", "App::PropertyLinkListGlobal",
     "Robot", "Robot joints list"),
    ("Robot_links", "App::PropertyPlacementList",
     "Robot", "Robot links list"),
    ("Robot_joints_cfg", "App::PropertyString", "Robot",
     "per-joint config json: {joint: {dir, zero, home}}"),

    # tool handling properties
    ("Tools", "App::PropertyLinkListGlobal",
     "Tools", "Tool FPOs attached"),
    ("Active_tool", "App::PropertyLinkGlobal",
     "Tools", "Currently active tool"),

    # kinematics lib properties
    ("Kinematics_lib", "App::PropertyEnumeration",
     "Kinematics", "FK/IK solver"),

    # placement properties
    ("Base_placement", "App::PropertyPlacement", "Placement",
     "World -> robot base frame"),
    ("Base_offset", "App::PropertyPlacement", "Placement",
     "base frame in base-link coords "
     "(moves the frame label, not the robot)"),

    # trajectory properties
    # hidden scope for "Trajectories" below to prevent dependency loop
    ("Trajectories", "App::PropertyLinkListHidden", "Trajectory",
     "Trajectories attached to this robot"),
    ("Robot_joints_max_speed", "App::PropertyFloatList", "Kinematics",
     "Full joint speed (=100%), deg/s (revolute) | mm/s (prismatic)"),
]


# ------------------------------------------------
#                   Robot Objects
# ------------------------------------------------

class Robot:
    def __init__(self, obj):
        self.Type = _TYPE_
        self.add_properties(obj)
        obj.Kinematics_lib = KIN_LIB_NAMES
        obj.Kinematics_lib = DEFAULT_KIN_LIB
        obj.Proxy = self
        ensure_sync_observer()

    def add_properties(self, obj):
        self.Type = _TYPE_
        for name, ptype, group, doc in ROBOT_SCHEMA:
            if not hasattr(obj, name):
                obj.addProperty(ptype, name, group, doc)

    def add_joint_prop(self, obj):
        if (hasattr(obj, "Robot_joints_dir")
                and "Robot_joints_cfg" not in obj.PropertiesList):
            obj.addProperty("App::PropertyString", "Robot_joints_cfg",
                            "Robot", "per-joint config")
            js = list(obj.Robot_joints)
            dirs = list(obj.Robot_joints_dir) + [1] * len(js)
            zeros = list(obj.Robot_zero_pose) + [0.0] * len(js)
            homes = list(obj.Robot_home_pos)
            homes += zeros[len(homes):]  # unset home falls back to zero
            save_cfg_map(obj, {j.Name: JointCfg(dirs[i], zeros[i], homes[i])
                               for i, j in enumerate(js)})
            for p in ("Robot_joints_dir", "Robot_zero_pose",
                      "Robot_home_pos"):
                obj.removeProperty(p)

    def onChanged(self, obj, prop):
        '''Do something when a property has changed'''

        if "Restore" in obj.State:
            return

        if prop == "Base_placement":
            # whole assembly moves
            push_base_placement(obj)
            return

        if prop == "Base_offset":
            # asm stays, only frame moves
            pull_base_placement(obj)

        if prop in ("Robot_joints", "Robot_joints_cfg",
                    "Active_tool", "Kinematics_lib"):
            try:
                invalidate(obj)
            except Exception:
                pass

    def onDocumentRestored(self, obj):
        # check and repair saved doc when its reopened
        self.add_joint_prop(obj)
        self.add_properties(obj)
        self.check_kin_libs(obj)
        self.check_default_tool(obj)

        # robot placement
        ensure_sync_observer()
        pull_base_placement(obj)  # migrates old docs

    def check_kin_libs(self, obj):
        """
        Check if kin lib is attached to the obj
        """
        if (set(obj.getEnumerationsOfProperty("Kinematics_lib"))
                != set(KIN_LIB_NAMES)):
            obj.Kinematics_lib = KIN_LIB_NAMES
            obj.Kinematics_lib = DEFAULT_KIN_LIB

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
    return (hasattr(obj, "Robot_joints") and
            hasattr(obj, "Robot_assembly"))


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
