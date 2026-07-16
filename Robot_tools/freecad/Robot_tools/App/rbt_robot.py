"""Robot FreeCAD Python Object

Name: rbt_robot.py

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

from freecad.Robot_tools.App.rbt_kine import invalidate
from freecad.Robot_tools.App.rbt_kine_chain import joint_dirs
from freecad.Robot_tools.App.rbt_global_constants import DEFAULT_KIN_LIB
from freecad.Robot_tools.App.rbt_placement import (
    ensure_sync_observer, push_base_placement, pull_base_placement)

from freecad.Robot_tools.backends import KIN_LIB_NAMES


ROBOT_SCHEMA = [

    # core properties
    ("Robot_assembly", "App::PropertyLinkGlobal",
     "Robot", "Robot assembly"),
    ("Robot_joints", "App::PropertyLinkListGlobal",
     "Robot", "Robot joints list"),
    ("Robot_links", "App::PropertyPlacementList",
     "Robot", "Robot links list"),
    ("Robot_joints_dir", "App::PropertyIntegerList",
     "Robot", "Joint direction sign (+1/-1) per joint"),
    ("Robot_home_pos", "App::PropertyFloatList",
     "Robot", "Home position (revolute: deg, slider: mm)"),
    ("Robot_zero_pose", "App::PropertyFloatList",
     "Robot", "Robot pose that is to be taken as null reference"),

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
]


# ------------------------------------------------
#                   Robot Objects
# ------------------------------------------------

class Robot:
    def __init__(self, obj):
        '''Add some custom properties to our box feature'''
        self.add_properties(obj)
        obj.Kinematics_lib = KIN_LIB_NAMES
        obj.Kinematics_lib = DEFAULT_KIN_LIB
        obj.Proxy = self
        ensure_sync_observer()

    def add_properties(self, obj):
        for name, ptype, group, doc in ROBOT_SCHEMA:
            if not hasattr(obj, name):
                obj.addProperty(ptype, name, group, doc)

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

        if prop in ("Robot_joints", "Robot_joints_dir", "Robot_zero_pose",
                    "Active_tool", "Kinematics_lib"):
            try:
                invalidate(obj)
            except Exception:
                pass

    def onDocumentRestored(self, obj):
        # check and repair saved doc when its reopened
        self.add_properties(obj)
        self.check_kin_libs(obj)
        self.check_joints_direction(obj)
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

    def check_joints_direction(self, obj):
        """
        Check if the rotation direction for joints is set
        """
        dirs = joint_dirs(obj)
        if dirs != list(obj.Robot_joints_dir or []):
            obj.Robot_joints_dir = dirs

    def check_default_tool(self, obj):
        """
        robots restored without a tool get default
        """
        # TODO: check with Carlo how to handle this UX
        pass

    def execute(self, obj):
        '''Do something when doing a recomputation, this method is mandatory'''
        pass


# --------------------------------
#         HELPERS
# --------------------------------

def is_robot(obj) -> bool:
    """
    True if an obj is of type Robot FPO
    """
    return (hasattr(obj, "Robot_joints") and
            hasattr(obj, "Robot_assembly"))
