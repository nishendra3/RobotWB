"""
Joint creation handling for new robot creation
Current Support: Revolute Jnts, Base Jnt
"""

import FreeCAD as App  # type: ignore

import JointObject   # type: ignore
import UtilsAssembly  # type: ignore

from freecad.Robot_tools.App.rbt_creator_geom import find_center
from freecad.Robot_tools.App.rbt_kine_types import JOINT_TYPES


def add_joint(asm, jtype, refs, label=""):
    """
    Create a new joint in the assembly & return it
    refs are the objects that make up the joint
    refs = [(obj, elem_ref), ...]
    """
    jg = UtilsAssembly.getJointGroup(asm)
    j = jg.newObject("App::FeaturePython", "Joint")
    j.Label2 = label
    proxy = JointObject.Joint(j, JOINT_TYPES[jtype])

    # handle base-link padlock icon
    if App.GuiUp:
        from freecad.Robot_tools.Gui.vp_rbt_joint \
            import ViewProviderBaseJoint
        ViewProviderBaseJoint(j.ViewObject)
        j.Visibility = True

    (o1, r1), (o2, r2) = refs[0], refs[1]
    j.Reference1 = find_center(o1, r1, jtype)
    j.Reference2 = find_center(o2, r2, jtype)
    j.Offset2 = initial_offset2(proxy, j)
    # proxy.preSolve(j, savePlc=False)
    asm.Document.recompute()
    return j


def initial_offset2(proxy, j):
    """
    Offset2 that are present by default when new parts are
    opened, so making new robot assembly won't spin the part
    """
    plc1 = UtilsAssembly.getJcsGlobalPlc(j.Placement1, j.Reference1)
    plc2 = UtilsAssembly.getJcsGlobalPlc(j.Placement2, j.Reference2)
    rel = plc2.inverse() * plc1
    if not proxy.areJcsSameDir(j):
        # normalize the 180 z-flip
        rel = UtilsAssembly.flipPlacement(rel)
    if j.JointType == "Fixed":
        return rel
    yaw = rel.Rotation.toEuler()[0]
    z = rel.Base.z if j.JointType == "Slider" else 0.0
    return App.Placement(App.Vector(0, 0, z), App.Rotation(yaw, 0, 0))
