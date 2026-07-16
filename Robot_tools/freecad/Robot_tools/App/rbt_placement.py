"""
rbt_placement.py
Handles placement of robot model & syncs observer
"""

import FreeCAD as App  # type: ignore
import UtilsAssembly  # type: ignore

from freecad.Robot_tools.App.rbt_helpers_log import fcl_err

PLC_TOL = 1e-6
_SYNC = False  # guard flag to prevent recursion
_observer = None


# REMOVE: Legacy Path --------
def find_grounded_joint(asm):
    """
    Search which object is the ground link
    """
    if asm is None:
        return None

    jg = UtilsAssembly.getJointGroup(asm)
    if jg is None:
        return None

    return next((o for o in jg.Group
                 if hasattr(o, "ObjectToGround")), None)
# ----------------------------


def is_grounded_datum(obj, asm):
    """
    True if obj is a datum the asm solver treats as ground:
    LCS/datum type, directly in the asm root (not nested in
    a part), and unattached (MapMode None/'Deactivated')
    """
    if obj is None or asm is None:
        return False

    s_lcs = "App::LocalCoordinateSystem"
    s_de = "App::DatumElement"

    if not (obj.isDerivedFrom(s_lcs)
            or obj.isDerivedFrom(s_de)):
        return False

    if obj not in asm.Group:
        return False

    return getattr(obj, "MapMode", None) in (None, "Deactivated")


def is_base_joint(joint, asm):
    """
    True if the joint's Reference1 is the
    grounded BaseFrame datum
    """
    refs = getattr(joint, "Reference1", None)
    return bool(refs) and is_grounded_datum(refs[0], asm)


def joint_dir(joint):
    """
    +1/-1 jog direction from the parent robot obj
    default: +1
    """
    for fpo in joint.Document.findObjects("App::FeaturePython"):
        js = getattr(fpo, "Robot_joints", None)
        if js and joint in js:
            dirs = getattr(fpo, "Robot_joints_dir", ())
            i = list(js).index(joint)
            return dirs[i] if i < len(dirs) else 1
    return 1


def chain_root(rbt_obj):
    """Reference1 obj of the first robot joint, or None"""
    joints = list(getattr(rbt_obj, "Robot_joints", None) or [])
    if not joints or not joints[0].Reference1:
        return None
    return joints[0].Reference1[0]


def base_link(robot):
    """
    identify the base link for given robot obj
    priority order: BaseFrame Datum -> GroundedJoint -> First Ref1
    """
    asm = getattr(robot, "Robot_assembly", None)
    root = chain_root(robot)

    if is_grounded_datum(root, asm):
        return root

    # REMOVE: Legacy Path -------------
    gj = find_grounded_joint(getattr(robot, "Robot_assembly", None))
    if gj is not None and gj.ObjectToGround is not None:
        return gj.ObjectToGround
    # ---------------------------------

    return root


def p_asm_in_world(robot) -> App.Placement:
    """
    pose of the robot_assembly container in the world coords
    p_asm_world = p_world * p_asm
    """
    return robot.Robot_assembly.getGlobalPlacement()


def p_parent_in_world(robot) -> App.Placement:
    """
    pose of the frame the assembly is kept in
    p_world = p_asm_world * inv(p_asm)
    """
    asm = robot.Robot_assembly
    return (asm.getGlobalPlacement()
            .multiply(asm.Placement.inverse()))


def get_base_placement(robot):
    bl = base_link(robot)
    if (getattr(robot, "Robot_assembly", None) is None or
            bl is None):
        return None

    # bl.Placement places base CAD in asm, frozen when BaseFrame
    # is made moving baselink placement moves the whole geometry
    #
    # Base_offset places the frame on that link relative to its CAD
    # origin. Changing it just moves frame, not the part

    return (p_asm_in_world(robot)
            .multiply(bl.Placement)
            .multiply(robot.Base_offset))


def push_base_placement(robot):
    """
    FPO.Base_placement -> asm.Placement
    (from Robot.onChanged)
    """
    global _SYNC
    if _SYNC:
        return
    asm, bl = getattr(robot, "Robot_assembly", None), base_link(robot)
    if asm is None or bl is None:
        # TODO: This will currently skip unless the robot exists
        # check if we want to handle the case where user wants to
        # move the robot in the middle of the creation process already
        return

    tgt = (p_parent_in_world(robot).inverse()
           .multiply(robot.Base_placement)
           .multiply(robot.Base_offset.inverse())
           .multiply(bl.Placement.inverse()))

    if asm.Placement.isSame(tgt, PLC_TOL):
        return

    _SYNC = True

    try:
        asm.Placement = tgt  # rigid body move
    finally:
        _SYNC = False

    after_base_move(robot)


def pull_base_placement(robot):
    """
    asm.Placement -> FPO.Base_placement
    (on resotre or edits of Base_offset)
    """
    global _SYNC
    if _SYNC:
        return
    bp = get_base_placement(robot)
    if bp is None:
        return
    if not robot.Base_placement.isSame(bp, PLC_TOL):
        _SYNC = True
        try:
            robot.Base_placement = bp
        finally:
            _SYNC = False

    after_base_move(robot)


def after_base_move(robot):
    """
    refresh the tool position after robot is moved
    """
    tool = getattr(robot, "Active_tool", None)
    if tool is not None:
        try:
            tool.recompute()
        except Exception as e:
            fcl_err(f"tool recompute after base move failed: {e}")


class BaseLinkSyncObserver:
    """
    mirrors the direct assembly moves into the base placement
    thus keeping the frame location & assembly in sync
    """

    def slotChangedObject(self, obj, prop):
        if prop != "Placement" or obj.TypeId != "Assembly::AssemblyObject":
            return

        # avoid importing rbt_robot.is_robot here due to cyclic import
        for rob in obj.InList:
            if (getattr(rob, "Robot_assembly", None) is obj
                    and hasattr(rob, "Base_placement")):
                pull_base_placement(rob)


def ensure_sync_observer():
    global _observer
    if _observer is None:
        _observer = BaseLinkSyncObserver()
        App.addDocumentObserver(_observer)
