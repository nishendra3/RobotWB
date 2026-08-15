"""
rbt_placement.py
Handles placement of robot model & syncs observer
"""

import FreeCAD as App  # type: ignore
import UtilsAssembly  # type: ignore

from freecad.Robot_tools.App.rbt_kine_joints import get_joint_cfg
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err
from freecad.Robot_tools.App.rbt_properties import JNT_KINE_PROPS

PLC_TOL = 1e-6
_SYNC: set[tuple[str, str]] = set()  # (doc, name) of robs currently mid-sync
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


def is_inertial_datum(obj, ref):
    if obj is None or not ref:
        return None
    o = UtilsAssembly.getObject([obj, [ref, ref]])
    return o if getattr(o, "MapMode", None) == "InertialCS" else None


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
    for fpo in joint.InList:
        js = getattr(fpo, "RobotJoints", None)
        if js and joint in js:
            return get_joint_cfg(fpo, joint).dir
    return 1


def chain_root(rbt_obj):
    """Reference1 obj of the first robot joint, or None"""
    joints = list(getattr(rbt_obj, "RobotJoints", None) or [])
    if not joints or not joints[0].Reference1:
        return None
    return joints[0].Reference1[0]


def base_link(robot):
    """
    identify the base link for given robot obj
    priority order: BaseFrame Datum -> GroundedJoint -> First Ref1
    """
    asm = getattr(robot, "RobotAssembly", None)
    root = chain_root(robot)

    if is_grounded_datum(root, asm):
        return root

    # REMOVE: Legacy Path -------------
    gj = find_grounded_joint(getattr(robot, "RobotAssembly", None))
    if gj is not None and gj.ObjectToGround is not None:
        return gj.ObjectToGround
    # ---------------------------------

    return root


def p_asm_in_world(robot) -> App.Placement:
    """
    pose of the robotAssembly container in the world coords
    p_asm_world = p_world * p_asm
    """
    return robot.RobotAssembly.getGlobalPlacement()


def p_world_to_asm(robot, plc_world: App.Placement) -> App.Placement:
    return p_asm_in_world(robot).inverse().multiply(plc_world)


def p_asm_to_world(robot, plc_asm: App.Placement) -> App.Placement:
    return p_asm_in_world(robot).multiply(plc_asm)


def p_parent_in_world(robot) -> App.Placement:
    """
    pose of the frame the assembly is kept in
    p_world = p_asm_world * inv(p_asm)
    """
    asm = robot.RobotAssembly
    return (asm.getGlobalPlacement()
            .multiply(asm.Placement.inverse()))


def get_base_placement(robot):
    bl = base_link(robot)
    if (getattr(robot, "RobotAssembly", None) is None or
            bl is None):
        return None

    # bl.Placement places base CAD in asm, frozen when BaseFrame
    # is made moving baselink placement moves the whole geometry
    #
    # BaseOffset places the frame on that link relative to its CAD
    # origin. Changing it just moves frame, not the part

    return (p_asm_in_world(robot)
            .multiply(bl.Placement)
            .multiply(robot.BaseOffset))


def push_base_placement(robot):
    """
    FPO.BasePlacement -> asm.Placement
    (from Robot.onChanged)
    """
    key = (robot.Document.Name, robot.Name)
    if key in _SYNC:
        return
    asm, bl = getattr(robot, "RobotAssembly", None), base_link(robot)
    if asm is None or bl is None:
        # TODO: This will currently skip unless the robot exists
        # check if we want to handle the case where user wants to
        # move the robot in the middle of the creation process already
        return

    tgt = (p_parent_in_world(robot).inverse()
           .multiply(robot.BasePlacement)
           .multiply(robot.BaseOffset.inverse())
           .multiply(bl.Placement.inverse()))

    if asm.Placement.isSame(tgt, PLC_TOL):
        return

    _SYNC.add(key)

    try:
        asm.Placement = tgt  # rigid body move
    finally:
        _SYNC.discard(key)

    after_base_move(robot)


def pull_base_placement(robot):
    """
    asm.Placement -> FPO.BasePlacement
    (on resotre or edits of BaseOffset)
    """
    key = (robot.Document.Name, robot.Name)
    if key in _SYNC:
        return
    bp = get_base_placement(robot)
    if bp is None:
        return
    if robot.BasePlacement.isSame(bp, PLC_TOL):
        return

    _SYNC.add(key)

    try:
        robot.BasePlacement = bp
    finally:
        _SYNC.discard(key)

    after_base_move(robot)


def after_base_move(robot):
    """
    refresh dependent displays after
    robot has been moved
    """
    refresh_tool(robot)
    invalidate_traj_plans(robot)
    refresh_trajectories(robot)


def invalidate_traj_plans(robot):
    """
    base moved -> cartesian IK solutions are invalid
    """
    for traj in (getattr(robot, "Trajectories", None) or []):
        traj.Proxy.plan_cache = None


def refresh_tool(robot):
    """
    recompute the active tool's tcp
    """
    tool = getattr(robot, "ActiveTool", None)
    if tool is not None:
        try:
            tool.recompute()
        except Exception as e:
            fcl_err(f"tool recompute after base move failed: {e}")


def refresh_trajectories(robot):
    """
    re-aim trajectory displays
    """
    if not App.GuiUp:
        return
    for traj_obj in getattr(robot, "Trajectories", None) or []:
        vp = getattr(traj_obj.ViewObject, "Proxy", None)
        if vp is not None:
            vp.resample(traj_obj)


class BaseLinkSyncObserver:
    """
    doc-wide observer:
    - joint speed/limit edits -> drop stale plan caches
    - direct assembly moves -> mirror into BasePlacement
    """

    def slotChangedObject(self, obj, prop):
        # joint speed/limit edit -> owning robot's plans are stale
        if prop in JNT_KINE_PROPS:
            for rob in obj.InList:
                js = getattr(rob, "RobotJoints", None)
                if js and obj in js:
                    for traj in (rob.Trajectories or []):
                        traj.Proxy.plan_cache = None
            return

        # direct assembly moves only
        if prop != "Placement" or obj.TypeId != "Assembly::AssemblyObject":
            return

        # mirror the move into BasePlacement
        for rob in obj.InList:
            if (getattr(rob, "RobotAssembly", None) is obj
                    and hasattr(rob, "BasePlacement")):
                pull_base_placement(rob)


def ensure_sync_observer():
    global _observer
    if _observer is None:
        _observer = BaseLinkSyncObserver()
        App.addDocumentObserver(_observer)
