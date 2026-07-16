"""
rbt_kine_chain.py
extract chain spec from robot fpo
"""

from __future__ import annotations

from typing import List, Optional, TypeAlias

import FreeCAD as App  # type: ignore
import UtilsAssembly  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import (
    REVOLUTE, ChainSpec, JointSpec, LinkSpec, joint_type_FC2WB,
    FIXED, PRISMATIC
)
from freecad.Robot_tools.App.rbt_placement import p_asm_in_world, base_link
from freecad.Robot_tools.App.rbt_helpers_math import deg_to_rad, rad_to_deg
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err
from freecad.Robot_tools.App.rbt_global_constants import (
    MM_PER_M, DEFAULT_SLIDER_TRAVEL_MM)

V3: TypeAlias = App.Vector
Placement: TypeAlias = App.Placement


def extract_chain(robot_obj: "App.DocumentObject") -> Optional[ChainSpec]:
    """
    ChainSpec of the q=0 structure, measured at any pose:
    the Rot(axis, q) terms that apply_joint_angles inserts are removed
    algebraically (unrotate)
    All frames in robot-assembly coords -> chain invariant under base moves
    """
    joints_fpo: List["App.DocumentObject"] = list(robot_obj.Robot_joints or [])
    if not joints_fpo:
        fcl_err("Robot has no joints")
        return

    # world -> asm converter
    # (asm ← X) = (asm ← world)*(world ← X)
    # p_X_in_asm = p_world_in_asm * p_X_in_world

    p_world_in_asm: Placement = p_asm_in_world(robot_obj).inverse()
    base_in_asm: Placement = p_world_in_asm.multiply(
        UtilsAssembly.getGlobalPlacement(joints_fpo[0].Reference1))

    # init links, joints & joint directions
    ground_obj: "App.DocumentObject" = base_link(robot_obj)
    if ground_obj is None:
        fcl_err("No grounded link found — ground the base part first")
        return None
    if (joints_fpo[0].Reference1[0] is not ground_obj):
        fcl_err(f"'{joints_fpo[0].Label}': Reference1 must be grounded")
        return None
    links: List[LinkSpec] = [LinkSpec(name=ground_obj.Name,
                                      parent_name=None)]
    joints: List[JointSpec] = []
    dirs: List[int] = joint_dirs(robot_obj)
    zeros: List[float] = joint_zeros(robot_obj)

    prev_joint_in_asm: Placement = base_in_asm
    prev_axis, prev_q, prev_jtype = None, 0, None

    for idx, j in enumerate(joints_fpo):
        try:
            parent_obj: "App.DocumentObject" = j.Reference1[0]
            child_obj: "App.DocumentObject" = j.Reference2[0]
        except Exception as e:
            fcl_err(f"Unable to read parent & child obj: {e}")
            return None

        if idx > 0 and parent_obj is not joints_fpo[idx - 1].Reference2[0]:
            fcl_err("Error in connectivity. Check faces pick order")
            return None

        joint_in_asm: Placement = jcs_in_asm(p_world_in_asm,
                                             j.Reference1,
                                             j.Placement1)

        jcs2_in_asm: Placement = jcs_in_asm(p_world_in_asm,
                                            j.Reference2,
                                            j.Placement2)
        d = dirs[idx]
        jtype = joint_type_FC2WB(j.JointType)
        axis: V3 = joint_axis(joint_in_asm, jcs2_in_asm, d)
        q = joint_value_doc(j, d, zeros[idx])

        # prev joint frame -> this joint frame
        # relative placement from prev to this joint frame
        # derotate first by previous joint's angle
        parent_to_joint: Placement = (unpose(prev_jtype, prev_axis, prev_q)
                                      .multiply(
                                          prev_joint_in_asm
                                          .inverse()
                                          .multiply(joint_in_asm)
                                          ))

        joint_to_part = unpose(jtype, axis, q).multiply(
            joint_in_asm.inverse().multiply(child_obj.Placement))

        # joint limits
        lim_low, lim_high = q_limits_si(j, d)

        joints.append(JointSpec(
            name=j.Label or f"joint{idx:02d}",
            type=jtype,
            parent_to_joint=parent_to_joint,
            axis=axis,
            lim_low=lim_low,
            lim_high=lim_high,
        ))
        links.append(LinkSpec(
            name=child_obj.Name,
            parent_name=parent_obj.Name,
            joint_to_part=joint_to_part
        ))

        prev_joint_in_asm = joint_in_asm
        prev_axis, prev_q, prev_jtype = axis, q, jtype

    # flange - active tool's TCP taken as chain tip (TCP_Placement in World)
    flange_local: Placement = Placement()
    tool: "Optional[App.DocumentObject]" = getattr(robot_obj,
                                                   "Active_tool", None)
    if tool and getattr(tool, "Flange_link", None) and tool.Flange_link[0]:
        # use the active tool's tcp as the chain tip
        tcp_in_asm = p_world_in_asm.multiply(Placement(tool.TCP_placement))
        flange_local = unpose(prev_jtype, prev_axis, prev_q).multiply(
            prev_joint_in_asm.inverse().multiply(tcp_in_asm))

    return ChainSpec(
        base_in_asm=base_in_asm,
        flange_local=flange_local,
        links=links,
        joints=joints,
    )


def jcs_in_asm(p_world_in_asm, ref, jcs_plc):
    """
    joint coord system in asm coords
    getGlobalPlacement*jcs is valid across nested/linked
    sub-asms whereas obj.Placement*jcs is not
    """
    return p_world_in_asm.multiply(
        UtilsAssembly.getGlobalPlacement(ref).multiply(jcs_plc))


def joint_axis(joint_in_asm, jcs2_in_asm, d):
    """
    rotation axis in joint frame
    """
    # Assembly WB convention
    # Revolut axis is local Z of placement 1
    # axis: V3 = V3(0, 0, 1) <- Hardcoded value
    # Ass. WB aligns child JCS (Placement2, incl. Offset2) onto
    # the parent JCS (Placement1) either same dir or z-flipped
    # flipped (z antiparallel): child +q
    # same-dir (z parallel): child -q
    jcs2_in_joint = joint_in_asm.inverse().multiply(jcs2_in_asm)
    z_dot = jcs2_in_joint.Rotation.multVec(V3(0, 0, 1)).z
    return V3(0, 0, (1 if z_dot < 0 else -1) * d)


def joint_value_doc(j, d, z=0.0):
    """
    current joint value
    q = dir * (raw - zero); raw = Offset2 (Yaw | Z)
    """
    jt = joint_type_FC2WB(j.JointType)
    if jt == REVOLUTE:
        return d * (float(j.Offset2.Rotation.toEuler()[0])-z)
    if jt == PRISMATIC:
        return d * (float(j.Offset2.Base.z)-z)
    return 0.0


def q_limits_si(j, d):
    """doc limits -> q-space in SI Units radians/ m"""
    jt = joint_type_FC2WB(j.JointType)
    low, high = joint_limits_doc(j)
    if d == -1:
        low, high = -high, -low
    return q_doc_to_si(jt, low), q_doc_to_si(jt, high)


def unpose(jtype, axis, q_doc):
    """
    inverse of the joint-motion placement
    """
    if axis is None or q_doc == 0:
        return Placement()

    if jtype == PRISMATIC:
        return Placement(axis * (-q_doc), App.Rotation())

    # REVOLUTE
    return Placement(V3(), App.Rotation(axis, -q_doc))


def joint_dirs(robot_obj: "App.DocumentObject") -> List[int]:
    """
    q (angle used by kinematic chains) = dir * yaw (angle stored in fc)
    """
    joints = list(robot_obj.Robot_joints or [])

    # get the joint dirs and 1 pad if not of same length as num joints
    dirs = list(getattr(robot_obj, "Robot_joints_dir", [])
                or [])[:len(joints)]
    dirs += [1] * (len(joints) - len(dirs))

    return [-1 if d < 0 else 1 for d in dirs]


def joint_zeros(robot_obj: "App.DocumentObject") -> List[float]:
    """
    Return the Offset2 values
    corresponding to q = 0 for each joint
    """
    n = len(robot_obj.Robot_joints or ())
    zeros = list(getattr(robot_obj, "Robot_zero_pose", ())[:n])

    if len(zeros) < n:
        zeros.extend([0.0] * (n - len(zeros)))

    return [float(z) for z in zeros]


def joint_limits_doc(j: "App.DocumentObject") -> tuple:
    """
    joint limits in doc units (deg / mm)
    """
    jt = joint_type_FC2WB(j.JointType)
    if jt == PRISMATIC:
        low = (float(j.LengthMin) if getattr(j, "EnableLengthMin", False)
               else -DEFAULT_SLIDER_TRAVEL_MM)
        high = (float(j.LengthMax) if getattr(j, "EnableLengthMax", False)
                else DEFAULT_SLIDER_TRAVEL_MM)
        return low, high

    if jt == FIXED:
        return 0.0, 0.0

    # REVOLUTE

    low = float(j.AngleMin) if getattr(j, "EnableAngleMin", False) else -180
    high = float(j.AngleMax) if getattr(j, "EnableAngleMax", False) else 180

    return low, high


def q_doc_to_si(jtype, v):
    """deg->rad (revolute) | mm->m (prismatic)"""
    return v / MM_PER_M if jtype == PRISMATIC else deg_to_rad(v)


def q_si_to_doc(jtype, v):
    """rad->deg (revolute) | m->mm (prismatic)"""
    return v * MM_PER_M if jtype == PRISMATIC else rad_to_deg(v)
