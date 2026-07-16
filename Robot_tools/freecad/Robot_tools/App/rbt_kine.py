"""
rbt_kine.py
kinematics module for robot wb
"""

from __future__ import annotations

import traceback
from typing import Dict, List, Optional, Tuple, TypeAlias

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import (
    ChainSpec, joint_type_FC2WB, REVOLUTE, PRISMATIC, FIXED)
from freecad.Robot_tools.App.rbt_kine_chain import (
    extract_chain, joint_dirs, joint_limits_doc,
    joint_value_doc, q_doc_to_si, q_si_to_doc,
    joint_zeros)
from freecad.Robot_tools.App.rbt_helpers_math import deg_to_rad
from freecad.Robot_tools.backends import load_kinematics_lib
from freecad.Robot_tools.backends.base import KinematicsBackend
from freecad.Robot_tools.App.rbt_placement import p_asm_in_world
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_KIN_LIB, PIP_HINTS)
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err, fcl_warn

Placement: TypeAlias = App.Placement
Chain: TypeAlias = ChainSpec

# backend instance : (doc.Name, robot.Name, backend library)
cache: Dict[Tuple[str, str, str], KinematicsBackend] = {}
c_pruner = None

# chain cache
chain_cache: Dict[Tuple[str, str], Chain] = {}


def recompute_asm(rbt_obj: "App.DocumentObject") -> None:
    """
        Recomputes robot assembly
    """
    asm = getattr(rbt_obj, "Robot_assembly", None)
    if asm is not None:
        try:
            asm.recompute()
        except Exception as e:
            fcl_err(f"Unable to recompute assembly: {e}")
    else:
        fcl_err("Unable to find attr <Robot_assembly>")


def recompute_tool(rbt_obj: "App.DocumentObject") -> None:
    """
        Recomputes tool object
    """
    tool = getattr(rbt_obj, "Active_tool", None)
    if tool is None:
        return
    try:
        tool.recompute()
    except Exception as e:
        fcl_err(f"Unable to recompute tool: {e}")


def backend_name(rbt_obj: "App.DocumentObject") -> str:
    return getattr(rbt_obj, "Kinematics_lib", DEFAULT_KIN_LIB)


def get_backend(rbt_obj: "App.DocumentObject",
                ) -> Optional[KinematicsBackend]:
    """
    Returns the current backend selected for robot object.\n
    If no active B.E exists, it selects one and caches the value.\n
    Inputs:
        - rbt_obj: Robot FC Object
    """

    # ensure clear cache on earlier doc closures
    ensure_pruner()

    curr_doc = rbt_obj.Document.Name
    name = backend_name(rbt_obj)
    key: Tuple[str, str] = (curr_doc, rbt_obj.Name, name)

    # return existing from cache
    if key in cache:
        return cache[key]

    # if no cached BE exists, init new instance & cache it

    # extract robot kinematic chain after refreshing tcp
    recompute_tool(rbt_obj)
    chain = extract_chain(rbt_obj)
    if chain is None:
        fcl_err("Cant extract robot kinematics info")
        return None

    # create new backend instance
    def load_lib(lib_name: str):
        lib = load_kinematics_lib(lib_name)
        be = lib()
        be.load(chain)
        return be

    # create new backend instance
    # try to load user selection, fallback to DEFAULT_KIN_LIB
    libs = [name] if name == DEFAULT_KIN_LIB else [name, DEFAULT_KIN_LIB]
    be = None
    for lib in libs:
        try:
            be = load_lib(lib)
            break
        except ImportError:
            fcl_warn(f"Kin Lib '{lib}' is not installed.\n")
        except Exception as e:
            fcl_err(f"Failed to load '{lib}': {e}\n")
            return None

    if be is None:
        fcl_err("No kinematics backend available.\n")
        return None

    # update the UI if we used fallback
    if lib != name:
        _ph = PIP_HINTS.get(name, f'pip install {name}')
        fcl_warn(
            f"Falling back to '{lib}' for this session. To use '{name}', "
            f"install it ({_ph}) and restart FreeCAD.\n")

    cache[key] = be
    chain_cache[key] = chain
    return be


def invalidate(rbt_obj: "App.DocumentObject") -> None:
    """
        Removes current backend from cache.
        Inputs:
            - rbt_obj: Robot FC Object
    """
    try:
        curr_doc = rbt_obj.Document.Name
        name = backend_name(rbt_obj)
        cache.pop((curr_doc, rbt_obj.Name, name), None)
        chain_cache.pop((curr_doc, rbt_obj.Name, name), None)
    except Exception as e:
        fcl_err(f"Failed to clear kinematics lib from cache: {e}\n")
        fcl_err(traceback.format_exc())


def curr_joint_vals_doc(rbt_obj: "App.DocumentObject") -> List[float]:
    """
    joint values in doc units, measured from the zero pose
    q = dir * (raw - zero), raw = Offset2 yaw deg | z mm
    """
    return [joint_value_doc(j, d, z)
            for d, z, j in zip(joint_dirs(rbt_obj), joint_zeros(rbt_obj),
                               rbt_obj.Robot_joints)]


def joint_limits_q_deg(rbt_obj, j_idx: int):
    """q-space limits (about the zero pose), direction flipped"""
    low, high = joint_limits_doc(rbt_obj.Robot_joints[j_idx])
    if joint_dirs(rbt_obj)[j_idx] == -1:
        low, high = -high, -low
    return low, high


def jog_q_deg(rbt_obj, q_deg):
    """continuous jog: FK only"""
    apply_joint_angles(rbt_obj, q_deg)
    tool = getattr(rbt_obj, "Active_tool", None)
    if tool:
        tool.recompute()


def raw_pose(rbt_obj) -> List[float]:
    """
    current raw Offset2 values (yaw deg | z mm), dir-less
    """
    return [joint_value_doc(j, 1) for j in rbt_obj.Robot_joints]


def save_home(rbt_obj) -> None:
    """
    save current pose as home position
    """
    rbt_obj.Robot_home_pos = raw_pose(rbt_obj)


def set_zero_pose(rbt_obj) -> None:
    """
    save current pose as zero (null) reference
    """
    rbt_obj.Robot_zero_pose = raw_pose(rbt_obj)


def home_q_deg(rbt_obj) -> List[float]:
    """
    q_home = dir * (stored raw home - zero)
    for unset home pos --> zero pose
    """
    zeros = joint_zeros(rbt_obj)
    home = list(rbt_obj.Robot_home_pos or [])
    home += zeros[len(home):]
    return [d * (float(y)-z)
            for d, y, z in zip(joint_dirs(rbt_obj), home, zeros)]


def get_chain(rbt_obj):
    """
    returns the robot kinematic chain
    """
    curr_doc = rbt_obj.Document.Name
    key = (curr_doc, rbt_obj.Name, backend_name(rbt_obj))
    if key not in chain_cache:
        get_backend(rbt_obj)  # builds & caches
    return chain_cache.get(key)


def apply_joint_angles(rbt_obj, q_deg):
    """
    Use fk to position the parts
    q_deg is in urdf style q convention
    """
    chain = get_chain(rbt_obj)
    if chain is None:
        return
    if len(q_deg) != len(chain.joints):
        fcl_err("joints lengths does not match with kin chain length")
        return

    doc = rbt_obj.Document
    F = App.Placement(chain.base_in_asm)
    for i, joint in enumerate(chain.joints):
        # forward pass transpose
        F = F.multiply(joint.parent_to_joint)

        if joint.type == REVOLUTE:
            F = F.multiply(App.Placement(App.Vector(),
                                         App.Rotation(joint.axis, q_deg[i])))

        elif joint.type == PRISMATIC:
            F = F.multiply(App.Placement(joint.axis * q_deg[i],
                                         App.Rotation()))

        # apply joint to part offset
        part = doc.getObject(chain.links[i+1].name)
        off = chain.links[i+1].joint_to_part

        if part is not None and off is not None:
            part.Placement = F.multiply(off)
            part.purgeTouched()  # prevent assm recompute

    tool = getattr(rbt_obj, "Active_tool", None)
    if tool is not None:
        tool.TCP_placement = (p_asm_in_world(rbt_obj)
                              .multiply(F)
                              .multiply(chain.flange_local))
        # tool.recompute()


def resolve_offsets(rbt_obj, q_deg):
    """
    write robot pose into offset2 using assembly solver
    """
    if q_deg is None:
        return
    apply_joint_angles(rbt_obj, q_deg)
    prefs = App.ParamGet("User parameter:BaseApp/Preferences/Mod/Assembly")
    prev = prefs.GetBool("SolveInJointCreation", True)
    prefs.SetBool("SolveInJointCreation", False)  # avoid calculation at start
    try:
        dirs = joint_dirs(rbt_obj)
        zeros = joint_zeros(rbt_obj)
        for i, joint in enumerate(rbt_obj.Robot_joints):
            jt = joint_type_FC2WB(joint.JointType)
            off = dirs[i]*q_deg[i] + zeros[i]
            if jt == REVOLUTE:
                joint.Offset2 = App.Placement(joint.Offset2.Base,
                                              App.Rotation(off, 0, 0))
            elif jt == PRISMATIC:
                b = joint.Offset2.Base
                joint.Offset2 = App.Placement(App.Vector(b.x, b.y, off),
                                              joint.Offset2.Rotation)
    finally:
        prefs.SetBool("SolveInJointCreation", prev)  # reset to original val

    recompute_asm(rbt_obj)
    recompute_tool(rbt_obj)


def dof_mask(rbt_obj) -> List[bool]:
    """
    true for joints that add a DOF to the chain
    """
    return [j.type != FIXED
            for j in get_chain(rbt_obj).joints]


def dof_types(rbt_obj) -> List[str]:
    """
    Joint types of the active dof joints (compressed chain)
    """
    kin_chain = get_chain(rbt_obj).joints
    return [j.type for j in kin_chain if j.type != FIXED]


def compress_chain(vals, mask):
    """
    compress kinematic chain from full len
    to the one containing only active dof
    """
    return [v for v, m in zip(vals, mask) if m]


def expand_chain(dof_vals, mask, fill):
    """
    expands a kin chaing from being active
    DOF only to the one containing info of
    all the joints, including fixed ones
    """
    it = iter(dof_vals)
    return [next(it) if m else f for m, f in zip(mask, fill)]


def fk(
        rbt_obj: "App.DocumentObject",
        q_deg: Optional[List[float]] = None
        ) -> Optional[Placement]:
    """
        Runs a fwd kinematics pass on current joint angles
        and returns the placement of the TCP.
        Input:
            rbt_obj: Robot FC Object
            q_deg  : Current robot joint angles
    """
    be = get_backend(rbt_obj)
    if be is None:
        return None
    if q_deg is None:
        q_deg = curr_joint_vals_doc(rbt_obj)

    mask = dof_mask(rbt_obj)
    types = dof_types(rbt_obj)
    c_chain = compress_chain(q_deg, mask)
    q_si: List[float] = [q_doc_to_si(t, v)
                         for t, v in
                         zip(types, c_chain)]
    try:
        # return fk in world-frame
        return p_asm_in_world(rbt_obj).multiply(be.fk(q_si))
    except Exception as e:
        fcl_err(f"FK failed: {e}")
        return None


def ik(
        rbt_obj: "App.DocumentObject",
        target: Placement,
        q_seed_deg: Optional[List[float]] = None,
        pos_tol_mm: float = 0.01,
        rot_tol_deg: float = 0.5,
        ) -> Optional[List[float]]:
    """
        Runs inverse kinematic pass and returns
        the list of joint angles.
        Inputs:
            - rbt_obj: Robot FC Object
            - target: Target TCP placement
            - q_seed_deg: Initial joint angles for IK solver
            - pos_tol_mm: Accuracy tolerance in position (millimeters)
            - rot_tol_deg: Accuracy tolerance in orientation (degrees)
    """
    be = get_backend(rbt_obj)
    if be is None:
        return None

    # take last valid joint angles or current angles when
    # no initial values are provided in input
    if q_seed_deg is None:
        q_seed_deg = curr_joint_vals_doc(rbt_obj)

    # filter out fixed/active joints & convert deg -> rad
    mask = dof_mask(rbt_obj)
    types = dof_types(rbt_obj)
    c_chain = compress_chain(q_seed_deg, mask)
    q_seed_rad: List[float] = [q_doc_to_si(t, v)
                               for t, v in
                               zip(types, c_chain)]

    # ik pass
    try:
        target_in_asm = (p_asm_in_world(rbt_obj)
                         .inverse()
                         .multiply(target))
        q_rad: Optional[List[float]] = be.ik(
            target_in_asm, q_seed_rad,
            pos_tol=pos_tol_mm / 1000.0,
            rot_tol=deg_to_rad(rot_tol_deg),
        )
    except Exception as e:
        fcl_err(f"IK failed: {e}\n")
        return None
    if q_rad is None:
        return None

    # apply back the fixed joints & convert back rad -> deg
    q_deg: List[float] = expand_chain([q_si_to_doc(t, v)
                                       for t, v in
                                       zip(types, q_rad)],
                                      mask, q_seed_deg)

    return q_deg


class CachePruner:
    """
    Drops cache entries for deleted robots / closed documents
    """
    def slotDeletedDocument(self, doc):
        purge(lambda k: k[0] == doc.Name)

    def slotDeletedObject(self, obj):
        doc = getattr(obj, "Document", None)
        if doc is None:
            return
        purge(lambda k: k[0] == doc.Name and k[1] == obj.Name)


def purge(match):
    for d in (cache, chain_cache):
        for k in [k for k in d if match(k)]:
            del d[k]


def ensure_pruner():
    global c_pruner
    if c_pruner is None:
        c_pruner = CachePruner()
        App.addDocumentObserver(c_pruner)
