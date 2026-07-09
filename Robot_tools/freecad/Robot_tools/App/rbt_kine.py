"""
rbt_kine.py
kinematics module for robot wb
"""

from __future__ import annotations

import traceback
from typing import Dict, List, Optional, Tuple, TypeAlias

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import ChainSpec
from freecad.Robot_tools.App.rbt_kine_chain import (
    extract_chain, joint_dirs, doc_limits_deg)
from freecad.Robot_tools.App.rbt_helpers_math import deg_to_rad, rad_to_deg
from freecad.Robot_tools.backends import load_kinematics_lib
from freecad.Robot_tools.backends.base import KinematicsBackend
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


def extract_chain_at_zeropos(rbt_obj: "App.DocumentObject"):
    """
        Run extract_chain with every joint forced to q=0
    """
    joints = list(rbt_obj.Robot_joints or [])
    saved = [App.Placement(j.Offset2) for j in joints]  # cache for reset
    try:
        for j in joints:
            j.Offset2 = App.Placement()  # set q=0

        recompute_asm(rbt_obj)
        recompute_tool(rbt_obj)

        return extract_chain(rbt_obj)

    except Exception as e:
        fcl_err(f"Unable to set robot to neutral Q=0 pos: {e}")

    finally:
        for j, plc in zip(joints, saved):
            j.Offset2 = plc  # reset back to original vals

        recompute_asm(rbt_obj)
        recompute_tool(rbt_obj)


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
    if tool is not None:
        try:
            tool.recompute()
        except Exception as e:
            fcl_err(f"Unable to recompute tool: {e}")
    else:
        fcl_err("Unable to find attr <Active_tool>")


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

    # extract robot kinematic chain at all neutral angles (q=0)
    chain = extract_chain_at_zeropos(rbt_obj)
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


def current_q_deg(rbt_obj: "App.DocumentObject") -> List[float]:
    """
        Canonical URDF style joint angles
        q = direction * FC Offset2 based yaw
    """
    out: List[float] = []
    for d, j in zip(joint_dirs(rbt_obj), rbt_obj.Robot_joints):
        # offset2 has curr rotation. Z component is what
        # AnimationController writes via Rotation(angle, 0, 0)
        # toEuler() returns (yaw, pitch, roll) in deg
        # TODO: FIX toEuler warp angles to +-180deg
        yaw, _, _ = j.Offset2.Rotation.toEuler()
        out.append(d * float(yaw))
    return out


def set_q_deg(rbt_obj, j_idx: int, q: float) -> None:
    """
    canonical urdf style angle q
    fc's yaw = dir * q
    or canonical q = dir * yaw
    As dir is +-1, above equations are same
    """
    d = joint_dirs(rbt_obj)[j_idx]
    joint = rbt_obj.Robot_joints[j_idx]
    joint.Offset2 = App.Placement(App.Vector(),
                                  App.Rotation(d*q, 0, 0))
    joint.recompute()
    tool = getattr(rbt_obj, "Active_tool", None)
    if tool:
        tool.recompute()


def joint_limits_q_deg(rbt_obj, j_idx: int):
    """
    q-space limits, considering direction into account
    """
    low, high = doc_limits_deg(rbt_obj.Robot_joints[j_idx])
    if joint_dirs(rbt_obj)[j_idx] == -1:
        low, high = -high, -low
    return low, high


def save_home(rbt_obj) -> None:
    """Robot_home_pos in FC yaw convention"""
    rbt_obj.Robot_home_pos = [float(j.Offset2.Rotation.toEuler()[0])
                              for j in rbt_obj.Robot_joints]


def home_q_deg(rbt_obj) -> List[float]:
    """q_home = dir * stored yaw home pos"""
    return [d * float(y) for d, y in zip(joint_dirs(rbt_obj),
                                         rbt_obj.Robot_home_pos or [])]


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

    doc = rbt_obj.Document
    F = App.Placement(chain.base_world)
    for i, joint in enumerate(chain.joints):
        # forward pass transpose
        F = F.multiply(joint.parent_to_joint)

        if joint.type == "revolute":
            F = F.multiply(App.Placement(App.Vector(),
                                         App.Rotation(joint.axis, q_deg[i])))

        # apply joint to part offset
        part = doc.getObject(chain.links[i+1].name)
        off = chain.links[i+1].joint_to_part

        if part is not None and off is not None:
            part.Placement = F.multiply(off)
            part.purgeTouched()  # prevent assm recompute

    tool = getattr(rbt_obj, "Active_tool", None)
    if tool is not None:
        tool.TCP_placement = F.multiply(chain.flange_local)
        # tool.recompute()


def resolve_offsets(rbt_obj, q_deg):
    """
    write robot pose into offset2 using assembly solver
    """
    if q_deg is None:
        return
    prefs = App.ParamGet("User parameter:BaseApp/Preferences/Mod/Assembly")
    prev = prefs.GetBool("SolveInJointCreation", True)
    prefs.SetBool("SolveInJointCreation", False)  # avoid calculation at start
    try:
        dirs = joint_dirs(rbt_obj)
        for i, joint in enumerate(rbt_obj.Robot_joints):
            joint.Offset2 = App.Placement(App.Vector(),
                                          App.Rotation(dirs[i]*q_deg[i], 0, 0))
    finally:
        prefs.SetBool("SolveInJointCreation", prev)  # reset to original val

    recompute_asm(rbt_obj)
    recompute_tool(rbt_obj)


def dof_mask(rbt_obj) -> List[bool]:
    """
    true for joints that add a DOF to the chain
    only true for revolute joints for now
    """
    return [j.type == "revolute"
            for j in get_chain(rbt_obj).joints]


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
        q_deg = current_q_deg(rbt_obj)

    mask: List[bool] = dof_mask(rbt_obj)
    q_rad: List[float] = [deg_to_rad(v)
                          for v in compress_chain(q_deg, mask)]
    try:
        return be.fk(q_rad)
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
        q_seed_deg = current_q_deg(rbt_obj)

    # filter out fixed/active joints & convert deg -> rad
    mask = dof_mask(rbt_obj)
    q_seed_rad: List[float] = [deg_to_rad(v)
                               for v in compress_chain(q_seed_deg, mask)]

    # ik pass
    try:
        q_rad: Optional[List[float]] = be.ik(
            target, q_seed_rad,
            pos_tol=pos_tol_mm / 1000.0,
            rot_tol=deg_to_rad(rot_tol_deg),
        )
    except Exception as e:
        fcl_err(f"IK failed: {e}\n")
        return None
    if q_rad is None:
        return None

    # apply back the fixed joints & convert back rad -> deg
    q_deg: List[float] = expand_chain([rad_to_deg(v) for v in q_rad],
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
