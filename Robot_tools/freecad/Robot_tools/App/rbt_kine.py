"""
rbt_kine.py
kinematics module for robot wb
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from contextlib import contextmanager
from typing import List, Optional, TypeAlias

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import (
    ChainSpec, joint_type_FC2WB, REVOLUTE, PRISMATIC, FIXED)
from freecad.Robot_tools.App.rbt_kine_chain import (
    extract_chain, joint_dirs, joint_limits_doc,
    joint_value_doc, q_doc_to_si, q_si_to_doc,
    joint_zeros)
from freecad.Robot_tools.App.rbt_kine_joints import (
    JointCfg, load_cfg_map, save_cfg_map)
from freecad.Robot_tools.App.rbt_helpers_math import deg_to_rad
from freecad.Robot_tools.backends import load_kinematics_lib
from freecad.Robot_tools.backends.base import KinematicsBackend
from freecad.Robot_tools.App.rbt_placement import (
    p_asm_in_world, p_asm_to_world)
from freecad.Robot_tools.App.rbt_global_constants import (
    DEFAULT_KIN_LIB, PIP_HINTS, DEFAULT_MAX_SPEEDS)
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err, fcl_warn
from freecad.Robot_tools.App.rbt_errors import RbtInputError

Placement: TypeAlias = App.Placement
Chain: TypeAlias = ChainSpec


@dataclass
class KinState:
    """
    kinematics state to store on the robot object proxy
    """
    lib: Optional[str] = None
    backend: Optional[KinematicsBackend] = None
    chain: Optional[Chain] = None


def get_kin_state(rbt_obj) -> KinState:
    """
    create new or return existing kinematic state
    """
    proxy = rbt_obj.Proxy
    ks = getattr(proxy, "kin", None)
    if ks is None:
        ks = KinState()
        proxy.kin = ks
    return ks


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


def load_backend(lib_name: str, chain: Chain) -> Optional[KinematicsBackend]:
    """
    load and init a kinematics backend
    """
    try:
        lib = load_kinematics_lib(lib_name)
        be = lib()
        be.load(chain)
        return be
    except ImportError:
        fcl_warn(f"Kin Lib '{lib_name}' is not installed.\n")
    except Exception as e:
        fcl_err(f"Failed to load '{lib_name}': {e}\n")
        return None

    return None


def get_backend(rbt_obj: "App.DocumentObject",
                ) -> Optional[KinematicsBackend]:
    """
    Returns the current backend selected for robot object.\n
    If no active B.E exists, it selects one and keeps it in the proxy.\n
    """

    ks = get_kin_state(rbt_obj)
    name = backend_name(rbt_obj)

    # return exsiting state
    if ks.backend is not None and ks.lib == name:
        return ks.backend

    # extract robot kinematic chain after refreshing tcp
    recompute_tool(rbt_obj)
    chain = extract_chain(rbt_obj)
    if chain is None:
        fcl_err("Cant extract robot kinematics info")
        return None
    ks.chain = chain

    # try to load user selection, fallback to DEFAULT_KIN_LIB
    libs = [name] if name == DEFAULT_KIN_LIB else [name, DEFAULT_KIN_LIB]

    be = None
    lib = None

    for candidate in libs:
        be = load_backend(candidate, chain)
        if be is not None:
            lib = candidate
            break

    if be is None:
        fcl_err("No kinematics backend available.\n")
        return None

    # Update UI on change of lib
    if lib != name:
        pip_hint = PIP_HINTS.get(name, f"pip install {name}")
        fcl_warn(
            f"Falling back to '{lib}' for this session. "
            f"To use '{name}', install it ({pip_hint}) and restart FreeCAD.\n"
        )

    ks.backend = be
    ks.lib = name

    return be


def invalidate(rbt_obj: "App.DocumentObject") -> None:
    """
        Drops the kinematics state of robot
    """
    proxy = getattr(rbt_obj, "Proxy", None)
    if proxy is not None:
        proxy.kin = None


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


def check_q(rbt_obj, q_deg) -> List[float]:
    """
    validate length against Robot_joints
    """
    n = len(rbt_obj.Robot_joints)
    if len(q_deg) != n:
        raise RbtInputError(f"need {n} joint values, got {len(q_deg)}")
    return [float(v) for v in q_deg]


def clamp_q(rbt_obj, q_deg) -> List[float]:
    """
    clamp to joint limits [FIXED : (0, 0)]
    """
    lims = [joint_limits_q_deg(rbt_obj, i) for i in range(len(q_deg))]
    return [min(max(v, lo), hi) for v, (lo, hi) in zip(q_deg, lims)]


def set_q(rbt_obj, q_deg, clamp=False, preview=False) -> List[float]:
    """
    write path for joint values
    """
    q = check_q(rbt_obj, q_deg)
    if clamp:
        q = clamp_q(rbt_obj, q)
    if preview:
        jog_q_deg(rbt_obj, q)
    else:
        resolve_offsets(rbt_obj, q)
    return q


def move_to(rbt_obj, target, q_seed=None, clamp=False, preview=False,
            pos_tol_mm=0.01, rot_tol_deg=0.5) -> Optional[List[float]]:
    """
    ik + set_q
    """
    q = ik_tcp_in_world(rbt_obj, target, q_seed_deg=q_seed,
                        pos_tol_mm=pos_tol_mm, rot_tol_deg=rot_tol_deg)
    if q is None:
        return None
    return set_q(rbt_obj, q, clamp=clamp, preview=preview)


def jog_q_deg(rbt_obj, q_deg):
    """continuous jog: FK only"""
    apply_joint_angles(rbt_obj, q_deg)
    tool = getattr(rbt_obj, "Active_tool", None)
    if tool:
        tool.recompute()


def save_home(rbt_obj) -> None:
    """
    save current pose as home position
    """
    m = load_cfg_map(rbt_obj)
    for j in rbt_obj.Robot_joints:
        m[j.Name] = replace(m.get(j.Name, JointCfg()),
                            home=joint_value_doc(j, 1))
    save_cfg_map(rbt_obj, m)


def set_zero_pose(rbt_obj) -> None:
    """
    save current pose as zero (null) reference
    """
    m = load_cfg_map(rbt_obj)
    for j in rbt_obj.Robot_joints:
        m[j.Name] = replace(m.get(j.Name, JointCfg()),
                            zero=joint_value_doc(j, 1))
    save_cfg_map(rbt_obj, m)


def home_q_deg(rbt_obj) -> List[float]:
    """
    q_home = dir * (stored raw home - zero)
    for unset home pos --> zero pose
    """
    m = load_cfg_map(rbt_obj)
    out = []
    for j in rbt_obj.Robot_joints:
        cfg = m.get(j.Name, JointCfg())
        d = -1 if cfg.dir < 0 else 1
        out.append(d * (float(cfg.home) - float(cfg.zero)))
    return out


def get_chain(rbt_obj):
    """
    returns the robot kinematic chain
    """
    ks = get_kin_state(rbt_obj)
    if ks.chain is None:
        # build the kinematics state & store it
        get_backend(rbt_obj)
    return ks.chain


def apply_joint_angles(rbt_obj, q_deg):
    """
    Use fk to position the parts
    q_deg is in urdf style q convention
    """
    chain = get_chain(rbt_obj)
    if chain is None:
        return
    if len(q_deg) != len(chain.joints):
        raise RbtInputError(f"need {len(chain.joints)} joint "
                            f"values, got {len(q_deg)}")

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

    # update the tool placement
    tool = getattr(rbt_obj, "Active_tool", None)
    if tool is not None:
        tool.TCP_placement = (p_asm_in_world(rbt_obj)
                              .multiply(F)
                              .multiply(chain.flange_local))
        # tool.recompute()

    # update the joint markers
    if App.GuiUp:
        for jnt in (rbt_obj.Robot_joints or []):
            proxy = getattr(jnt.ViewObject, "Proxy", None)
            if hasattr(proxy, "redrawJointPlacements"):
                proxy.redrawJointPlacements(jnt)


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
        with hold_part_placements(rbt_obj.Robot_assembly):
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
    chain = get_chain(rbt_obj)
    if chain is None:
        return []
    return [j.type != FIXED for j in chain.joints]


def dof_types(rbt_obj) -> List[str]:
    """
    Joint types of the active dof joints (compressed chain)
    """
    chain = get_chain(rbt_obj)
    if chain is None:
        return []
    return [j.type for j in chain.joints if j.type != FIXED]


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


def _fk_tcp_in_asm(rbt_obj, q_doc: List[float]) -> Optional[Placement]:
    """
    FK to the TCP in asm coords (RCS)
    in: robot fpo, joint values in doc units
    out: tcp placement in robot-asm coords / None
    """
    chain = get_chain(rbt_obj)
    if chain is None:
        return None
    if len(q_doc) != len(chain.joints):
        fcl_err("Joints length does not match kinematic chain length")
        return None

    F = App.Placement(chain.base_in_asm)
    for i, joint in enumerate(chain.joints):
        # forward pass
        F = F.multiply(joint.parent_to_joint)
        if joint.type == REVOLUTE:
            t = App.Vector()
            r = App.Rotation(joint.axis, q_doc[i])
            F = F.multiply(App.Placement(t, r))
        elif joint.type == PRISMATIC:
            t = joint.axis * q_doc[i]
            r = App.Rotation()
            F = F.multiply(App.Placement(t, r))

    return F.multiply(chain.flange_local)


def fk_tcp_in_world(rbt_obj, q_doc: Optional[List[float]] = None
                    ) -> Optional[Placement]:
    """
    FK to the TCP in world coords
    """
    q = curr_joint_vals_doc(rbt_obj) if q_doc is None \
        else check_q(rbt_obj, q_doc)
    plc_asm = _fk_tcp_in_asm(rbt_obj, q)
    return None if plc_asm is None else p_asm_to_world(rbt_obj, plc_asm)


def _ik_tcp_in_asm(
        rbt_obj: "App.DocumentObject",
        target_in_asm: Placement,
        q_seed_deg: Optional[List[float]] = None,
        pos_tol_mm: float = 0.01,
        rot_tol_deg: float = 0.5,
        ) -> Optional[List[float]]:
    """
        Runs inverse kinematic pass in the assembly coordinate frame
        attached to base link of the robot.
        Returns the list of joint angles.
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


def ik_tcp_in_world(
        rbt_obj: "App.DocumentObject",
        target_in_world: Placement,
        q_seed_deg: Optional[List[float]] = None,
        pos_tol_mm: float = 0.01,
        rot_tol_deg: float = 0.5,
        ) -> Optional[List[float]]:
    """
        Runs inverse kinematic pass in the world coordinate frame.
        Returns the list of joint angles.
        Inputs:
            - rbt_obj: Robot FC Object
            - target: Target TCP placement
            - q_seed_deg: Initial joint angles for IK solver
            - pos_tol_mm: Accuracy tolerance in position (millimeters)
            - rot_tol_deg: Accuracy tolerance in orientation (degrees)
    """
    target_in_asm = (p_asm_in_world(rbt_obj)
                     .inverse().multiply(target_in_world))
    return _ik_tcp_in_asm(rbt_obj, target_in_asm, q_seed_deg,
                          pos_tol_mm, rot_tol_deg)


@contextmanager
def hold_part_placements(asm):
    """
    Freeze link placements across joint-property writes
    FC "matchJCS" applies a world frame transform to
    assembly local placements, which divergers when
    asm.Placement != identity
    """
    snap = [(o, App.Placement(o.Placement))
            for o in asm.Group if o.isDerivedFrom("App::Link")]
    try:
        yield
    finally:
        for o, plc in snap:
            if not o.Placement.isSame(plc, 1e-9):
                o.Placement = plc
                o.purgeTouched()


def joint_max_speeds(rbt_obj) -> List[float]:
    """
    per-joint full speed (use defaults if missing)
    in: robot_fpo
    out: List[deg/sec (revolute)|mm/sec (prismatic)|0 (fixed)]
    """
    stored = list(getattr(rbt_obj, "Robot_joints_max_speed", None) or [])
    speeds: List[float] = []
    for i, jnt in enumerate(rbt_obj.Robot_joints):
        if i < len(stored) and stored[i] > 0.0:
            speeds.append(float(stored[i]))
            continue
        jtype = joint_type_FC2WB(jnt.JointType)
        if jtype == REVOLUTE:
            speeds.append(DEFAULT_MAX_SPEEDS[REVOLUTE])
        elif jtype == PRISMATIC:
            speeds.append(DEFAULT_MAX_SPEEDS[PRISMATIC])
        else:
            speeds.append(0.0)

    return speeds
