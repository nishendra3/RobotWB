"""Tesseract Robotics backend for robot_tools kinematics.

Mirrors KinematicsBackend. Generates URDF + SRDF + kinematics_plugin_config
YAML to a temp directory and lets Tesseract's plugin factory load them via
`env.init(FilesystemPath, FilesystemPath, GeneralResourceLocator)`.
"""

from __future__ import annotations

import atexit
import io
import os
import shutil
import tempfile
from typing import List, Optional, Tuple

import numpy as np
import FreeCAD
from FreeCAD import Placement, Rotation, Vector

from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    Translation3d,
    Quaterniond,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import (
    KinGroupIKInput,
    KinGroupIKInputs,
)

from freecad.Robot_tools.App.rbt_kine_types import ChainSpec


MM_PER_M = 1000.0
_GROUP = "manipulator"
_FLANGE_LINK = "flange"


class TesseractBackend:
    name: str = "tesseract"

    def __init__(self) -> None:
        self._env: Optional[Environment] = None
        self._kg = None
        self._chain: Optional[ChainSpec] = None
        self._base_link: str = ""
        self._tip_link: str = _FLANGE_LINK
        self._tmpdir: Optional[str] = None

    # ---- KinematicsBackend Protocol ----

    def load(self, chain: ChainSpec) -> None:
        if not chain.links:
            raise RuntimeError("ChainSpec has no links")
        self._chain = chain
        self._base_link = chain.links[0].name

        tmpdir = tempfile.mkdtemp(prefix="robot_tools_tess_")
        self._tmpdir = tmpdir
        atexit.register(_safe_rmtree, tmpdir)

        urdf_path = os.path.join(tmpdir, "robot.urdf")
        yaml_path = os.path.join(tmpdir, "kinematics.yaml")
        srdf_path = os.path.join(tmpdir, "robot.srdf")
        yaml_uri = "file:///" + yaml_path.replace(os.sep, "/")

        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write(_chain_to_urdf_string(chain, self._base_link))
        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(_kinematics_yaml(self._base_link, self._tip_link, _GROUP))
        with open(srdf_path, "w", encoding="utf-8") as f:
            f.write(
                _chain_to_srdf_string(
                    self._base_link, self._tip_link, _GROUP, yaml_uri
                )
            )

        env = Environment()
        rl = GeneralResourceLocator()
        ok = env.init(FilesystemPath(urdf_path), FilesystemPath(srdf_path), rl)
        if not ok:
            raise RuntimeError("env.init failed for generated URDF/SRDF")
        kg = env.getKinematicGroup(_GROUP)
        if kg is None:
            raise RuntimeError(
                f"KinematicGroup {_GROUP!r} not registered "
                "(check kinematics.yaml syntax / search_libraries)"
            )
        self._env = env
        self._kg = kg

    def fk(self, q_rad: List[float]) -> Placement:
        assert self._chain is not None and self._kg is not None
        q = np.asarray(q_rad, dtype=float)
        flange_in_base = _iso3_to_placement(
            self._kg.calcFwdKin(q)[self._tip_link]
        )
        return self._chain.base_world.multiply(flange_in_base)

    def ik(
        self,
        target: Placement,
        q_seed_rad: List[float],
        pos_tol: float = 1e-4,
        rot_tol: float = 1e-3,
        max_iter: int = 50,
    ) -> Optional[List[float]]:
        assert self._chain is not None and self._kg is not None
        target_in_base = self._chain.base_world.inverse().multiply(target)
        target_iso = _placement_to_iso3(target_in_base)

        ik_input = KinGroupIKInput()
        ik_input.pose = target_iso
        ik_input.tip_link_name = self._tip_link
        ik_input.working_frame = self._base_link
        ik_inputs = KinGroupIKInputs()
        ik_inputs.append(ik_input)

        seed = np.asarray(q_seed_rad, dtype=float)
        try:
            solutions = self._kg.calcInvKin(ik_inputs, seed)
        except Exception as e:
            FreeCAD.Console.PrintWarning(f"[tesseract] ik() raised: {e}\n")
            return None
        if solutions is None or len(solutions) == 0:
            return None
        return list(np.asarray(solutions[0]).flatten())

    def jacobian(self, q_rad: List[float]) -> Optional[np.ndarray]:
        assert self._kg is not None
        q = np.asarray(q_rad, dtype=float)
        return np.asarray(self._kg.calcJacobian(q, self._tip_link))


# --------------------------------------------------------------------------
# Helpers (private to this module)
# --------------------------------------------------------------------------

def _safe_rmtree(path: str) -> None:
    try:
        shutil.rmtree(path, ignore_errors=True)
    except Exception:
        pass


def _placement_to_iso3(pl: Placement) -> Isometry3d:
    # FreeCAD Rotation.Q = (x, y, z, w). Quaterniond constructor is (w, x, y, z).
    qx, qy, qz, qw = pl.Rotation.Q
    tx = pl.Base.x / MM_PER_M
    ty = pl.Base.y / MM_PER_M
    tz = pl.Base.z / MM_PER_M
    return (
        Isometry3d.Identity()
        * Translation3d(tx, ty, tz)
        * Quaterniond(qw, qx, qy, qz)
    )


def _iso3_to_placement(iso: Isometry3d) -> Placement:
    t = np.asarray(iso.translation()).flatten()
    R = np.asarray(iso.rotation())
    q = Quaterniond(R)
    # FreeCAD Rotation(x, y, z, w)
    return Placement(
        Vector(float(t[0]) * MM_PER_M, float(t[1]) * MM_PER_M, float(t[2]) * MM_PER_M),
        Rotation(q.x(), q.y(), q.z(), q.w()),
    )


def _placement_to_rpy(pl: Placement) -> Tuple[float, float, float]:
    yaw, pitch, roll = pl.Rotation.getYawPitchRoll()
    return (np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))


def _chain_to_urdf_string(chain: ChainSpec, base_link: str) -> str:
    buf = io.StringIO()
    buf.write('<?xml version="1.0"?>\n')
    buf.write('<robot name="robot_tools_chain">\n')

    for lk in chain.links:
        buf.write(f'  <link name="{lk.name}"/>\n')
    buf.write(f'  <link name="{_FLANGE_LINK}"/>\n')

    n_links = len(chain.links)
    for idx, j in enumerate(chain.joints):
        if idx + 1 >= n_links:
            FreeCAD.Console.PrintWarning(
                f"[tesseract] joint {j.name!r} has no child link, skipped\n"
            )
            continue
        parent_name = chain.links[idx].name
        child_name = chain.links[idx + 1].name
        T = j.parent_to_joint
        xyz = (T.Base.x / MM_PER_M, T.Base.y / MM_PER_M, T.Base.z / MM_PER_M)
        rpy = _placement_to_rpy(T)
        jtype = "revolute" if j.type == "revolute" else "fixed"

        buf.write(f'  <joint name="{j.name}" type="{jtype}">\n')
        buf.write(f'    <parent link="{parent_name}"/>\n')
        buf.write(f'    <child link="{child_name}"/>\n')
        buf.write(
            f'    <origin xyz="{xyz[0]:.9g} {xyz[1]:.9g} {xyz[2]:.9g}" '
            f'rpy="{rpy[0]:.9g} {rpy[1]:.9g} {rpy[2]:.9g}"/>\n'
        )
        if jtype == "revolute":
            ax = j.axis
            buf.write(
                f'    <axis xyz="{float(ax.x):.9g} {float(ax.y):.9g} {float(ax.z):.9g}"/>\n'
            )
            buf.write(
                f'    <limit lower="{j.lim_low:.9g}" upper="{j.lim_high:.9g}" '
                f'effort="100" velocity="1"/>\n'
            )
        buf.write('  </joint>\n')

    last_link = chain.links[-1].name
    F = chain.flange_local
    fxyz = (F.Base.x / MM_PER_M, F.Base.y / MM_PER_M, F.Base.z / MM_PER_M)
    frpy = _placement_to_rpy(F)
    buf.write('  <joint name="flange_fixed" type="fixed">\n')
    buf.write(f'    <parent link="{last_link}"/>\n')
    buf.write(f'    <child link="{_FLANGE_LINK}"/>\n')
    buf.write(
        f'    <origin xyz="{fxyz[0]:.9g} {fxyz[1]:.9g} {fxyz[2]:.9g}" '
        f'rpy="{frpy[0]:.9g} {frpy[1]:.9g} {frpy[2]:.9g}"/>\n'
    )
    buf.write('  </joint>\n')

    buf.write('</robot>\n')
    return buf.getvalue()


def _chain_to_srdf_string(
    base_link: str, tip_link: str, group_name: str, yaml_uri: str
) -> str:
    return (
        '<?xml version="1.0"?>\n'
        '<robot name="robot_tools_chain">\n'
        f'  <group name="{group_name}">\n'
        f'    <chain base_link="{base_link}" tip_link="{tip_link}"/>\n'
        '  </group>\n'
        f'  <kinematics_plugin_config filename="{yaml_uri}"/>\n'
        '</robot>\n'
    )


def _kinematics_yaml(base_link: str, tip_link: str, group_name: str) -> str:
    return (
        "kinematic_plugins:\n"
        "  search_libraries:\n"
        "    - tesseract_kinematics_kdl_factories\n"
        "  fwd_kin_plugins:\n"
        f"    {group_name}:\n"
        "      default: KDLFwdKinChain\n"
        "      plugins:\n"
        "        KDLFwdKinChain:\n"
        "          class: KDLFwdKinChainFactory\n"
        "          config:\n"
        f"            base_link: {base_link}\n"
        f"            tip_link: {tip_link}\n"
        "  inv_kin_plugins:\n"
        f"    {group_name}:\n"
        "      default: KDLInvKinChainLMA\n"
        "      plugins:\n"
        "        KDLInvKinChainLMA:\n"
        "          class: KDLInvKinChainLMAFactory\n"
        "          config:\n"
        f"            base_link: {base_link}\n"
        f"            tip_link: {tip_link}\n"
    )