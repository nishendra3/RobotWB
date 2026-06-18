"""ikpy backend for robot_tools kinematics.

Pure-Python FK/IK using `ikpy`. Builds the chain programmatically from
ChainSpec — no URDF round-trip. Matches the KinematicsBackend Protocol
and mirrors the Pinocchio backend's coordinate conventions
(base_world pre/post-multiplication, flange_local at the tip).
"""

from __future__ import annotations

from typing import List, Optional, TypeAlias

import numpy as np  # type: ignore
import FreeCAD as App  # type: ignore

import ikpy.chain as _ik_chain  # type: ignore
import ikpy.link as _ik_link  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import ChainSpec
from freecad.Robot_tools.backends.base import (
    placement_to_matrix4, matrix4_to_placement,
)

MM_PER_M = 1000.0

Placement: TypeAlias = App.Placement
fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning
fcl_err = App.Console.PrintError


class IkpyBackend:
    name: str = "ikpy"

    def __init__(self) -> None:

        # internal FreeCAD kinematic chain
        self._chain: Optional[ChainSpec] = None

        # Chain class from ikpy library
        self._ikc: Optional[_ik_chain.Chain] = None

        # list of boolean indicating that whether or not the corresponding
        # link is active
        self._active_mask: List[bool] = []

        # degrees of freedom based on current active links
        self._n_dof: int = 0

    # ---- KinematicsBackend Protocol ----

    def load(self, chain: ChainSpec) -> None:
        if not chain.joints:
            raise RuntimeError("ChainSpec has no joints")
        self._chain = chain

        # Init the links with the link at the origin of the robot
        links: List[_ik_link.Link] = [_ik_link.OriginLink()]
        mask: List[bool] = [False]  # OriginLink is assumed fixed

        # Note => In IKPY "Link" term is used for "Joints"
        # IKPY URDF Link representation
        # ikpy.link.URDFLink(
        # name, -> (str) name of the link
        # origin_translation, -> (np.array) translation vector "xyz"
        # origin_orientation, -> (np.array) orientation vector "rpy"
        # rotation, -> (np.array) rotation axis of the link (joint)
        # joint_type, -> (str) "type" attribute of the joint (rev./pris./fix.)
        # .. others see doc)

        # convert from FreeCAD to IKPY link notation
        for j in chain.joints:
            if j.type != "revolute":
                # Treat fixed joints as fixed URDFLinks (no rotation axis).
                links.append(_ik_link.URDFLink(
                    name=j.name,
                    origin_translation=_pl_translation_m(j.parent_to_joint),
                    origin_orientation=_pl_rpy(j.parent_to_joint),
                    rotation=None,
                    joint_type="fixed",
                ))
                mask.append(False)  # mark this joint as inactive
                continue

            links.append(_ik_link.URDFLink(
                name=j.name,
                origin_translation=_pl_translation_m(j.parent_to_joint),
                origin_orientation=_pl_rpy(j.parent_to_joint),
                rotation=[float(j.axis.x), float(j.axis.y), float(j.axis.z)],
                bounds=(float(j.lim_low), float(j.lim_high)),
                joint_type="revolute",
            ))
            mask.append(True)

        # Append the flange as a fixed link past the last joint.
        F = chain.flange_local
        links.append(_ik_link.URDFLink(
            name="flange",
            origin_translation=_pl_translation_m(F),
            origin_orientation=_pl_rpy(F),
            rotation=None,
            joint_type="fixed",
        ))
        mask.append(False)

        # make a IKPY Chain object
        self._ikc = _ik_chain.Chain(
            name="robot_tools_chain",
            links=links,
            active_links_mask=mask,
        )

        # update the mask list and current DoF
        self._active_mask = mask
        self._n_dof = sum(1 for m in mask if m)

    def fk(self, q_rad: List[float]) -> Placement:
        assert self._chain is not None and self._ikc is not None
        q_full = self._expand(q_rad)
        T_m = self._ikc.forward_kinematics(q_full)   # 4x4 in base-local meters
        flange_in_base = matrix4_to_placement(T_m)
        return self._chain.base_world.multiply(flange_in_base)

    def ik(
        self,
        target: Placement,
        q_seed_rad: List[float],
        pos_tol: float = 1e-4,
        rot_tol: float = 1e-3,
        max_iter: int = 50,
    ) -> Optional[List[float]]:

        assert self._chain is not None and self._ikc is not None
        target_in_base = self._chain.base_world.inverse().multiply(target)
        T_m = placement_to_matrix4(target_in_base)
        target_pos = T_m[0:3, 3]
        target_rot = T_m[0:3, 0:3]
        # target_x = target_rot[:, 0]  # X Axis of target frame
        # target_y = target_rot[:, 1]  # Y Axis of target frame
        target_z = target_rot[:, 2]  # Z Axis of target frame
        q_seed_full = self._expand(q_seed_rad)

        try:
            q_sol_full = self._ikc.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_z,
                orientation_mode=None,  # "all",
                initial_position=q_seed_full,
                max_iter=max_iter,
            )
        except Exception as e:
            fcl_warn(f"[ikpy] inverse_kinematics raised: {e}\n")
            return None

        # Verify achieved pose against tolerances (ikpy returns the best
        # it found, even if it didn't converge — we filter here).
        achieved = matrix4_to_placement(
            self._ikc.forward_kinematics(q_sol_full)
        )
        dp = (achieved.Base - target_in_base.Base).Length / MM_PER_M  # meters
        # for now we just check if the translation is achieved
        # TODO: extend the check for orientation too
        # rel = target_in_base.Rotation.inverted().multiply(achieved.Rotation)
        if dp > pos_tol:  # or abs(rel.Angle) > rot_tol:
            return None
        return self._collapse(q_sol_full)

    def jacobian(self, q_rad: List[float]) -> Optional[np.ndarray]:
        # ikpy does not expose a stable Jacobian API
        # across versions; return None
        # (the Protocol allows None and the workbench's
        # drag handler doesn't use it).
        raise NotImplementedError
        # return None

    # ---- helpers ----

    def _expand(self, q_active: List[float]) -> np.ndarray:
        """
        Map [q0..q_{n_dof-1}] -> full ikpy q vector with 0s for
        inactive links.
        """
        q_full = np.zeros(len(self._active_mask), dtype=float)
        k = 0
        for i, m in enumerate(self._active_mask):
            if m:
                q_full[i] = float(q_active[k])
                k += 1
        return q_full

    def _collapse(self, q_full: np.ndarray) -> List[float]:
        """Inverse of _expand."""
        return [float(q_full[i]) for i, m in enumerate(self._active_mask) if m]


# --------------------------------------------------------------------------
# Helpers (private to this module)
# --------------------------------------------------------------------------

def _pl_translation_m(pl: Placement) -> List[float]:
    return [pl.Base.x / MM_PER_M, pl.Base.y / MM_PER_M, pl.Base.z / MM_PER_M]


def _pl_rpy(pl: Placement) -> List[float]:
    # FreeCAD getYawPitchRoll → "degrees" in (yaw, pitch, roll) [ZYX intrinsic]
    # URDF / ikpy RPY in "radians" as (roll, pitch, yaw).
    yaw, pitch, roll = pl.Rotation.getYawPitchRoll()
    return [float(np.deg2rad(roll)),
            float(np.deg2rad(pitch)),
            float(np.deg2rad(yaw))]
