"""backends/pinocchio.py — Pinocchio kinematics backend.

Builds a pin.Model programmatically from a ChainSpec. No URDF.

IK: Damped least squares Newton iteration on log6 error.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, TypeAlias

import FreeCAD as App

from freecad.Robot_tools.App.rbt_kine_types import ChainSpec
from freecad.Robot_tools.backends.base import (
    placement_to_matrix4, matrix4_to_placement,
)

if TYPE_CHECKING:
    import numpy as np  # type: ignore
    import pinocchio as pin  # noqa: F401  (type-only)

Placement: TypeAlias = App.Placement


class PinocchioBackend:
    name: str = "pinocchio"

    def __init__(self) -> None:
        # Lazy import so the WB still loads if pinocchio is missing.
        import pinocchio as pin
        import numpy as np  # type: ignore
        self.pin = pin
        self.np = np
        self.model: Optional["pin.Model"] = None
        self.data: Optional["pin.Data"] = None
        self.ee_frame_id: Optional[int] = None
        self.chain: Optional[ChainSpec] = None

    # ---- helpers ----
    def _pl_to_se3(self, pl: Placement) -> "pin.SE3":
        M = placement_to_matrix4(pl)
        return self.pin.SE3(M[0:3, 0:3], M[0:3, 3])

    def _se3_to_pl(self, se3: "pin.SE3") -> Placement:
        np = self.np
        M = np.eye(4)
        M[0:3, 0:3] = se3.rotation
        M[0:3, 3]   = se3.translation
        return matrix4_to_placement(M)

    def _signed(self, q_facade: List[float]) -> "np.ndarray":
        # np = self.np
        # assert self.chain is not None
        # return np.array([q_facade[i] * self.chain.joints[i].dir_sign
        #                  for i in range(len(q_facade))], dtype=float)
        return self.np.array(q_facade, dtype=float)

    def _unsigned(self, q_signed: "np.ndarray") -> List[float]:
        # assert self.chain is not None
        # return [float(q_signed[i]) * self.chain.joints[i].dir_sign
        #         for i in range(len(q_signed))]
        return [float(v) for v in q_signed]

    # ---- API ----
    def load(self, chain: ChainSpec) -> None:
        pin = self.pin
        np = self.np
        model: "pin.Model" = pin.Model()
        parent_id: int = 0  # universe joint

        # accumulate fixed-joint transforms for folding their transform
        pending = App.Placement()

        for j in chain.joints:
            if j.type != "revolute":
                pending = pending.multiply(j.parent_to_joint)
                continue

            se3 = self._pl_to_se3(pending.multiply(j.parent_to_joint))

            # empty the fix joints placement buffer
            pending = App.Placement()

            axis = np.array([j.axis.x, j.axis.y, j.axis.z], dtype=float)
            jm = pin.JointModelRevoluteUnaligned(axis)
            jid = model.addJoint(parent_id, jm, se3, j.name)
            # joint indexing in model.{lower,upper}PositionLimit is 0-based for
            # joint_id minus 1 (joint 0 is the universe).
            model.lowerPositionLimit[jid - 1] = j.lim_low
            model.upperPositionLimit[jid - 1] = j.lim_high
            parent_id = jid

        # Flange frame attached to last joint
        flange_se3 = self._pl_to_se3(pending.multiply(chain.flange_local))
        frame = pin.Frame("flange", parent_id, 0, flange_se3,
                          pin.FrameType.OP_FRAME)
        self.ee_frame_id = model.addFrame(frame)

        self.model = model
        self.data = model.createData()
        self.chain = chain

    def fk(self, q_rad: List[float]) -> Placement:
        pin = self.pin
        assert self.chain is not None and self.ee_frame_id is not None
        q = self._signed(q_rad)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        ee_in_base = self._se3_to_pl(self.data.oMf[self.ee_frame_id])
        return self.chain.base_world.multiply(ee_in_base)

    def ik(
        self,
        target: Placement,
        q_seed_rad: List[float],
        pos_tol: float = 1e-4,
        rot_tol: float = 1e-3,
        max_iter: int = 50,
    ) -> Optional[List[float]]:
        """Canonical Pinocchio DLS IK (matches examples/inverse-kinematics.py)."""
        pin = self.pin
        np  = self.np
        assert self.chain is not None and self.ee_frame_id is not None
        q = self._signed(q_seed_rad)

        # Move target into base-local (pin's "world" is the base joint).
        target_in_base = self.chain.base_world.inverse().multiply(target)
        oMdes = self._pl_to_se3(target_in_base)

        damp = 1e-6
        DT   = 1.0  # step size; with Jlog6 correction we can take full steps

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[self.ee_frame_id]

            # Frame error expressed in the FRAME's local coords (canonical form).
            fMd = oMf.actInv(oMdes)
            err = pin.log6(fMd).vector  # 6-vec twist

            if (np.linalg.norm(err[0:3]) < pos_tol and
                    np.linalg.norm(err[3:6]) < rot_tol):
                return self._unsigned(q)

            J = pin.computeFrameJacobian(
                self.model, self.data, q,
                self.ee_frame_id, pin.ReferenceFrame.LOCAL)
            # Jlog6 correction: turns first-order error into a true Newton step.
            J = -pin.Jlog6(fMd.inverse()) @ J

            v = -J.T @ np.linalg.solve(J @ J.T + (damp ** 2) * np.eye(6), err)
            q = pin.integrate(self.model, q, v * DT)

            # Respect limits (clamp on the manifold result)
            k = 0
            for js in self.chain.joints:
                if js.type != "revolute":
                    continue
                q[k] = max(js.lim_low, min(js.lim_high, q[k]))
                k += 1

        return None  # didn't converge

    def jacobian(self, q_rad: List[float]) -> "np.ndarray":
        pin = self.pin
        assert self.chain is not None and self.ee_frame_id is not None
        q = self._signed(q_rad)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.computeFrameJacobian(
            self.model, self.data, q,
            self.ee_frame_id, pin.ReferenceFrame.LOCAL)
