"""
numpy_dls.py — DLS IK backend.
Damped-least-squares with analytic geometric Jacobian
"""
from __future__ import annotations

from typing import List, Optional, TypeAlias

import numpy as np  # type: ignore
import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import (
    ChainSpec, FIXED, PRISMATIC)
from freecad.Robot_tools.backends.base import (
    placement_to_matrix4, matrix4_to_placement,
)

Placement: TypeAlias = App.Placement

DAMPING_M = 0.01      # max λ, task-space meters (standard DLS units)
STEP_CLAMP_RAD = 0.2  # max per-iteration joint step


def _rot4(axis: np.ndarray, q: float) -> np.ndarray:
    """4x4 rotation about a unit axis by q rad (Rodrigues)."""
    x, y, z = axis
    c, s = np.cos(q), np.sin(q)
    C = 1.0 - c
    return np.array([
        [c + x*x*C,   x*y*C - z*s, x*z*C + y*s, 0.0],
        [y*x*C + z*s, c + y*y*C,   y*z*C - x*s, 0.0],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C,   0.0],
        [0.0,         0.0,         0.0,         1.0],
    ])


def _trans4(axis: np.ndarray, q: float) -> np.ndarray:
    """4x4 translation of q [m] along a unit axis."""
    M = np.eye(4)
    M[0:3, 3] = axis * q
    return M


def _mat3_to_rotvec(R):
    """axis*angle (rad) of a 3x3 rotation"""
    tr = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    ang = float(np.arccos(tr))
    if ang < 1e-9:
        return np.zeros(3)
    if ang > np.pi - 1e-6:
        A = (R + np.eye(3)) * 0.5
        k = int(np.argmax(np.diag(A)))
        axis = A[:, k] / np.sqrt(max(A[k, k], 1e-12))
        return axis / np.linalg.norm(axis) * ang
    w = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    return w * (0.5 * ang / np.sin(ang))


class NumpyDLSBackend:
    name: str = "numpy_dls"

    def __init__(self) -> None:
        self._chain: Optional[ChainSpec] = None
        # per chain step: (A 4x4 [m], unit axis or None, (lo, hi) rad or None)
        self._steps = []
        self._F = None  # flange_local [m]
        self._n_dof: int = 0

    # ---- KinematicsBackend Protocol ----

    def load(self, chain: ChainSpec) -> None:
        if not chain.joints:
            raise RuntimeError("ChainSpec has no joints")
        self._chain = chain
        self._steps = []
        for j in chain.joints:
            A = placement_to_matrix4(j.parent_to_joint)
            if j.type == FIXED:
                self._steps.append((A, None, None, False))
                continue

            ax = np.array([j.axis.x, j.axis.y, j.axis.z], dtype=float)
            n = np.linalg.norm(ax)
            if n < 1e-10:
                raise RuntimeError(f"Joint '{j.name}' has zero axis")
            ax /= n
            self._steps.append(
                (A, ax, (float(j.lim_low),
                         float(j.lim_high)), j.type == PRISMATIC))

        self._F = placement_to_matrix4(chain.flange_local)
        self._n_dof = sum(1 for _, ax, _, _ in self._steps if ax is not None)

    def fk(self, q_rad: List[float]) -> Placement:
        assert self._chain is not None
        T, _, _, _ = self._fk_jac_frames(np.asarray(q_rad, dtype=float))
        return self._chain.base_in_asm.multiply(matrix4_to_placement(T))

    def ik(
        self,
        target: Placement,
        q_seed_rad: List[float],
        pos_tol: float = 1e-4,  # meters
        rot_tol: float = 1e-3,  # radians
        max_iter: int = 50,
    ) -> Optional[List[float]]:
        assert self._chain is not None
        if len(q_seed_rad) != self._n_dof:
            raise ValueError(
                f"seed has {len(q_seed_rad)} values, chain has "
                f"{self._n_dof} DOF")

        # target into base-local meters (same convention as ikpy backend)
        t_loc = self._chain.base_in_asm.inverse().multiply(target)
        T_t = placement_to_matrix4(t_loc)
        p_t = T_t[0:3, 3]
        R_t = T_t[0:3, 0:3]

        def task_err(T):
            """(stacked 6-error, converged) vs target, base-local"""
            e_pos = p_t - T[0:3, 3]
            e_rot = _mat3_to_rotvec(R_t @ T[0:3, 0:3].T)   # base-frame, rad
            done = (np.linalg.norm(e_pos) < pos_tol
                    and np.linalg.norm(e_rot) < rot_tol)
            return np.concatenate([e_pos, e_rot]), done

        q = np.asarray(q_seed_rad, dtype=float).copy()
        I_task = np.eye(6)

        for _ in range(max_iter):
            T, zs, ps, prisms = self._fk_jac_frames(q)
            p_e = T[0:3, 3]
            e, done = task_err(T)
            if done:
                return [float(v) for v in q]

            # geometric Jacobian, position rows:
            # revolute: z_i x (p_e - p_i) | prismatic: z_i
            J = np.stack([z if pr else np.cross(z, p_e - p)
                          for z, p, pr in zip(zs, ps, prisms)], axis=1)

            # angular rows: revolute: z_i | prismatic: 0
            Jo = np.stack([np.zeros(3) if pr else z
                           for z, pr in zip(zs, prisms)], axis=1)
            J = np.vstack([J, Jo])

            # adaptive damping
            lam = min(DAMPING_M, np.linalg.norm(e))
            lam2 = max(lam * lam, 1e-10)

            # DLS step: dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * I_task, e)
            if not np.all(np.isfinite(dq)):
                return None

            m = np.max(np.abs(dq))
            if m > STEP_CLAMP_RAD:
                dq *= STEP_CLAMP_RAD / m
            q += dq
            self._clamp_limits(q)

        # best-effort check after the final update
        T, _, _, _ = self._fk_jac_frames(q)
        _, done = task_err(T)
        return [float(v) for v in q] if done else None

    def jacobian(self, q_rad: List[float]) -> Optional[np.ndarray]:
        T, zs, ps, prisms = self._fk_jac_frames(np.asarray(q_rad, dtype=float))
        p_e = T[0:3, 3]
        Jp = np.stack([z if pr else np.cross(z, p_e - p)
                       for z, p, pr in zip(zs, ps, prisms)], axis=1)
        Jo = np.stack([np.zeros(3) if pr else z
                       for z, pr in zip(zs, prisms)], axis=1)
        return np.vstack([Jp, Jo])

    # ---- internals ----

    def _fk_jac_frames(self, q: np.ndarray):
        """One chain pass"""
        T = np.eye(4)
        zs, ps, prisms = [], [], []
        k = 0
        for A, ax, _, prism in self._steps:
            T = T @ A
            if ax is not None:
                zs.append(T[0:3, 0:3] @ ax)
                ps.append(T[0:3, 3].copy())
                prisms.append(prism)
                T = T @ (_trans4(ax, float(q[k])) if prism
                         else _rot4(ax, float(q[k])))
                k += 1
        T = T @ self._F
        return T, zs, ps, prisms

    def _clamp_limits(self, q: np.ndarray) -> None:
        k = 0
        for _, ax, lim, _ in self._steps:
            if ax is None:
                continue
            lo, hi = lim
            if lo < hi:  # skip degenerate/unset limits
                q[k] = min(max(q[k], lo), hi)
            k += 1
