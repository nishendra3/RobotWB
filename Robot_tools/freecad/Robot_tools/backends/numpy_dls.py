"""numpy_dls.py — zero-dependency DLS IK backend (BENCHMARK ONLY).

Damped-least-squares with analytic geometric Jacobian. Position-only,
mirroring the current ikpy backend behaviour (orientation_mode=None,
rot_tol accepted but unused). Known throwaway limitations: hard clamp
at joint limits can stall near limits (solver then returns None =
skipped frame, same as an ikpy reject); unreachable targets return None.
Delete this file + its two registration lines after benchmarking.
"""

from __future__ import annotations

from typing import List, Optional, TypeAlias

import numpy as np  # type: ignore
import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_kine_types import ChainSpec
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
            if j.type == "revolute":
                ax = np.array([j.axis.x, j.axis.y, j.axis.z], dtype=float)
                n = np.linalg.norm(ax)
                if n < 1e-10:
                    raise RuntimeError(f"Joint '{j.name}' has zero axis")
                ax /= n
                self._steps.append(
                    (A, ax, (float(j.lim_low), float(j.lim_high))))
            else:
                self._steps.append((A, None, None))
        self._F = placement_to_matrix4(chain.flange_local)
        self._n_dof = sum(1 for _, ax, _ in self._steps if ax is not None)

    def fk(self, q_rad: List[float]) -> Placement:
        assert self._chain is not None
        T, _, _ = self._fk_jac_frames(np.asarray(q_rad, dtype=float))
        return self._chain.base_world.multiply(matrix4_to_placement(T))

    def ik(
        self,
        target: Placement,
        q_seed_rad: List[float],
        pos_tol: float = 1e-4,   # meters
        rot_tol: float = 1e-3,   # radians (unused: position-only parity)
        max_iter: int = 50,
    ) -> Optional[List[float]]:
        assert self._chain is not None
        if len(q_seed_rad) != self._n_dof:
            raise ValueError(
                f"seed has {len(q_seed_rad)} values, chain has "
                f"{self._n_dof} DOF")

        # target into base-local meters (same convention as ikpy backend)
        t_loc = self._chain.base_world.inverse().multiply(target)
        p_t = placement_to_matrix4(t_loc)[0:3, 3]

        q = np.asarray(q_seed_rad, dtype=float).copy()
        I3 = np.eye(3)

        for _ in range(max_iter):
            T, zs, ps = self._fk_jac_frames(q)
            p_e = T[0:3, 3]
            e = p_t - p_e
            e_norm = np.linalg.norm(e)
            if e_norm < pos_tol:
                return [float(v) for v in q]

            # geometric Jacobian, position rows: J[:, i] = z_i × (p_e − p_i)
            J = np.stack(
                [np.cross(z, p_e - p) for z, p in zip(zs, ps)], axis=1)

            # adaptive damping: full λ while far (fast-drag safety near
            # singularities), shrinks with the error so the tight release
            # solve (1e-5 m) still converges
            lam = min(DAMPING_M, e_norm)
            lam2 = max(lam * lam, 1e-10)

            # DLS step: dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * I3, e)
            if not np.all(np.isfinite(dq)):
                return None

            m = np.max(np.abs(dq))
            if m > STEP_CLAMP_RAD:
                dq *= STEP_CLAMP_RAD / m
            q += dq
            self._clamp_limits(q)

        # best-effort check after the final update
        T, _, _ = self._fk_jac_frames(q)
        if np.linalg.norm(p_t - T[0:3, 3]) < pos_tol:
            return [float(v) for v in q]
        return None

    def jacobian(self, q_rad: List[float]) -> Optional[np.ndarray]:
        T, zs, ps = self._fk_jac_frames(np.asarray(q_rad, dtype=float))
        p_e = T[0:3, 3]
        Jp = np.stack([np.cross(z, p_e - p) for z, p in zip(zs, ps)], axis=1)
        Jo = np.stack(zs, axis=1)
        return np.vstack([Jp, Jo])

    # ---- internals ----

    def _fk_jac_frames(self, q: np.ndarray):
        """One chain pass: flange 4x4 [m, base-local] plus per-DOF axis
        directions z_i and origins p_i for the geometric Jacobian.
        z_i/p_i are sampled after A_i and before Rot(ax_i, q_i) — a
        rotation moves neither its own axis nor origin."""
        T = np.eye(4)
        zs, ps = [], []
        k = 0
        for A, ax, _ in self._steps:
            T = T @ A
            if ax is not None:
                zs.append(T[0:3, 0:3] @ ax)
                ps.append(T[0:3, 3].copy())
                T = T @ _rot4(ax, float(q[k]))
                k += 1
        T = T @ self._F
        return T, zs, ps

    def _clamp_limits(self, q: np.ndarray) -> None:
        k = 0
        for _, ax, lim in self._steps:
            if ax is None:
                continue
            lo, hi = lim
            if lo < hi:  # skip degenerate/unset limits
                q[k] = min(max(q[k], lo), hi)
            k += 1
