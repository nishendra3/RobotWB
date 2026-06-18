"""
backends/base.py
backend contract & shared adapters for kin library
"""

from __future__ import annotations
from typing import (
    TYPE_CHECKING, List, Optional, Protocol,
    TypeAlias, runtime_checkable
)

import FreeCAD as App  # type: ignore
from freecad.Robot_tools.App.rbt_kine_types import ChainSpec

MM_PER_M: float = 1000.0

if TYPE_CHECKING:
    # import numpy only for type 
    # hints with TYPE_CHECKING
    import numpy as np  # type: ignore

Placement: TypeAlias = App.Placement


@runtime_checkable
class KinematicsBackend(Protocol):
    """
    This class implements a template that you need to
    follow to be able to qualify a given class as a
    "Kinematics Backend" class. The implemented class
    methods must be a superset of this class.
    """

    name: str  # lib identifier, eg "pinocchio"

    def load(self, chain: ChainSpec) -> None:
        pass

    def fk(self, q_rad: List[float]) -> Placement:
        """
            q in rad, returns flange placement in world
            frame in mm/deg
        """
        pass

    def ik(self,
           target: Placement,
           q_seed_rad: List[float],
           pos_tol: float = 1e-4,  # unit: meters
           rot_tol: float = 1e-3,  # unit: radians
           max_iter: int = 50,) -> Optional[List[float]]:
        """
            success: returns q in rad
            failure to converge: returns none
        """
        pass

    def jacobian(self, q_rad: List[float]) -> Optional["np.ndarray"]:
        """
            (optional) returns None or ndarray of shape (6, n_dof)
            in Local frame
        """
        pass


# ------------------------------------------------------------
#    SE(3) adapters: FreeCAD placement <-> 4x4 numpy arr
# ------------------------------------------------------------


def placement_to_matrix4(pl: Placement) -> "np.ndarray":
    """
        FC placement (mm) to 4x4 numpy matrix (m)
        translation: scaled to meters
        rotation: unitless
    """
    import numpy as np  # type: ignore

    m = pl.toMatrix()
    M = np.array([
        [m.A11, m.A12, m.A13, m.A14],
        [m.A21, m.A22, m.A23, m.A24],
        [m.A31, m.A32, m.A33, m.A34],
        [m.A41, m.A42, m.A43, m.A44],
    ], dtype=float)
    M[0:3, 3] /= MM_PER_M
    return M


def matrix4_to_placement(M: "np.ndarray") -> Placement:
    """
        4x4 numpy matrix (m) to FC placement (mm)
        translation: scaled to millimeters
        rotation: unitless
    """
    M2 = M.copy()
    M2[0:3, 3] *= MM_PER_M
    fm = App.Matrix(*M2.flatten().tolist())
    return Placement(fm)
