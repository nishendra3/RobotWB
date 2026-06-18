"""Robot math helper functions.

Name: rbt_helpers_math.py

See Changelog below.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence:  LGPL 2.1
"""
from __future__ import annotations

__version__ = "0.01"
__build__ = "20260507_1255"

import math

from typing import Union, TypeAlias

import FreeCAD as App

V3: TypeAlias = App.Vector
Number: TypeAlias = Union[int, float]
MM_PER_M: float = 1000.0
RAD_PER_DEG: float = math.pi / 180.0
DEG_PER_RAD: float = 180.0 / math.pi

# ------------------------------------------------
#               Service functions
# ------------------------------------------------


def roundvec(r_vec, prec=6):
    """Round value in vectors."""
    vlist = [round(r_vec.x, prec), round(r_vec.y, prec), round(r_vec.z, prec)]
    return vlist


def roundrot(r_rot, prec=6):
    """Round value in vectors."""
    vlist = [round(r_rot[0], prec), round(r_rot[1], prec), round(r_rot[2], prec)]
    return vlist


def rpy_to_rot(rx_deg, ry_deg, rz_deg):
    """ZYX - euler degrees to App.Rotation"""
    return App.Rotation(rz_deg, ry_deg, rx_deg)


def rot_to_rpy(rot):
    """App.Rotation to euler degrees - ZYX format"""
    yaw, pitch, roll = rot.toEuler()
    return roll, pitch, yaw


def flip_z_dir(plc):
    """
    flips placement's z axis
    - [input] plc : App.placement
    - [out] App.placement rotated 180 deg about local X
    """
    flip = App.Placement(App.Vector(), App.Rotation(App.Vector(1, 0, 0), 180))
    return plc.multiply(flip)


def mm_to_m(v: Union[Number, V3]) -> Union[float, V3]:
    """
        converts mm to m
        accepts either scalar or vector input
    """
    if hasattr(v, "x"):
        return V3(v.x / MM_PER_M,
                  v.y / MM_PER_M,
                  v.z / MM_PER_M)
    return float(v) / MM_PER_M


def m_to_mm(v: Union[Number, V3]) -> Union[float, V3]:
    """
        converts m to mm
        accepts either scalar or vector input
    """
    if hasattr(v, "x"):
        return V3(v.x * MM_PER_M,
                  v.y * MM_PER_M,
                  v.z * MM_PER_M)
    return float(v) * MM_PER_M


def deg_to_rad(d: Number) -> float:
    """
        converts deg to rad
    """
    return float(d) * RAD_PER_DEG


def rad_to_deg(d: Number) -> float:
    """
        converts rad to deg
    """
    return float(d) * DEG_PER_RAD

