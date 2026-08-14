"""
rbt_kine_types.py
kinematic data-structures for robot fk & ik
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Literal, Optional, TypeAlias


import FreeCAD as App  # type: ignore

# --------------------
V3: TypeAlias = App.Vector
Placement: TypeAlias = App.Placement
JointType: TypeAlias = Literal["revolute", "prismatic", "fixed"]
# --------------------

# Supported Joints
REVOLUTE: JointType = "revolute"
PRISMATIC: JointType = "prismatic"
FIXED: JointType = "fixed"

# map name to Assembly JointType index
JOINT_TYPES = {FIXED: 0, REVOLUTE: 1, PRISMATIC: 3}


def joint_type_FC2WB(fc_jnt_name: str) -> JointType:
    """
        Map Freecad assembly wb joint type string
        to the joint string names used in robot WB
    """
    if fc_jnt_name == "Revolute":
        return REVOLUTE
    if fc_jnt_name == "Slider":
        return PRISMATIC
    else:
        return FIXED


# --------------------
#    chain spec
# --------------------


@dataclass(frozen=True)
class JointSpec:
    """
    Base data class for Joints. (non-mutable)\n
    Contains:\n
        - name: Name of the current joint
        - type: Joint Type (Revolute, Prismatic etc)
        - parent_to_joint: Placement wrt parent frame to curr frame
        - axis: Joint axis
        - lim_low: Lower joint limit
        - lim_high: Upper joint limit
    """
    name: str
    type: JointType  # rev, prismatic, fixed
    parent_to_joint: Placement  # SE(3) from prev to curr joint frame
    axis: V3

    lim_low: float
    lim_high: float


@dataclass(frozen=True)
class LinkSpec:
    """
    Base data class for Links. (non-mutable)\n
    Contains:\n
        - name: Name of the current link
        - parent_name: Name of the parent link
        - joint_to_part: constant transform from joint
                         frame to its child part, set
                         when all angles are 0 (q=0)
                         ("None" for base)
    """
    name: str
    parent_name: Optional[str]
    joint_to_part: Optional[Placement] = None


@dataclass
class ChainSpec:
    """
    Base data class for kinematic chains.\n
    Contains:\n
        - base_in_asm: Base link frame in Robot Assembly Coords
        - flange_local: TCP when tool is configured else last joint frame
        - links: List of links
        - joints: List of joints
    """
    base_in_asm: Placement  # asm -> base
    flange_local: Placement  # last jnt frame -> flange
    links: List[LinkSpec]
    joints: List[JointSpec]
