"""
rbt_kine_types.py
kinematic data-structures for robot fk & ik
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Literal, Optional, Union, TypeAlias


import FreeCAD as App  # type: ignore

# --------------------
V3: TypeAlias = App.Vector
Placement: TypeAlias = App.Placement
Number: TypeAlias = Union[int, float]
JointType: TypeAlias = Literal["revolute", "prismatic", "fixed"]
# --------------------

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
    # dir_sign: int  # +1 / -1 Robot_joints_dir


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
        - base_world: Placement of the base frame wrt world
        - flange_local: TCP when tool is configured else last joint frame
        - links: List of links
        - joints: List of joitns
    """
    base_world: Placement  # world -> base
    flange_local: Placement  # last jnt frame -> flange
    links: List[LinkSpec]
    joints: List[JointSpec]

    def n_dof(self) -> int:
        """
        Returns total DoF by counting non-fixed joints.
        """
        return sum(1 for j in self.joints
                   if j.type != "fixed")
