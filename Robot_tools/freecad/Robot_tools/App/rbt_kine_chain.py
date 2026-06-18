"""
rbt_kine_chain.py
extract chain spec from robot fpo
reads:
    robot_obj.Robot_joints
    robot_obj.Robot_joints_dir
    robot_obj.Active_tool
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, TypeAlias

import FreeCAD as App
import UtilsAssembly

from freecad.Robot_tools.App.rbt_kine_types import (
    ChainSpec, JointSpec, LinkSpec
)
from freecad.Robot_tools.rbt_helpers_math import deg_to_rad


fcl_err = App.Console.PrintError

V3: TypeAlias = App.Vector
Placement: TypeAlias = App.Placement
if TYPE_CHECKING:
    RobotFPO: TypeAlias = App.DocumentObject
    JointFPO: TypeAlias = App.DocumentObject


def extract_chain(robot_obj: "App.DocumentObject") -> Optional[ChainSpec]:
    joints_fpo: List["App.DocumentObject"] = list(robot_obj.Robot_joints or [])
    if not joints_fpo:
        fcl_err("Robot has no joints")
        return

    # base link
    j0 = joints_fpo[0]
    ground_obj: "App.DocumentObject" = j0.Reference1[0]
    base_world: Placement = UtilsAssembly.getGlobalPlacement(j0.Reference1)
    links: List[LinkSpec] = [LinkSpec(name=ground_obj.Name,
                                      parent_name=None)]
    joints: List[JointSpec] = []

    prev_joint_world: Placement = base_world
    # dirs: List[int] = [int(x) for x in (robot_obj.Robot_joints_dir or [])]
    # while len(dirs) < len(joints_fpo):
    #     dirs.append(1)

    for idx, j in enumerate(joints_fpo):

        try:
            parent_obj: "App.DocumentObject" = j.Reference1[0]
            child_obj: "App.DocumentObject" = j.Reference2[0]
        except Exception as e:
            fcl_err(f"Unable to read parent & child obj: {e}")

        #   TODO: check -> joint frame in world at q=0
        #   Correct form across nested/linked sub-assemblies. Matches the
        #   `UtilsAssembly.getJcsGlobalPlc` helper (= getGlobalPlacement * jcs).
        #   Do NOT use `parent_obj.Placement * j.Placement1` directly — that
        #   only works for top-level App::Part children.

        joint_world: Placement = (UtilsAssembly
                                  .getGlobalPlacement(j.Reference1)
                                  .multiply(j.Placement1))

        # relative placement from prev to this joint frame
        parent_to_joint: Placement = (prev_joint_world
                                      .inverse()
                                      .multiply(joint_world))

        # Assembly WB convention
        # Revolut axis is local Z of placement 1
        # axis: V3 = V3(0, 0, 1) <- Hardcoded value

        # Ass. WB aligns child JCS (Placement2, incl. Offset2) onto
        # the parent JCS (Placement1) either same dir or z-flipped
        # flipped (z antiparallel): child +q
        # same-dir (z parallel): child -q
        jcs2_world: Placement = (UtilsAssembly.
                                 getGlobalPlacement(j.Reference2).
                                 multiply(j.Placement2))
        rel = joint_world.inverse().multiply(jcs2_world)
        z_dot = rel.Rotation.multVec(V3(0, 0, 1)).z

        # Axis detection sanity check
        if abs(z_dot) < 0.9:
            fcl_err(f"joint '{j.Label}': JCS z-axes not (anti)parallel "
                    f"(z_dot={z_dot:.3f}) — extracted chain may be wrong\n")

        axis: V3 = V3(0, 0, 1) if z_dot < 0.0 else V3(0, 0, -1)

        low: float = float(j.AngleMin if
                           getattr(j, "EnableAngleMin", False) else -180.00)
        high: float = float(j.AngleMax if
                            getattr(j, "EnableAngleMax", False) else 180.00)

        joints.append(JointSpec(
            name=j.Label or f"joint{idx:02d}",
            type="revolute" if j.JointType == "Revolute" else "fixed",
            parent_to_joint=parent_to_joint,
            axis=axis,
            lim_low=deg_to_rad(low),
            lim_high=deg_to_rad(high),
            # dir_sign=int(dirs[idx]),
        ))
        links.append(LinkSpec(
            name=child_obj.Name,
            parent_name=parent_obj.Name,
            joint_to_part=joint_world.inverse().multiply(child_obj.Placement)
        ))
        prev_joint_world = joint_world

    # flange - active tool's flange link taken as last joint frame
    flange_local: Placement = Placement()
    tool: "Optional[App.DocumentObject]" = getattr(robot_obj,
                                                   "Active_tool", None)
    if tool and getattr(tool, "Flange_link", None) and tool.Flange_link[0]:
        # flange_world = (UtilsAssembly
        #                 .getGlobalPlacement(tool.Flange_link)
        #                 .multiply(UtilsAssembly
        #                           .findPlacement(tool.Flange_link)))

        # use the active tool's tcp as the chain tip
        tcp_world = Placement(tool.TCP_placement)
        flange_local = prev_joint_world.inverse().multiply(tcp_world)

    return ChainSpec(
        base_world=base_world,
        flange_local=flange_local,
        links=links,
        joints=joints,
    )
