"""
rbt_kine_joints.py - per joint config
(direction/zero position/home position)
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, replace
from typing import Dict

from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn


@dataclass
class JointCfg:
    """
    per-joint values we dont have in default FC assembly joint
    - dir: rotation direction around the JCS Z axis (+1/-1)
    - zero: raw Offset2 value at q = 0
    - home: raw Offset2 value of the home position for this joint
    """
    dir: int = 1
    zero: float = 0.0
    home: float = 0.0

    @staticmethod
    def from_dict(data: dict) -> "JointCfg":
        return JointCfg(dir=data.get("dir", 1),
                        zero=data.get("zero", 0.0),
                        home=data.get("home", 0.0))


def load_cfg_map(fpo) -> Dict[str, JointCfg]:
    """
    {joint name : joint configuration}
    """
    try:
        data = data = json.loads(
            getattr(fpo, "Robot_joints_cfg", None) or "{}")
    except (ValueError, TypeError) as e:
        fcl_warn(f"{fpo.Name}: bad joint config {e}")
        return {}
    return {nm: JointCfg.from_dict(kv) for nm, kv in data.items()}


def save_cfg_map(fpo, m: Dict[str, JointCfg]) -> None:
    fpo.Robot_joints_cfg = json.dumps(
        {nm: asdict(cfg) for nm, cfg in m.items()})


def get_joint_cfg(fpo, joint) -> JointCfg:
    return load_cfg_map(fpo).get(joint.Name, JointCfg())


def set_joint_cfg(fpo, joint, **changes) -> None:
    """
    set_joint_cfg
    (fpo, j, dir=-1) / (fpo, j, zero=..., home=...)
    """
    m = load_cfg_map(fpo)
    m[joint.Name] = replace(m.get(joint.Name, JointCfg()), **changes)
    save_cfg_map(fpo, m)


def drop_joint_cfg(fpo, joint) -> None:
    m = load_cfg_map(fpo)
    if m.pop(joint.Name, None) is not None:
        save_cfg_map(fpo, m)
