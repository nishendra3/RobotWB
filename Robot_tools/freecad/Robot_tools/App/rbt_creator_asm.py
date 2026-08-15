"""
Assembly document handling for creation of new robot
Builds the Robot_Assembly & links part instances to it
"""

import FreeCAD as App  # type: ignore

from freecad.Robot_tools.App.rbt_global_constants import (
    ROBOT_ASSEMBLY_LABEL)
from freecad.Robot_tools.App.rbt_robot import is_robot, all_robots


def create_assembly(doc):
    asm = doc.addObject("Assembly::AssemblyObject", "Assembly")
    asm.Label = ROBOT_ASSEMBLY_LABEL
    asm.Type = "Assembly"
    asm.newObject("Assembly::JointGroup", "Joints")
    asm.recompute()
    return asm


def add_asm_object(obj_doc, asm, feat_nm, link_nm, glbl):
    """
    Adds objects/links to assembly document
    - Inputs:
      - obj_doc: Document containinng the part to be added
      - asm: Assembly instance where the part has to be added
      - feat_nm: Name of the object in obj_doc
      - link_nm: Name of the new link being created
    """
    item = asm.newObject("App::Link", link_nm)
    item.LinkedObject = obj_doc.getObject(feat_nm)
    item.Label = glbl
    item.recompute()
    asm.recompute()
    return item


def resolve_asm_ref(asm_doc: App.Document,
                    hint: App.DocumentObject | None = None):
    """
    Resolve (asm, fpo, how)
    hint: robot/assembly chosen by the Gui caller.
    how: "from_hint" | "only_robot" | "label_scan" | "unresolved"
    """

    robots = all_robots(asm_doc)

    if (hint is not None and
            hint.Document is asm_doc):
        if is_robot(hint):
            return hint.RobotAssembly, hint, "from_hint"

        fpo = next((r for r in robots if r.RobotAssembly is hint), None)
        return hint, fpo, "from_hint"

    if len(robots) == 1:
        return robots[0].RobotAssembly, robots[0], "only_robot"

    objs = find_assemblies(asm_doc)  # FPO-less in-progress asm
    if not robots and len(objs) == 1:
        return objs[0], None, "label_scan"

    return None, None, "unresolved"


def find_assemblies(doc):
    """
    All Robot Assembly objects in doc
    """
    return [o for o in doc.Objects
            if o.isDerivedFrom("Assembly::AssemblyObject")
            and o.Label.startswith(ROBOT_ASSEMBLY_LABEL)]
