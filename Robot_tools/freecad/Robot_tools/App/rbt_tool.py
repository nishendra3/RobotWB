"""rbt_tool.py — Tool / TCP document object."""

import FreeCAD as App  # type: ignore
import UtilsAssembly   # type: ignore

from freecad.Robot_tools.App.rbt_helpers_frames import (
    JogFrame, apply_jog_frames)
from freecad.Robot_tools.App.rbt_global_constants import WB_NAME
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err

_TYPE_ = f"{WB_NAME}::Tool"

TOOL_SCHEMA = [
    ("Tool_shape", "App::PropertyLinkGlobal", "Tool",
     "(optional) CAD for the tool"),

    ("Tool_offset", "App::PropertyPlacement", "Tool",
     "Flange -> tool body"),

    ("TCP_offset", "App::PropertyPlacement", "Tool", "Tool body -> TCP"),

    ("TCP_placement", "App::PropertyPlacement", "Tool",
     "(read only) world TCP"),

    ("Tool_mass", "App::PropertyFloat", "Tool", "kg"),

    ("Flange_link", "App::PropertyLinkSubGlobal", "Tool",
        "Face on the Robot where tool's flange mates"),

    ("Tool_flange_link", "App::PropertyLinkSubGlobal", "Tool",
        "Face on the tool that mates to robot's flange"),

    ("TCP_link",   "App::PropertyLinkSubGlobal", "Tool",
        "Reference on tool geom that defines TCP location"),

    ("TCP_drag_frame", "App::PropertyEnumeration", "Tool",
        "Current Axis-drag frame for inverse kinematic simulation"),

    ("Jog_frames", "App::PropertyStringList", "Tool",
     "Frames available for TCP dragging"),

    ("Source_file", "App::PropertyFile", "Tool", "Tool CAD's .fcstd path")
]


class Tool:
    def __init__(self, obj):
        self.Type = _TYPE_
        self.add_properties(obj)
        obj.Proxy = self

        obj.Jog_frames = JogFrame.names()

    def onDocumentRestored(self, fp):
        self.migrate_flange_link(fp)
        self.add_properties(fp)
        self.migrate_drag_frame(fp)

    def onChanged(self, fp, prop):
        if "Restore" in fp.State:
            return

        if prop == "Jog_frames":
            apply_jog_frames(fp)
            return

        # invalidate cached backend on tool change
        if prop in ("Tool_offset", "TCP_offset", "TCP_link",
                    "Tool_flange_link", "Flange_link", "Tool_shape"):
            doc = getattr(fp, "Document", None)
            if doc is None:
                return

            # find the rob that owns this tool
            rob = next((r for r in doc.Objects
                        if hasattr(r, "Tools") and fp in
                        r.Tools), None)
            if rob is not None:
                from freecad.Robot_tools.App import rbt_kine
                rbt_kine.invalidate(rob)

    def execute(self, fp):
        """
            Recomputes tcp placement & tool body shape
        """
        from freecad.Robot_tools.App.rbt_helpers_math import flip_z_dir

        rob_flange_ref = fp.Flange_link
        if not rob_flange_ref or not rob_flange_ref[0]:
            return

        rob_flange = (UtilsAssembly.getGlobalPlacement(rob_flange_ref) *
                      UtilsAssembly.findPlacement(rob_flange_ref))

        tool_flange_ref = fp.Tool_flange_link

        if fp.Tool_shape:
            # add tool flange to robot flange
            # if user didnt pick tool flange yet
            # then assume tool's local origin as flange
            if tool_flange_ref and tool_flange_ref[0]:
                tool_flange_local = UtilsAssembly.findPlacement(
                                    tool_flange_ref)
            else:
                tool_flange_local = App.Placement()

            # add tool flange face antiparallel to rob flange
            tool_shape_placement = (flip_z_dir(rob_flange)
                                    .multiply(fp.Tool_offset)
                                    .multiply(tool_flange_local.inverse()))
            if not fp.Tool_shape.Placement.isSame(tool_shape_placement):
                fp.Tool_shape.Placement = tool_shape_placement
                fp.Tool_shape.purgeTouched()

            # fp.Tool_shape.Placement = tool_shape_placement
            # seated_flange = tool_shape_placement.multiply(tool_flange_local)

            tcp_ref = fp.TCP_link
            tcp_local = UtilsAssembly.findPlacement(
                tcp_ref) if (tcp_ref and tcp_ref[0]) else App.Placement()

            # TCP placement relative to tool body
            tcp = (tool_shape_placement
                   .multiply(tcp_local)
                   .multiply(fp.TCP_offset))
        else:
            # TCP placement relative to robot's flange
            tcp = (rob_flange
                   .multiply(fp.Tool_offset)
                   .multiply(fp.TCP_offset))

        if not fp.TCP_placement.isSame(tcp):
            fp.TCP_placement = tcp

    def add_properties(self, fp):
        """
        Add properties to older versions of tool
        """
        self.Type = _TYPE_
        existing = set(fp.PropertiesList)
        for name, typ, group, doc in TOOL_SCHEMA:
            if name not in existing:
                fp.addProperty(typ, name, group, doc)

        # set Jog_frames to be read only in the UI for now
        fp.setEditorMode("Jog_frames", 1)

    def migrate_flange_link(self, fp):
        """
        Cleanup certain old properties & replace with newer
        """
        if "Flange_link" not in fp.PropertiesList:
            return
        if fp.getTypeIdOfProperty(
                "Flange_link") == "App::PropertyLinkSubGlobal":
            return
        old = fp.Flange_link
        fp.removeProperty("Flange_link")
        fp.addProperty("App::PropertyLinkSubGlobal", "Flange_link", "Tool",
                       "Face on the Robot where tool's flange mates")
        if old is not None:
            #  user must re-pick
            fp.Flange_link = (old, [""])

    def migrate_drag_frame(self, fp):
        """
        Upgrade docs saved with the old ["Local", "World"] enum
        """
        names = JogFrame.names()
        if fp.getEnumerationsOfProperty("TCP_drag_frame") == names:
            return
        fp.Jog_frames = names  # -> onChanged -> apply_jog_frames

# --------------------------
#         helpers
# --------------------------


# helpers
def create_tool(robot, name="Default_Tool"):
    """
    Creates a tool with no geom and identity offsets
    """
    doc = robot.Document
    tool_fpo = doc.addObject("App::FeaturePython", name)
    Tool(tool_fpo)

    # attach a viewprovider in GUI mode
    if App.GuiUp:
        from freecad.Robot_tools.Gui.vp_rbt_tool import ViewProviderTool
        ViewProviderTool(tool_fpo.ViewObject)

    tool_fpo.Flange_link = (robot.RobotJoints[-1].Reference2
                            if robot.RobotJoints else None)
    tool_fpo.Tool_offset = App.Placement()
    tool_fpo.TCP_offset = App.Placement()
    tool_fpo.Tool_mass = 0.0
    robot.Tools = list(robot.Tools) + [tool_fpo]
    robot.ActiveTool = tool_fpo
    return tool_fpo


def import_shape(rob, path):
    """
    import tool geom from .fcstd file
    - [input] rob : robot fpo (used for Document)
    - [input] path : .fcstd file path
    """
    doc = rob.Document
    before = {o.Name for o in doc.Objects}
    doc.mergeProject(path)
    new_objs = [o for o in doc.Objects if o.Name not in before]
    shapes = [o for o in new_objs if has_valid_shape(o)]
    if not shapes:
        fcl_err("no suitable shape in selected file\n")
        return None
    return shapes[0]


def tool_parent(tool_fpo):
    """Return the robot FPO that owns this tool, or None."""
    if tool_fpo is None:
        return None
    return next((r for r in tool_fpo.Document.Objects
                 if hasattr(r, "Tools") and tool_fpo in r.Tools),
                None)


def has_valid_shape(obj):
    """
    True when obj has non null shape
    with selectable faces
    """
    return (hasattr(obj, "Shape")
            and obj.Shape
            and bool(obj.Shape.Faces))
