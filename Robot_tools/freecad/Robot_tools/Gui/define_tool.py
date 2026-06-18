"""define_tool.py — Create Tool & TCP object."""
__version__ = "0.01"

import FreeCAD as App
import FreeCADGui as Gui
import UtilsAssembly
from PySide import QtGui

from freecad.Robot_tools.App.rbt_tool import Tool

fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning
fcl_err = App.Console.PrintError


# helpers
def create_default_tool(robot, name="Default_Tool"):
    """
    Creates a tool with no geom and identity offsets
    """
    from freecad.Robot_tools.Gui.vp_rbt_tool import ViewProviderTool

    doc = robot.Document
    tool_fpo = doc.addObject("App::FeaturePython", name)
    Tool(tool_fpo)
    ViewProviderTool(tool_fpo.ViewObject)
    if robot.Robot_joints:
        tool_fpo.Flange_link = (robot.Robot_joints[-1].Reference2
                                if robot.Robot_joints else None)
    tool_fpo.Tool_offset = App.Placement()
    tool_fpo.TCP_offset = App.Placement()
    tool_fpo.Tool_mass = 0.0
    robot.Tools = list(robot.Tools) + [tool_fpo]
    robot.Active_tool = tool_fpo
    return tool_fpo


def import_shape(rob, path):
    """
    import tool geom from .fcstd file
    - [input] rob : robot fpo (used for Document)
    - [input] path : .fcstd file path
    """
    from freecad.Robot_tools.App.rbt_tool import has_valid_shape
    doc = rob.Document
    before = {o.Name for o in doc.Objects}
    doc.mergeProject(path)
    new_objs = [o for o in doc.Objects if o.Name not in before]
    shapes = [o for o in new_objs if has_valid_shape(o)]
    if not shapes:
        fcl_err("no suitable shape in selected file\n")
        return None
    return shapes[0]


def import_tool(rob, path):
    """
    TODO:: Import tools directly in future
    import tool from .fcstd file & create tool fpo
    if tool fpo exists copy directly, if not create a default fpo
    - [input] rob : robot fpo to attach the tool to
    - [input] path : file path containing geom
    """
    import os
    from freecad.Robot_tools.App.rbt_tool import (
        is_tool_fpo,
        has_valid_shape
    )
    doc = rob.Document
    before = {o.Name for o in doc.Objects}
    doc.mergeProject(path)
    new_objs = [o for o in doc.Objects if o.Name not in before]

    # check if tool fpo already exists
    tool_fpos = [o for o in new_objs if is_tool_fpo(o)]
    t = tool_fpos[0] if tool_fpos else None
    if t is None:
        # create default tool fpo for first usable shape
        shapes = [o for o in new_objs
                  if has_valid_shape(o)]
        if not shapes:
            fcl_err("no Tool FPO or suitable shape in .fcstd")
            return

        # take first usable shape
        file_base = os.path.splitext(os.path.basename(path))[0]
        t = create_default_tool(rob, name=file_base)
        t.Tool_shape = shapes[0]

    # robot side connections
    t.Source_file = path
    if rob.Robot_joints:
        last_joint = rob.Robot_joints[-1]
        t.Flange_link = last_joint.Reference2

    tools = list(rob.Tools)
    if t not in tools:
        tools.append(t)
        rob.Tools = tools
    rob.Active_tool = t

    # restore active doc
    Gui.ActiveDocument = Gui.getDocument(doc.Name)
    return t


def tool_parent(tool_fpo):
    """Return the robot FPO that owns this tool, or None."""
    if tool_fpo is None:
        return None
    return next((r for r in tool_fpo.Document.Objects
                 if hasattr(r, "Tools") and tool_fpo in r.Tools),
                None)


def set_qsb(sb, value):
    """
    write a number into Gui::QuantitySpinBox (unit: mm, deg, kg)
    """
    sb.setProperty("value", App.Units.Quantity(float(value),
                                               sb.property("unit")))


def get_qsb(sb):
    """
    reads a number from Gui::QuantitySpinBox in specified unit
    """
    return float(App.Units.Quantity(
        sb.property("value")).getValueAs(sb.property("unit")))


class DefineTCP:
    """
    Task panel to edit the data in the Tool FPO
    cancel = abortTransaction
    accept = commitTransaction
    """
    def __init__(self, robot, tool=None):
        self.robot = robot
        self.doc = robot.Document
        self.inEdit = tool is not None
        self.doc.openTransaction("Edit Tool"
                                 if self.inEdit else "Create Tool")
        try:
            self.tool = tool if self.inEdit else create_default_tool(robot)
            self.form = self.cm_form()
            self.load()
            self.connect()
        except Exception:
            self.doc.abortTransaction()
            raise

    def default_flange(self):
        if not self.robot.Robot_joints:
            return None
        return self.robot.Robot_joints[-1].Reference2

    def cm_form(self):
        from freecad.Robot_tools.rbt_helpers_ui import load_panel_ui
        w = load_panel_ui("define_tool.ui")
        w.setWindowTitle(f"{'Edit' if self.inEdit else 'Define'} Tool & TCP")

        self.tool_name = w.tool_name
        self.tool_name.setText(self.tool.Label)
        self.tool_name.setEnabled(not self.inEdit)

        self.tool_flange_lbl = w.tool_flange_lbl
        self.tool_shape_lbl = w.tool_shape_lbl
        self.tool_mass = w.tool_mass

        self.tflange_lbl = w.tflange_lbl  # tool flange
        self.tcp_lbl = w.tcp_lbl  # tcp on the tool

        self.tool_off = [w.tool_off_x, w.tool_off_y, w.tool_off_z,
                         w.tool_off_w, w.tool_off_p, w.tool_off_r]
        self.tcp_off = [w.tcp_off_x, w.tcp_off_y, w.tcp_off_z,
                        w.tcp_off_w, w.tcp_off_p, w.tcp_off_r]
        return w

    def set_placement(self, placement, pose_row):
        """
        set placement in ui boxes
        """
        yaw, pitch, roll = placement.Rotation.toEuler()
        vals = (placement.Base.x, placement.Base.y, placement.Base.z,
                roll, pitch, yaw)
        for sb, v in zip(pose_row, vals):
            sb.blockSignals(True)
            set_qsb(sb, v)
            sb.blockSignals(False)

    def get_placement(self, pose_row):
        """
        get placement from ui boxes
        """
        x, y, z, w, p, r = (get_qsb(sb) for sb in pose_row)
        return App.Placement(App.Vector(x, y, z), App.Rotation(r, p, w))

    def load(self):
        self.refresh_flange_lbl()
        self.refresh_tflange_lbl()
        self.refresh_shape_lbl()
        self.refresh_tcp_lbl()
        self.set_placement(self.tool.Tool_offset, self.tool_off)
        self.set_placement(self.tool.TCP_offset, self.tcp_off)

        self.tool_mass.blockSignals(True)
        set_qsb(self.tool_mass, self.tool.Tool_mass)
        self.tool_mass.blockSignals(False)

    # --------------------
    #   label refreshers
    # --------------------

    def refresh_flange_lbl(self):
        fl = self.tool.Flange_link
        if fl and fl[0]:
            sub = fl[1][0] if fl[1] else ""
            self.tool_flange_lbl.setText(f"{fl[0].Label}.{sub}" if sub else fl[0].Label)
        else:
            self.tool_flange_lbl.setText("<none>")

    def refresh_tflange_lbl(self):
        fl = self.tool.Tool_flange_link
        if fl and fl[0]:
            sub = fl[1][0] if fl[1] else ""
            self.tflange_lbl.setText(f"{fl[0].Label}.{sub}"
                                     if sub else fl[0].Label)
        else:
            self.tflange_lbl.setText("<none>")

    def refresh_tcp_lbl(self):
        tl = self.tool.TCP_link
        if tl and tl[0]:
            sub = tl[1][0] if tl[1] else ""
            self.tcp_lbl.setText(f"{tl[0].Label}.{sub}"
                                 if sub else tl[0].Label)
        else:
            self.tcp_lbl.setText("<none>")

    def refresh_shape_lbl(self):
        s = self.tool.Tool_shape
        self.tool_shape_lbl.setText(s.Label if s else "<none>")

    def connect(self):
        """
        connect button actions to relevant functions
        """
        # flange area picker
        self.form.btn_flange.clicked.connect(self._on_pick_flange)

        # tool geometry
        self.form.btn_shape.clicked.connect(self._on_pick_shape)

        # tool offset entries
        for b in self.tool_off:
            b.valueChanged.connect(self._on_tool_off)

        # tcp offset entries
        for b in self.tcp_off:
            b.valueChanged.connect(self._on_tcp_off)

        # tool mass entry
        self.tool_mass.valueChanged.connect(self._on_mass)

        # tool flange
        self.form.btn_tflange.clicked.connect(self._on_pick_tflange)

        # tcp selection btn
        self.form.btn_tcp.clicked.connect(self._on_pick_tcp)

    def recompute(self):
        self.tool.touch()
        self.doc.recompute()

    # ---------------------
    # button press handlers
    # ---------------------

    def _on_tool_off(self, *_):
        self.tool.Tool_offset = self.get_placement(self.tool_off)
        self.recompute()

    def _on_tcp_off(self, *_):
        self.tool.TCP_offset = self.get_placement(self.tcp_off)
        self.recompute()

    def _on_mass(self, *_):
        self.tool.Tool_mass = get_qsb(self.tool_mass)

    def _on_pick_flange(self):
        selx = Gui.Selection.getSelectionEx()
        if not selx or not selx[0].SubElementNames:
            fcl_err("Pick a face on a link inside assembly")
            return
        s = selx[0]
        if s.Object.TypeId != "App::Link":
            fcl_err("Selection must be on App::Link inside assembly")
            return
        sub = s.SubElementNames[0]
        # use the picked point if available
        picked = s.PickedPoints[0] if s.PickedPoints else App.Vector()
        ref = [s.Object, [sub]]
        vtx = UtilsAssembly.findElementClosestVertex(ref, picked)
        self.tool.Flange_link = UtilsAssembly.addVertexToReference(ref, vtx)
        self.refresh_flange_lbl()
        self.recompute()
        # self.tool_flange_lbl.setText(f"{s.Object.Label}.{sub}")

    # def _on_pick_shape(self):
    #     """
    #     pick tool's cad shape. binds pre-selected shape to curr tool fpo
    #     if no shape selected - import tool from fcstd file
    #     """
    #     from freecad.Robot_tools.App.rbt_tool import has_valid_shape

    #     sel = Gui.Selection.getSelection()
    #     shape = next((o for o in sel if has_valid_shape(o)), None)

    #     if shape is not None:
    #         # pre-selection: bind shape to default tool
    #         self.tool.Tool_shape = shape
    #         self.refresh_shape_lbl()
    #         self.recompute()
    #         return

    #     # noting selected: open file picker
    #     path, _ = QtGui.QFileDialog.getOpenFileName(
    #         Gui.getMainWindow(),
    #         "Select Tool File",
    #         "",
    #         "FreeCAD Files (*.FCStd *.fcstd);;All files (*)"
    #     )
    #     if not path:
    #         return

    #     prev_flange = self.tool.Flange_link if (self.tool.Flange_link
    #                                             and self.tool.Flange_link[0]) else None

    #     new_tool = import_tool(self.robot, path)
    #     if new_tool is None:
    #         return
    #     if prev_flange is not None:
    #         new_tool.Flange_link = prev_flange

    #     self.swap_tool(new_tool)
    #     self.refresh_shape_lbl()
    #     self.refresh_tflange_lbl()
    #     self.refresh_tcp_lbl()
    #     self.load()
    #     self.recompute()

    def _on_pick_shape(self):
        """
        pick tool's cad shape. binds pre-selected shape to curr tool fpo
        if no shape selected - import tool from fcstd file
        """
        from freecad.Robot_tools.App.rbt_tool import has_valid_shape

        sel = Gui.Selection.getSelection()
        shape = next((o for o in sel if has_valid_shape(o)), None)

        if shape is None:
            # noting selected: open file picker
            path, _ = QtGui.QFileDialog.getOpenFileName(
                Gui.getMainWindow(),
                "Select Tool File",
                "",
                "FreeCAD Files (*.FCStd *.fcstd);;All files (*)"
            )
            if not path:
                return

            try:
                shape = import_shape(self.robot, path)
            except Exception as e:
                fcl_err(f"tool import failed: {e}\n")
                return

            if shape is None:
                return

        self.tool.Tool_shape = shape
        self.refresh_shape_lbl()
        self.recompute()

    def _on_pick_tflange(self):
        selx = Gui.Selection.getSelectionEx()
        if not selx or not selx[0].SubElementNames:
            fcl_err("Pick a face on tool shape")
            return
        s = selx[0]
        sub = s.SubElementNames[0]
        ref = [s.Object, [sub]]
        picked = s.PickedPoints[0] if s.PickedPoints else App.Vector()
        vtx = UtilsAssembly.findElementClosestVertex(ref, picked)
        self.tool.Tool_flange_link = UtilsAssembly.addVertexToReference(
            ref, vtx)
        self.refresh_tflange_lbl()
        self.recompute()

    def _on_pick_tcp(self):
        selx = Gui.Selection.getSelectionEx()
        if not selx or not selx[0].SubElementNames:
            fcl_err("pick a vertex, edge or face on tool")
            return
        s = selx[0]
        sub = s.SubElementNames[0]
        picked = s.PickedPoints[0] if s.PickedPoints else App.Vector()
        ref = [s.Object, [sub]]
        vtx = UtilsAssembly.findElementClosestVertex(ref, picked)
        self.tool.TCP_link = UtilsAssembly.addVertexToReference(ref, vtx)
        self.refresh_tcp_lbl()
        self.recompute()

    def swap_tool(self, new_tool):
        """
        replace the existing tool with the new tool
        """
        old = self.tool
        self.tool = new_tool
        self.robot.Tools = [t for t in
                            self.robot.Tools if t is not old]
        # TODO: check if we have to remove old tools
        self.doc.removeObject(old.Name)

    def accept(self):
        if not self.inEdit:
            name = self.tool_name.text().strip()
            if name and name != self.tool.Label:
                self.tool.Label = name

        self.doc.commitTransaction()
        self.doc.recompute()
        Gui.Control.closeDialog()
        return True

    def reject(self):
        self.doc.abortTransaction()
        Gui.Control.closeDialog()
        return True

    # NOTE:
    # UI labels are W/P/R = roll/pitch/yaw about X/Y/Z (robotics convention).
    # App.Rotation(a, b, c) = yaw, pitch, roll about Z, Y, X.
    # So when writing:  Rotation(r, p, w)
    # When reading:     yaw, pitch, roll = .toEuler();  W=roll, P=pitch, R=yaw

    # def read_placement(self, boxes):
    #     x, y, z, w, p, r = (b.value() for b in boxes)
    #     return App.Placement(App.Vector(x, y, z), App.Rotation(w, p, r))


def run():

    robot = find_rob()
    if robot is None:
        fcl_err("no robot selected or found in curr file")
        return
    if not robot.Robot_joints:
        fcl_err("robot has no joints yet")
        return
    Gui.Control.showDialog(DefineTCP(robot))


def find_rob():
    sel = Gui.Selection.getSelection()
    robot = next((o for o in sel if hasattr(o, "Robot_joints")), None)
    if robot is not None:
        return robot

    fcl_warn("no rob-fpo pre-selected, choosing first rob from curr file")
    # TODO: better define this selection behaviour
    # select the first rob from active document
    doc = App.ActiveDocument
    if doc is None:
        return None
    robs = [o for o in doc.Objects
            if hasattr(o, "Robot_joints")]
    if len(robs) < 1:
        return None
    return robs[0]
