"""taskpanel_rbt_tool.py — Create Tool & TCP object."""

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore
from PySide import QtGui  # type: ignore

from freecad.Robot_tools.App.rbt_robot import is_robot
from freecad.Robot_tools.App.rbt_tool import (
    Tool, import_shape, has_valid_shape)
from freecad.Robot_tools.App.rbt_creator_geom import find_center
from freecad.Robot_tools.App.rbt_helpers_log import (
    fcl_err, fcl_warn)
from freecad.Robot_tools.Gui.rbt_helpers_ui import is_alive


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

    def cm_form(self):
        from freecad.Robot_tools.Gui.rbt_helpers_ui import load_panel_ui
        w = load_panel_ui("taskpanel_rbt_tool.ui")
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
            self.tool_flange_lbl.setText(f"{fl[0].Label}.{sub}"
                                         if sub else fl[0].Label)
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
        self.tool.Flange_link = find_center(s.Object, sub)
        self.refresh_flange_lbl()
        self.recompute()

    def _on_pick_shape(self):
        """
        pick tool's cad shape. binds pre-selected shape to curr tool fpo
        if no shape selected - import tool from fcstd file
        """

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
        self.tool.Tool_flange_link = find_center(s.Object, sub)
        self.refresh_tflange_lbl()
        self.recompute()

    def _on_pick_tcp(self):
        selx = Gui.Selection.getSelectionEx()
        if not selx or not selx[0].SubElementNames:
            fcl_err("pick a vertex, edge or face on tool")
            return
        s = selx[0]
        sub = s.SubElementNames[0]
        self.tool.TCP_link = find_center(s.Object, sub)
        self.refresh_tcp_lbl()
        self.recompute()

    def accept(self):
        if not is_alive(self.doc):
            return self.reject()
        if not self.inEdit:
            name = self.tool_name.text().strip()
            if name and name != self.tool.Label:
                self.tool.Label = name

        self.doc.commitTransaction()
        self.doc.recompute()
        Gui.Control.closeDialog()
        return True

    def reject(self):
        if is_alive(self.doc):
            self.doc.abortTransaction()
        Gui.Control.closeDialog()
        return True

    # NOTE:
    # UI labels are W/P/R = roll/pitch/yaw about X/Y/Z (robotics convention).
    # App.Rotation(a, b, c) = yaw, pitch, roll about Z, Y, X.
    # So when writing:  Rotation(r, p, w)
    # When reading:     yaw, pitch, roll = .toEuler();  W=roll, P=pitch, R=yaw


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
    robot = next((o for o in sel if is_robot(o)), None)
    if robot is not None:
        return robot

    fcl_warn("no rob-fpo pre-selected, choosing first rob from curr file")
    # TODO: better define this selection behaviour
    # select the first rob from active document
    doc = App.ActiveDocument
    if doc is None:
        return None
    robs = [o for o in doc.Objects if is_robot(o)]
    if len(robs) < 1:
        return None
    return robs[0]
