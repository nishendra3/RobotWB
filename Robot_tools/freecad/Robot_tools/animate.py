"""Robot Animator.

Name: animator.py

See Changelog after import statements.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""
__version__ = "0.07"
__build__ = "20260508_1245"


import pathlib

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

# Layouts and Policy
from PySide import QtGui  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QApplication,  QFrame, QGroupBox, QLabel,
    QHBoxLayout,  QGridLayout,  QSizePolicy)

from freecad.Robot_tools.rbt_helpers_ui import (
    cm_gbx, cm_btn,
    cm_lbl,
    cm_dspb, cm_slider, cm_toggle,
    cm_scroll,
    getObjByName, set_wid_text,
    msg_box
)

from freecad.Robot_tools.rbt_helpers_doc import (
    switch_document
)

from freecad.Robot_tools.rbt_helpers_math import roundrot, roundvec

"""
----------------------------------------
Changelog:
----------------------------------------
v0.02 - Converted to make it compatible with the WB and the FPO.
v0.03 - Added direction field from FPO.
v0.04 - Some improvements.
v0.05 - Reworked 'Reload FPO Data' button to avoid unknown problem that causes
        a buggy  movement commands (it don't honour the steps)
v0.06 - Starting to adapt to the robot_FPO
v0.07 - Some hacks to reset the touched overlay icon. Added a check of joints
        dir data, to avoid initialization error.
v0.09 - Modded to work on robot_test too.
      - Some fixes in logic if no Robot_FPO is selected.
      - Message boxes fixed to show program name and the context.
"""

fcl_err = App.Console.PrintError
fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning


MODULE_PATH = pathlib.Path(__file__).parent

if str(MODULE_PATH.stem) == "Robot_test":
    pg_name = "Robot test"
    fcl_msg("Running on Robot Test\n")
elif str(MODULE_PATH.stem) == "Robot_tools":
    pg_name = "Robot Tools"
    fcl_msg("Running on Robot Tools\n")
else:
    pg_name = "Standalone"
    fcl_msg("Running from command line\n")


V3 = App.Vector
Rotation = App.Rotation
Placement = App.Placement


VEC0 = V3(0, 0, 0)
ROT0 = Rotation(0, 0, 0)

# ------------------------------------------------
#                 Module functions
# ------------------------------------------------


def create_link_row(dlg, gbx_l, row, fnt, jr, joint_nm, low, hi, sl_scale=1):
    """Create a link of buttons for a link."""
    # txt_wd = cm_lbl(dlg, f"lbl_j{jr}", joint_nm, fnt, 0)
    # gbx_l.addWidget(txt_wd, row, 0, 1, 1)

    # Col 0 : Joint label
    lbl_jnt = cm_lbl(dlg, f"lbl_jnt{jr}", "", fnt, 0)
    lbl_jnt.setFrameShape(QFrame.Shape.Panel)
    lbl_jnt.setFrameShadow(QFrame.Shadow.Sunken)
    lbl_jnt.setStyleSheet("QLabel {background-color: palette(base);"
                          "color: palette(text);}")
    lbl_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(lbl_jnt, row, 0, 1, 1)

    # Col 1 : Angle Spinbox for manual edits
    dspb_jnt = cm_dspb(dlg, f"dspb_jnt{jr}", fnt, sb_min=low, sb_max=hi)
    dspb_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(dspb_jnt, row, 1, 1, 1)

    # Col 2 : Min limit label
    lbl_jmin = cm_lbl(dlg, f"lbl_jmin{jr}", f"{low:g}°", fnt, 0, l_aln=2)
    gbx_l.addWidget(lbl_jmin, row, 2, 1, 1)

    # Col 3 : Angle reducing nudger
    btn_jnt_m = cm_btn(dlg, f"btn_jnt_m{jr}", "❮", fnt, 0)
    btn_jnt_m.setFixedWidth(22)
    # was: setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(btn_jnt_m, row, 3, 1, 1)

    # Col 4 : Angle Slider
    sl_jnt = cm_slider(dlg, f"sl_jnt{jr}", sl_min=low, sl_max=hi)
    gbx_l.addWidget(sl_jnt, row, 4, 1, 1)

    # col 5 : Angle increasing nudger
    btn_jnt_p = cm_btn(dlg, f"btn_jnt_p{jr}", "❯", fnt, 0)
    btn_jnt_p.setFixedWidth(22)
    # was: setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(btn_jnt_p, row, 5, 1, 1)

    # col 6 : max limit label
    lbl_jmax = cm_lbl(dlg, f"lbl_jmax{jr}", f"{hi:g}°", fnt, 0)
    gbx_l.addWidget(lbl_jmax, row, 6, 1, 1)

    # col 7 — flip toggle (checked == reversed direction)
    chk_flip = cm_toggle(dlg, f"chk_flip{jr}", fnt)
    gbx_l.addWidget(chk_flip, row, 7, 1, 1)


def print_joints(obj):
    """Show joints info on Report View."""
    for joint in obj.Joints:
        # fcl_msg(dir(joint))
        if joint.JointType == "Revolute":
            jref1 = joint.Reference1
            jpl1 = joint.Placement1
            jpl1_b = roundvec(jpl1.Base)
            jpl1_r = roundrot(jpl1.Rotation.toEuler())
            jref2 = joint.Reference2
            jpl2 = joint.Placement2
            jpl2_b = roundvec(jpl2.Base)
            jpl2_r = roundrot(jpl2.Rotation.toEuler())
            msg = (
                f"-- Joint {joint.Name} \n"
                f"- Type = {joint.JointType}\n"
                f"- Reference1 = {jref1}\n"
                f"- Placement1.Base = {jpl1_b}\n"
                f"- Placement1.Rot = {jpl1_r}\n"
                f"- Reference2 = {jref2}\n"
                f"- Placement2.Base = {jpl2_b}\n"
                f"- Placement2.Rot = {jpl2_r}\n"
                "\n"
                )
            fcl_msg(msg)


def find_joint(obj, name):
    for joint in obj.Joints:
        # fcl_msg(dir(joint))
        if joint.JointType == "Revolute":
            if joint.Name == name:
                return joint
    return None


# ---------------------------------------------
#             App Layer
# ---------------------------------------------

class AnimationController:
    """Core app logic for robot FPO interaction"""

    def __init__(self, robot_obj):
        self.robot = robot_obj
        joints = robot_obj.Robot_joints
        self.j_num = len(joints)  # number of jonits
        self.j_nms = [f"Joint{n + 1:02d}" for n in range(self.j_num)]  # jnames
        self.j_step = 1.0  # step increment size for angles
        self.j_vals = [0.0] * self.j_num  # joint values

        # default vals if no <<Robot_joints_dir>> yet
        if not robot_obj.Robot_joints_dir:
            robot_obj.Robot_joints_dir = [1] * self.j_num
        self.j_dirs = list(robot_obj.Robot_joints_dir)

        # if not hasattr(robot_obj, "Robot_home_pos"):
        #     robot_obj.addProperty(
        #         "App::PropertyFloatList", "Robot_home_pos", "Robot",
        #         "Robot home pose (one angle per joint)")
        # if not robot_obj.Robot_home_pos:
        #     robot_obj.Robot_home_pos = list(self.j_vals)
        # self.j_home = list(robot_obj.Robot_home_pos)

        # init home pose if it exists
        if not robot_obj.Robot_home_pos:
            robot_obj.Robot_home_pos = self.j_vals
        self.j_home = list(robot_obj.Robot_home_pos)

    # helpers

    def get_joint_limits(self, j_idx):
        """Returns (min, max) range for current angle"""
        jnt = self.robot.Robot_joints[j_idx]
        low = jnt.AngleMin if getattr(jnt, "EnableAngleMin", False) else -180
        high = jnt.AngleMax if getattr(jnt, "EnableAngleMax", False) else 180
        return float(low), float(high)

    # robot state mutations

    def set_joint_angle(self, j_idx, value):
        """Sets joint angle to given value"""
        joint = self.robot.Robot_joints[j_idx]
        angle = value * self.j_dirs[j_idx]  # flip ccw/cw if checkbox enabled
        joint.Offset2 = Placement(VEC0, Rotation(angle, 0, 0))
        joint.recompute()

        #  link the tool
        tool = getattr(self.robot, "Active_tool", None)
        if tool:
            tool.recompute()

    def set_joint_angle_clamped(self, j_idx, value):
        """Checks joint limits before setting joint angles"""
        low, high = self.get_joint_limits(j_idx)
        value = max(low, min(high, value))
        self.j_vals[j_idx] = value
        self.set_joint_angle(j_idx, value)
        return value

    def step_joint(self, j_idx, sign):
        """increment joint and return the value"""
        # rebase on the document first as
        # user may have moved the robot using
        # ik drag since last update
        yaw, _, _ = self.robot.Robot_joints[j_idx].Offset2.Rotation.toEuler()
        self.j_vals[j_idx] = float(yaw) / self.j_dirs[j_idx]
        new_val = self.j_vals[j_idx] + sign * self.j_step
        self.j_vals[j_idx] = new_val
        self.set_joint_angle_clamped(j_idx, new_val)
        return new_val

    def save_home_pos(self):
        """Saves the curr pose as home position"""
        # self.j_home = list(self.j_vals)
        # store the doc-space angles so we are not affected
        # by the direction flips later
        self.j_home = [float(j.Offset2.Rotation.toEuler()[0])
                       for j in self.robot.Robot_joints]
        self.robot.Robot_home_pos = self.j_home

    def go_home_pos(self):
        """Set home pos joint angle values"""
        for idx, val in enumerate(self.j_home):
            # set the angles, in consideration of the joint directions
            self.set_joint_angle_clamped(idx, val / self.j_dirs[idx])

    def sync_joints_from_doc(self):
        """
        Re-read j_vals from document (Offset2)
        """
        for i, jnt in enumerate(self.robot.Robot_joints):
            yaw, _, _ = jnt.Offset2.Rotation.toEuler()

            # convert from doc-space to panel space
            self.j_vals[i] = float(yaw / self.j_dirs[i])

    def reset_joints(self):
        """reset joints to original val"""
        asm = self.robot.Robot_assembly
        if asm is None:
            return

        for n, jnt in enumerate(self.robot.Robot_joints):
            jnt.Offset2 = Placement(VEC0, Rotation())
            self.j_vals[n] = 0.0
        asm.recompute()

    def set_initial_pose(self):
        """force apply offset2 with recompute-twice trick"""
        asm = self.robot.Robot_assembly
        for jnt in self.robot.Robot_joints:
            of2 = jnt.Offset2
            jnt.Offset2 = Placement(VEC0, Rotation(1, 0, 0)).multiply(of2)
            asm.recompute()
            jnt.Offset2 = of2
            asm.recompute()

    def reload_dirs(self):
        """reload joints directory"""
        self.j_dirs = list(self.robot.Robot_joints_dir)

    # debug

    def debug_dump_state(self, op_nm):
        fcl_msg(
            f"----- {op_nm} -----\n"
            f"- j_num : {self.j_num}\n"
            f"- j_dirs: {self.j_dirs}\n"
            f"- j_nms : {self.j_nms}\n"
            f"- j_step: {self.j_step}\n"
            f"- j_vals: {self.j_vals}\n"
            "--------------------------------\n"
        )

# ---------------------------------------------
#             GUI Layer
# ---------------------------------------------


class AnimationTaskPanel:
    """Task panel for Robot Animator."""
    #
    grb_ss = (
        ""
        "QGroupBox{"
        "    font-weight: bold;"
        "    font-style: normal;"
        "    text-decoration: none;"
        "}"
    )
    wrl_file = None
    csv_file = None

    def __init__(self, robot_obj):
        """Canonical Init & set robot obj"""
        self.robot = robot_obj
        self.initUI()

    def initUI(self):
        """Init UI."""
        # global work_dir

        fnt = QApplication.font("QMessageBox")
        self.fnt = fnt
        self.form = QtGui.QWidget()
        self.form.setWindowTitle("Robot Animator")
        self.form.setObjectName("RobotAnimationPanel")

        self.form_lay = QGridLayout(self.form)

        obj = self.robot
        row = 0
        lbl_rob_id = cm_lbl(self.form, "lbl_rob_id", f"<b>{obj.Name}</b>",
                            self.fnt, 0)
        self.form_lay.addWidget(lbl_rob_id, row, 0, 1, 4)
        row += 1
        self.ctrl = AnimationController(obj)
        tp_gb0 = self.create_joint_ui()
        # wrap in a scrollable area
        scroll = cm_scroll(self.form, "tp_gb0_scroll", tp_gb0)
        self.form_lay.addWidget(scroll, row, 0, 1, 4)
        # self.form_lay.addWidget(tp_gb0, row, 0, 1, 4)
        self.read_joints_data()
        switch_document(obj.Document.Name)
        self.ctrl.set_initial_pose()
        self.sync_panel_from_doc()

    def create_joint_ui(self):
        """Create Joint UI."""
        tp_gb0, tp_gb0l = cm_gbx(self.form, "tp_gb0", "Joint Axes Jog")
        tp_gb0.setStyleSheet(self.grb_ss)

        # -- Header Row --
        lbl_h0 = cm_lbl(self.form, "lbl_h0", "<b>Axis</b>", self.fnt, 1)
        tp_gb0l.addWidget(lbl_h0, 0, 0, 1, 1)

        lbl_h1 = cm_lbl(self.form, "lbl_h1", "<b>Angle</b>", self.fnt,
                        1, l_aln=1)
        tp_gb0l.addWidget(lbl_h1, 0, 1, 1, 1)

        lbl_h2 = cm_lbl(self.form, "lbl_h2", "<b>Position</b>", self.fnt,
                        1, l_aln=1)
        tp_gb0l.addWidget(lbl_h2, 0, 2, 1, 5)

        lbl_h3 = cm_lbl(self.form, "lbl_h3", "<b>Flip</b>", self.fnt,
                        1, l_aln=1)
        tp_gb0l.addWidget(lbl_h3, 0, 7, 1, 1)

        # -- Joint Rows --
        brow = 1
        for idx, jnm in enumerate(self.ctrl.j_nms):
            low, hi = self.ctrl.get_joint_limits(idx)
            create_link_row(self.form, tp_gb0l, brow, self.fnt,
                            f"{idx + 1:02d}", jnm, low, hi)
            brow += 1

        # -- one row gap --
        brow += 1

        # -- bottom row btns --
        tb_lay = QHBoxLayout()
        tb_lay.setContentsMargins(0, 0, 0, 0)

        lbl_step = cm_lbl(self.form, "lbl_step", "<b>Step</b>", self.fnt, 1)
        tb_lay.addWidget(lbl_step)

        # entry box to set step size
        dspb_step = cm_dspb(self.form, "dspb_step", self.fnt,
                            sb_min=0.01, sb_max=90.0, sb_dec=2, sb_step=0.1)
        dspb_step.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
        dspb_step.setValue(self.ctrl.j_step)
        tb_lay.addWidget(dspb_step)

        # push the rest to the right
        tb_lay.addStretch(1)

        # home pos buttons
        btn_home_go = cm_btn(self.form, "btn_home_go",
                             "Go Home",  self.fnt, 0)
        btn_home_set = cm_btn(self.form, "btn_home_set",
                              "Set Home", self.fnt, 0)
        tb_lay.addWidget(btn_home_go)
        tb_lay.addWidget(btn_home_set)

        # spacing
        tb_lay.addSpacing(12)

        # reset and reload FPO buttons
        btn_jnts_res = cm_btn(self.form, "btn_jnts_res", "Reset",
                              self.fnt, 0)
        btn_jnts_rld = cm_btn(self.form, "btn_jnts_rld", "Reload FPO",
                              self.fnt, 0)
        btn_jnts_rld.setToolTip("Reload FPO data ('joints directions')")
        tb_lay.addWidget(btn_jnts_res)
        tb_lay.addWidget(btn_jnts_rld)

        tp_gb0l.addLayout(tb_lay, brow, 0, 1, 8)

        # buttons connections
        btn_jnts_res.clicked.connect(self._on_reset_joints)
        btn_jnts_rld.clicked.connect(self._on_reload_dirs)
        btn_home_go.clicked.connect(self._on_go_home)
        btn_home_set.clicked.connect(self._on_set_home)
        dspb_step.valueChanged.connect(self._on_step_changed)

        # debug button on its own row
        # brow += 1
        # btn_dbg_div = cm_btn(self.form, "btn_dbg_div", "Dbg IntVar",
        #                      self.fnt, 0)
        # btn_dbg_div.clicked.connect(self._on_debug_dump)
        # tp_gb0l.addWidget(btn_dbg_div, brow, 0, 1, 2)

        # column stretch handler
        # only let the slider increase in size
        for c in (0, 1, 2, 3, 5, 6, 7):
            tp_gb0l.setColumnStretch(c, 0)
        tp_gb0l.setColumnStretch(4, 1)

        tp_gb0.setLayout(tp_gb0l)
        return tp_gb0

    def refresh_row(self, j_idx, value, skip=None):
        """reloads the row based on current state"""
        nm = f"{j_idx + 1:02d}"
        if skip != "dspb":
            sb = getObjByName(self.form, f"dspb_jnt{nm}")
            if sb is not None:
                sb.blockSignals(True)
                sb.setValue(value)
                sb.blockSignals(False)
        if skip != "slider":
            sl = getObjByName(self.form, f"sl_jnt{nm}")
            if sl is not None:
                sl.blockSignals(True)
                sl.setValue(int(value * sl._scale))
                sl.blockSignals(False)

    def sync_panel_from_doc(self):
        """Pull joint state from the document into ctrl and the widgets."""
        self.ctrl.sync_joints_from_doc()
        for idx in range(self.ctrl.j_num):
            self.refresh_row(idx, self.ctrl.j_vals[idx])

    # --------------------------------------------
    #         control button wrappers
    # --------------------------------------------

    def _on_step(self, j_idx, sign):
        new_val = self.ctrl.step_joint(j_idx, sign)
        self.refresh_row(j_idx, new_val)

    def _on_reset_joints(self):
        self.ctrl.reset_joints()
        for j_n in range(self.ctrl.j_num):
            self.refresh_row(j_n, 0.0)

    def _on_reload_dirs(self):
        self.ctrl.reload_dirs()
        for j_n in range(self.ctrl.j_num):
            ck = getObjByName(self.form, f"chk_flip{j_n + 1:02d}")
            if ck is not None:
                ck.blockSignals(True)
                ck.setChecked(self.ctrl.j_dirs[j_n] == -1)
                ck.blockSignals(False)
        self.sync_panel_from_doc()

    def _on_debug_dump(self):
        self.ctrl.debug_dump_state("Dbg Values")

    def _on_spin(self, j_idx, value):
        new_val = self.ctrl.set_joint_angle_clamped(j_idx, value)
        self.refresh_row(j_idx, new_val, skip="dspb")

    def _on_slider(self, j_idx, raw):
        sl = getObjByName(self.form, f"sl_jnt{j_idx + 1:02d}")
        new_val = self.ctrl.set_joint_angle_clamped(j_idx, raw / sl._scale)
        self.refresh_row(j_idx, new_val, skip="slider")

    def _on_flip(self, j_idx, checked):
        self.ctrl.j_dirs[j_idx] = -1 if checked else 1

        # mirror to FPO so it persists
        dirs = list(self.robot.Robot_joints_dir)

        dirs[j_idx] = self.ctrl.j_dirs[j_idx]
        self.robot.Robot_joints_dir = dirs

        # reapply angle on flip
        self.ctrl.set_joint_angle(j_idx, self.ctrl.j_vals[j_idx])

    def _on_step_changed(self, value):
        self.ctrl.j_step = float(value)

    def _on_set_home(self):
        self.ctrl.save_home_pos()

    def _on_go_home(self):
        self.ctrl.go_home_pos()
        for idx in range(self.ctrl.j_num):
            self.refresh_row(idx, self.ctrl.j_vals[idx])

    # --------------------------------------------
    #           Robot state interface
    # --------------------------------------------
    def read_joints_data(self, dbg_s=False):
        """Read joint data."""
        # dbg_s = True  # DBG
        p_wid = self.form.findChild(QGroupBox, "tp_gb0_wd")
        if p_wid is None:
            return
        else:
            # fcl_msg(wid.children())  # DBG
            pass

        for j_n, jnt in enumerate(self.ctrl.robot.Robot_joints):
            if dbg_s:
                fcl_msg(f"{j_n} {jnt.Label}")
            set_wid_text(p_wid, f"lbl_jnt{j_n + 1:02d}", QLabel,
                         f"<b>{jnt.Label}</b>")
            # Assign to increase angle button the action
            btn_p_nm = f"btn_jnt_p{j_n + 1:02d}"
            btn_p = getObjByName(p_wid, btn_p_nm)
            if btn_p is not None:
                # lambda workaround taken from BIM workbench
                btn_p.clicked.connect(lambda _checked=False,
                                      idx=j_n: self._on_step(idx, +1))
            else:
                fcl_err(f"button + for {btn_p_nm} not found!")
            # Assign to decrease angle button the action
            btn_m_nm = f"btn_jnt_m{j_n + 1:02d}"
            btn_m = getObjByName(p_wid, btn_m_nm)
            if btn_m is not None:
                # lambda workaround taken from BIM workbench
                btn_m.clicked.connect(lambda _checked=False,
                                      idx=j_n: self._on_step(idx, -1))
            else:
                fcl_err(f"button - for {btn_m_nm} not found!")
            # set the dir label
            set_wid_text(
                p_wid, f"lbl_jdir{j_n + 1:02d}", QLabel,
                f"<b>{self.ctrl.j_dirs[j_n]}</b>")

            # joint val and flip
            nm = f"{j_n + 1:02d}"
            sb = getObjByName(p_wid, f"dspb_jnt{nm}")
            sb.valueChanged.connect(lambda v, idx=j_n: self._on_spin(idx, v))

            sl = getObjByName(p_wid, f"sl_jnt{nm}")
            sl.valueChanged.connect(lambda raw,
                                    idx=j_n: self._on_slider(idx, raw))

            ck = getObjByName(p_wid, f"chk_flip{nm}")
            ck.setChecked(self.ctrl.j_dirs[j_n] == -1)
            ck.toggled.connect(lambda c, idx=j_n: self._on_flip(idx, c))


    # --------------------------------------------
    #      Standard Taskpanel functions
    # --------------------------------------------
    def getStandardButtons(self):
        """Draw a single X close btn"""
        return QtGui.QDialogButtonBox.Close

    def reject(self):
        """Runs when user closes taskpanel"""
        Gui.Control.closeDialog()
        return True
    # --------------------------------------------


def run():
    sel = Gui.Selection.getSelection()
    fnt = QApplication.font("QMessageBox")
    if len(sel) != 1 \
            or sel[0].TypeId != "App::FeaturePython" \
            or not sel[0].Name.startswith("Robot_FPO"):

        msg_box(Gui.getMainWindow(), "Robot", fnt,
                "<b>Robot Selection</b>"
                "<br><br>"
                "You must selecta 'Robot_FPO' from the tree")
        # fcl_err(f"sel:{str(len(sel))}, typeID:{sel[0].TypeId},
        # name:{sel[0].Name}")
        return

    if not hasattr(sel[0], "Robot_home_pos"):
        msg_box(Gui.getMainWindow(), "Robot", fnt,
                "<b>Robot Missing Properties</b>"
                "<br><br>"
                "You must recreate 'Robot_FPO'")

    # sel[0] contains the robot fpo
    panel = AnimationTaskPanel(sel[0])
    Gui.Control.showDialog(panel)
