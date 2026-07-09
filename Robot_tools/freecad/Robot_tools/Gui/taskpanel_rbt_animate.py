"""Robot Animator.

Name: animate.py

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

# Layouts and Policy
from PySide import QtGui, QtCore  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QApplication,  QFrame, QGroupBox, QLabel,
    QHBoxLayout,  QGridLayout,  QSizePolicy)

from freecad.Robot_tools.Gui.rbt_helpers_ui import (
    cm_gbx, cm_btn,
    cm_lbl,
    cm_dspb, cm_slider, cm_toggle,
    cm_scroll,
    cm_tool_btn,
    getObjByName,
    msg_box
)

from freecad.Robot_tools.App.rbt_kine import (
    joint_limits_q_deg, set_q_deg, current_q_deg,
    save_home, home_q_deg, joint_dirs
)

from freecad.Robot_tools.App.rbt_helpers_log import (
    fcl_err, fcl_msg)

V3 = App.Vector
Rotation = App.Rotation
Placement = App.Placement


VEC0 = V3(0, 0, 0)

# ------------------------------------------------
#                 Module functions
# ------------------------------------------------


def create_link_row(dlg, gbx_l, row, fnt, jr, low, hi):
    """
    Create a row of jog widgets for one joint.
    params:
        - jr : 1-based joint index
    """
    nm = f"{jr:02d}"

    # Col 0 : Joint label
    lbl_jnt = cm_lbl(dlg, f"lbl_jnt{nm}", "Joint{nm}", fnt, 0)
    lbl_jnt.setFrameShape(QFrame.Shape.Panel)
    lbl_jnt.setFrameShadow(QFrame.Shadow.Sunken)
    lbl_jnt.setStyleSheet("QLabel {background-color: palette(base);"
                          "color: palette(text);}")
    lbl_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(lbl_jnt, row, 0, 1, 1)

    # Col 1 : Angle Spinbox for manual edits
    dspb_jnt = cm_dspb(dlg, f"dspb_jnt{nm}", fnt, sb_min=low, sb_max=hi)
    dspb_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(dspb_jnt, row, 1, 1, 1)

    # Col 2 : Angle reducing nudger

    btn_jnt_m = cm_tool_btn(dlg, f"btn_jnt_m{nm}", "", fnt)
    btn_jnt_m.setArrowType(QtCore.Qt.LeftArrow)
    btn_jnt_m.setToolTip(f"min: {low:g}°")
    btn_jnt_m.setFixedWidth(18)
    gbx_l.addWidget(btn_jnt_m, row, 3, 1, 1)

    # Col 3 : Angle Slider
    sl_jnt = cm_slider(dlg, f"sl_jnt{nm}", sl_min=low, sl_max=hi)
    gbx_l.addWidget(sl_jnt, row, 4, 1, 1)

    # col 4 : Angle increasing nudger
    btn_jnt_p = cm_tool_btn(dlg, f"btn_jnt_p{nm}", "", fnt)
    btn_jnt_p.setArrowType(QtCore.Qt.RightArrow)
    btn_jnt_p.setToolTip(f"max: {hi:g}°")
    btn_jnt_p.setFixedWidth(18)
    gbx_l.addWidget(btn_jnt_p, row, 5, 1, 1)

    # col 6 — flip toggle (checked == reversed direction)
    chk_flip = cm_toggle(dlg, f"chk_flip{nm}", fnt)
    gbx_l.addWidget(chk_flip, row, 7, 1, 1)

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

    # robot state mutations

    def set_joint_angle_clamped(self, j_idx, value):
        """Checks joint limits before setting joint angles"""
        low, high = joint_limits_q_deg(self.robot, j_idx)
        value = max(low, min(high, value))
        self.j_vals[j_idx] = value
        set_q_deg(self.robot, j_idx, value)
        return value

    def step_joint(self, j_idx, sign):
        """increment joint and return the value"""
        # rebase on the document first as
        # user may have moved the robot using
        # ik drag since last update
        self.j_vals[j_idx] = current_q_deg(self.robot)[j_idx]
        new_val = self.j_vals[j_idx] + sign * self.j_step
        self.j_vals[j_idx] = new_val
        self.set_joint_angle_clamped(j_idx, new_val)
        return new_val

    def go_home_pos(self):
        """Set home pos joint angle values"""
        for idx, q in enumerate(home_q_deg(self.robot)):
            # set the angles, in consideration of the joint directions
            self.set_joint_angle_clamped(idx, q)

    def sync_joints_from_doc(self):
        """
        Re-read j_vals from document (Offset2)
        """
        self.j_vals = list(current_q_deg(self.robot))

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
        self.switch_document(obj.Document.Name)
        self.ctrl.set_initial_pose()
        self.sync_panel_from_doc()

    def switch_document(self, doc_name):
        """Switch a FreeCAD document."""
        App.setActiveDocument(doc_name)
        App.ActiveDocument = App.getDocument(doc_name)
        Gui.ActiveDocument = Gui.getDocument(doc_name)

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
            low, hi = joint_limits_q_deg(self.robot, idx)
            create_link_row(self.form, tp_gb0l, brow, self.fnt,
                            idx + 1, low, hi)
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
                             "Go Home",  self.fnt)
        btn_home_set = cm_btn(self.form, "btn_home_set",
                              "Set Home", self.fnt)
        tb_lay.addWidget(btn_home_go)
        tb_lay.addWidget(btn_home_set)

        # spacing
        tb_lay.addSpacing(12)

        # reset and reload FPO buttons
        btn_jnts_res = cm_btn(self.form, "btn_jnts_res", "Reset",
                              self.fnt)
        btn_jnts_rld = cm_btn(self.form, "btn_jnts_rld", "Reload FPO",
                              self.fnt)
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

    def refresh_row_limits(self, j_idx):
        """
        Push q-joint space limits + values into row widgets
        """
        low, hi = joint_limits_q_deg(self.robot, j_idx)
        val = self.ctrl.j_vals[j_idx]
        nm = f"{j_idx + 1:02d}"
        # spinbox
        sb = getObjByName(self.form, f"dspb_jnt{nm}")
        if sb is not None:
            sb.blockSignals(True)
            sb.setRange(low, hi)
            sb.setValue(val)
            sb.blockSignals(False)

        # slider
        sl = getObjByName(self.form, f"sl_jnt{nm}")
        if sl is not None:
            sl.blockSignals(True)
            sl.setRange(int(low * sl._scale), int(hi * sl._scale))
            sl.setValue(int(val * sl._scale))
            sl.blockSignals(False)

        # increment/decremnt buttons
        bm = getObjByName(self.form, f"btn_jnt_m{nm}")
        if bm is not None:
            bm.setToolTip(f"min: {low:g}°")
        bp = getObjByName(self.form, f"btn_jnt_p{nm}")
        if bp is not None:
            bp.setToolTip(f"max: {hi:g}°")

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
        dirs = joint_dirs(self.robot)
        for j_n in range(self.ctrl.j_num):
            ck = getObjByName(self.form, f"chk_flip{j_n+1:02d}")
            if ck is not None:
                ck.blockSignals(True)
                ck.setChecked(dirs[j_n] == -1)
                ck.blockSignals(False)

        self.sync_panel_from_doc()
        for j_n in range(self.ctrl.j_num):
            self.refresh_row_limits(j_n)

    def _on_spin(self, j_idx, value):
        new_val = self.ctrl.set_joint_angle_clamped(j_idx, value)
        self.refresh_row(j_idx, new_val, skip="dspb")

    def _on_slider(self, j_idx, raw):
        sl = getObjByName(self.form, f"sl_jnt{j_idx + 1:02d}")
        new_val = self.ctrl.set_joint_angle_clamped(j_idx, raw / sl._scale)
        self.refresh_row(j_idx, new_val, skip="slider")

    def _on_flip(self, j_idx, checked):
        dirs = joint_dirs(self.robot)
        dirs[j_idx] = -1 if checked else 1

        # invalidate & trigger recreation of kin chain
        self.robot.Robot_joints_dir = dirs

        self.ctrl.sync_joints_from_doc()
        self.refresh_row_limits(j_idx)

    def _on_step_changed(self, value):
        self.ctrl.j_step = float(value)

    def _on_set_home(self):
        save_home(self.robot)

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

            # shorten the joint names to J<idx> to save UI space
            # set_wid_text(p_wid, f"lbl_jnt{j_n + 1:02d}", QLabel,
            #              f"<b>{jnt.Label}</b>")
            lbl = p_wid.findChild(QLabel, f"lbl_jnt{j_n + 1:02d}")
            if lbl is not None:
                lbl.setText(f"<b>J{j_n+1}</b>")
                lbl.setToolTip(jnt.Label)  # full name on hover

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

            # joint val and flip
            nm = f"{j_n + 1:02d}"
            sb = getObjByName(p_wid, f"dspb_jnt{nm}")
            sb.valueChanged.connect(lambda v, idx=j_n: self._on_spin(idx, v))

            sl = getObjByName(p_wid, f"sl_jnt{nm}")
            sl.valueChanged.connect(lambda raw,
                                    idx=j_n: self._on_slider(idx, raw))

            ck = getObjByName(p_wid, f"chk_flip{nm}")
            ck.setChecked(joint_dirs(self.robot)[j_n] == -1)
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
