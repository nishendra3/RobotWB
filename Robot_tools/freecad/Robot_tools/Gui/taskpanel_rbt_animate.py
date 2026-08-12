"""Robot Animator.

Name: animate.py

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

from contextlib import contextmanager

# Layouts and Policy
from PySide import QtGui, QtCore  # type: ignore
from PySide.QtCore import QTimer  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QWidget, QApplication,  QFrame, QGroupBox, QLabel,
    QHBoxLayout,  QGridLayout,  QSizePolicy)

from freecad.Robot_tools.Gui.rbt_helpers_ui import (
    cm_gbx, cm_btn,
    cm_lbl,
    cm_dspb, cm_slider, cm_toggle,
    cm_scroll,
    cm_tool_btn,
    getObjByName,
    msg_box,
    is_alive
)

from freecad.Robot_tools.App.rbt_robot import is_robot

from freecad.Robot_tools.App.rbt_kine import (
    joint_limits_q_deg, curr_joint_vals_doc,
    save_home, home_q_deg, joint_dirs,
    set_zero_pose, hold_part_placements
)
from freecad.Robot_tools.App.rbt_kine_types import (
    PRISMATIC, REVOLUTE, FIXED, joint_type_FC2WB)
from freecad.Robot_tools.App.rbt_helpers_log import (
    fcl_err, fcl_msg)
from freecad.Robot_tools.App.rbt_kine import set_q
from freecad.Robot_tools.App.rbt_kine_joints import set_joint_cfg

V3 = App.Vector
Rotation = App.Rotation
Placement = App.Placement


VEC0 = V3(0, 0, 0)

# ------------------------------------------------
#                 Module functions
# ------------------------------------------------


def create_link_row(dlg, gbx_l, row, fnt, jr, low, hi, jtype):
    """
    Create a row of jog widgets for one joint.
    params:
        - jr : 1-based joint index
    """
    nm = f"{jr:02d}"
    unit = " mm" if jtype == PRISMATIC else "°"

    # Col 0 : Joint label
    lbl_jnt = cm_lbl(dlg, f"lbl_jnt{nm}", f"Joint{nm}", fnt, 0)
    lbl_jnt.setFrameShape(QFrame.Shape.Panel)
    lbl_jnt.setFrameShadow(QFrame.Shadow.Sunken)
    lbl_jnt.setStyleSheet("QLabel {background-color: palette(base);"
                          "color: palette(text);}")
    lbl_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(lbl_jnt, row, 0, 1, 1)

    # Col 1 : Angle Spinbox for manual edits
    dspb_jnt = cm_dspb(dlg, f"dspb_jnt{nm}", fnt, sb_min=low,
                       sb_max=hi, sb_suf=unit)
    dspb_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(dspb_jnt, row, 1, 1, 1)

    # Col 2 : Angle reducing nudger

    btn_jnt_m = cm_tool_btn(dlg, f"btn_jnt_m{nm}", "", fnt)
    btn_jnt_m.setArrowType(QtCore.Qt.LeftArrow)
    btn_jnt_m.setToolTip(f"min: {low:g}{unit}")
    btn_jnt_m.setFixedWidth(18)
    gbx_l.addWidget(btn_jnt_m, row, 3, 1, 1)

    # Col 3 : Angle Slider
    sl_jnt = cm_slider(dlg, f"sl_jnt{nm}", sl_min=low, sl_max=hi)
    gbx_l.addWidget(sl_jnt, row, 4, 1, 1)

    # col 4 : Angle increasing nudger
    btn_jnt_p = cm_tool_btn(dlg, f"btn_jnt_p{nm}", "", fnt)
    btn_jnt_p.setArrowType(QtCore.Qt.RightArrow)
    btn_jnt_p.setToolTip(f"max: {hi:g}{unit}")
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
        self.j_nms = [f"Joint{n:02d}" for n in range(self.j_num)]  # jnames
        self.j_step = 1.0  # step increment size for angles
        self.j_vals = [0.0] * self.j_num  # joint values

    # robot state mutations

    def set_joint_angle_clamped(self, j_idx, value):
        """Checks joint limits before setting joint angles"""
        q = curr_joint_vals_doc(self.robot)
        q[j_idx] = float(value)
        self.j_vals = set_q(self.robot, q, clamp=True, preview=True)
        return self.j_vals[j_idx]

    def step_joint(self, j_idx, sign):
        """
        increment joint and return the value
        """
        new_val = self.j_vals[j_idx] + sign * self.j_step
        return self.set_joint_angle_clamped(j_idx, new_val)

    def commit_joints(self):
        """
        write j_vals into Offset2
        """
        set_q(self.robot, self.j_vals)

    def go_home_pos(self):
        """
        home pos, clampled to joint limits
        """
        self.j_vals = set_q(self.robot, home_q_deg(self.robot),
                            clamp=True)

    def sync_joints_from_doc(self):
        """
        Re-read j_vals from document (Offset2)
        """
        self.j_vals = list(curr_joint_vals_doc(self.robot))

    def reset_joints(self):
        """reset joints to null val"""
        self.j_vals = set_q(self.robot, [0.0] * len(self.j_vals))

    def set_initial_pose(self):
        """force apply offset2 with recompute-twice trick"""
        asm = self.robot.Robot_assembly
        for jnt in self.robot.Robot_joints:
            jtype = joint_type_FC2WB(jnt.JointType)

            # skip fixed joints
            if jtype == FIXED:
                continue

            of2 = jnt.Offset2
            if jtype == PRISMATIC:
                nudge = Placement(V3(0, 0, 1), Rotation())  # 1 mm
            elif jtype == REVOLUTE:
                nudge = Placement(VEC0, Rotation(1, 0, 0))  # 1 deg

            with hold_part_placements(asm):
                jnt.Offset2 = nudge.multiply(of2)
            asm.recompute()
            with hold_part_placements(asm):
                jnt.Offset2 = of2
            asm.recompute()

# ---------------------------------------------
#             GUI Layer
# ---------------------------------------------


class RobotControlWidget(QWidget):
    """
    Widget for Robot Control
    """
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
        super().__init__()
        self.robot = robot_obj
        self.page_key = (robot_obj.Document.Name, robot_obj.Name)
        self._writing = False  # for observer echo guard

        # flags for slider movement
        self._sl_pending = None
        self._sl_timer = QTimer(self)
        self._sl_timer.setSingleShot(True)
        self._sl_timer.timeout.connect(self._apply_slider)

        self.initUI()

    @contextmanager
    def writing(self):
        self._writing = True
        try:
            yield
        finally:
            self._writing = False

    def initUI(self):
        """Init UI."""
        # global work_dir

        fnt = QApplication.font("QMessageBox")
        self.fnt = fnt

        self.setWindowTitle("Robot Animator")
        self.setObjectName("RobotAnimationPanel")

        self.form_lay = QGridLayout(self)

        obj = self.robot
        row = 0
        lbl_rob_id = cm_lbl(self, "lbl_rob_id", f"<b>{obj.Name}</b>",
                            self.fnt, 0)
        self.form_lay.addWidget(lbl_rob_id, row, 0, 1, 4)
        row += 1
        self.ctrl = AnimationController(obj)
        tp_gb0 = self.create_joint_ui()
        # wrap in a scrollable area
        scroll = cm_scroll(self, "tp_gb0_scroll", tp_gb0)
        self.form_lay.addWidget(scroll, row, 0, 1, 4)

        # self.form_lay.addWidget(tp_gb0, row, 0, 1, 4)
        self.read_joints_data()
        self.ctrl.set_initial_pose()
        self.sync_panel_from_doc()

    def create_joint_ui(self):
        """Create Joint UI."""
        tp_gb0, tp_gb0l = cm_gbx(self, "tp_gb0", "Joint Axes Jog")
        tp_gb0.setStyleSheet(self.grb_ss)

        # -- Header Row --
        lbl_h0 = cm_lbl(self, "lbl_h0", "<b>Axis</b>", self.fnt, 1)
        tp_gb0l.addWidget(lbl_h0, 0, 0, 1, 1)

        lbl_h1 = cm_lbl(self, "lbl_h1", "<b>Value</b>", self.fnt,
                        1, l_aln=1)
        tp_gb0l.addWidget(lbl_h1, 0, 1, 1, 1)

        lbl_h2 = cm_lbl(self, "lbl_h2", "<b>Position</b>", self.fnt,
                        1, l_aln=1)
        tp_gb0l.addWidget(lbl_h2, 0, 2, 1, 5)

        lbl_h3 = cm_lbl(self, "lbl_h3", "<b>Flip</b>", self.fnt,
                        1, l_aln=1)
        tp_gb0l.addWidget(lbl_h3, 0, 7, 1, 1)

        # -- Joint Rows --
        brow = 1
        for idx, jnm in enumerate(self.ctrl.j_nms):
            jtype = joint_type_FC2WB(self.robot.Robot_joints[idx].JointType)

            # skip making rows for fixed joints
            if jtype == FIXED:
                continue

            low, hi = joint_limits_q_deg(self.robot, idx)
            create_link_row(self, tp_gb0l, brow, self.fnt,
                            idx + 1, low, hi, jtype)
            brow += 1

        # -- one row gap --
        brow += 1

        # -- bottom row btns --
        tb1 = QHBoxLayout()
        tb2 = QHBoxLayout()
        tb1.setContentsMargins(0, 0, 0, 0)
        tb2.setContentsMargins(0, 0, 0, 0)

        tp_gb0l.addLayout(tb1, brow, 0, 1, 8)
        tp_gb0l.addLayout(tb2, brow+1, 0, 1, 8)

        # push from the left
        tb1.addStretch(1)

        lbl_step = cm_lbl(self, "lbl_step", "<b>Step</b>", self.fnt, 1)
        tb1.addWidget(lbl_step)

        # entry box to set step size
        dspb_step = cm_dspb(self, "dspb_step", self.fnt,
                            sb_min=0.01, sb_max=90.0,
                            sb_dec=2, sb_step=0.1, sb_suf="")
        dspb_step.setToolTip("jog step in joint units (° | mm)")
        dspb_step.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
        dspb_step.setValue(self.ctrl.j_step)
        tb1.addWidget(dspb_step)

        # reset and reload FPO buttons
        btn_jnts_res = cm_btn(self, "btn_jnts_res", "Reset",
                              self.fnt)
        btn_jnts_rld = cm_btn(self, "btn_jnts_rld", "Reload FPO",
                              self.fnt)
        btn_jnts_rld.setToolTip("Reload FPO data ('joints directions')")
        tb1.addWidget(btn_jnts_res)
        tb1.addWidget(btn_jnts_rld)

        # push from the right
        tb1.addStretch(1)

        # push from the left
        tb2.addStretch(1)

        # home pos buttons
        btn_home_go = cm_btn(self, "btn_home_go",
                             "Go Home",  self.fnt)
        btn_home_set = cm_btn(self, "btn_home_set",
                              "Set Home", self.fnt)
        btn_zero_set = cm_btn(self, "btn_zero_set", "Set Zero",
                              self.fnt)
        btn_zero_set.setToolTip(
            "Make the current pose the robot's zero pose: all joints\n"
            "read 0 here, and limits/home are measured from it\n"
            "('mastering' on industrial robots).\n"
            "The Assembly joint dialog keeps showing raw values")

        tb2.addWidget(btn_home_go)
        tb2.addWidget(btn_home_set)
        tb2.addWidget(btn_zero_set)

        # push from the left
        tb2.addStretch(1)

        # spacing
        tb1.addSpacing(12)
        tb2.addSpacing(12)

        # buttons connections
        btn_jnts_res.clicked.connect(self._on_reset_joints)
        btn_jnts_rld.clicked.connect(self._on_reload_dirs)
        btn_home_go.clicked.connect(self._on_go_home)
        btn_home_set.clicked.connect(self._on_set_home)
        btn_zero_set.clicked.connect(self._on_set_zero)
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
            sb = getObjByName(self, f"dspb_jnt{nm}")
            if sb is not None:
                sb.blockSignals(True)
                sb.setValue(value)
                sb.blockSignals(False)
        if skip != "slider":
            sl = getObjByName(self, f"sl_jnt{nm}")
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
        sb = getObjByName(self, f"dspb_jnt{nm}")
        if sb is not None:
            sb.blockSignals(True)
            sb.setRange(low, hi)
            sb.setValue(val)
            sb.blockSignals(False)

        # slider
        sl = getObjByName(self, f"sl_jnt{nm}")
        if sl is not None:
            sl.blockSignals(True)
            sl.setRange(int(low * sl._scale), int(hi * sl._scale))
            sl.setValue(int(val * sl._scale))
            sl.blockSignals(False)

        # increment/decremnt buttons
        jt = joint_type_FC2WB(self.robot.Robot_joints[j_idx].JointType)
        unit = " mm" if jt == PRISMATIC else "°"
        bm = getObjByName(self, f"btn_jnt_m{nm}")
        if bm is not None:
            bm.setToolTip(f"min: {low:g}{unit}")
        bp = getObjByName(self, f"btn_jnt_p{nm}")
        if bp is not None:
            bp.setToolTip(f"max: {hi:g}{unit}")

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
        self._commit()

    def _on_reset_joints(self):
        with self.writing():
            self.ctrl.reset_joints()
        for j_n in range(self.ctrl.j_num):
            self.refresh_row(j_n, 0.0)

    def _on_reload_dirs(self):
        self._commit()
        dirs = joint_dirs(self.robot)
        for j_n in range(self.ctrl.j_num):
            ck = getObjByName(self, f"chk_flip{j_n+1:02d}")
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
        self._sl_pending = (j_idx, raw)
        self._sl_timer.start(0)

    def _apply_slider(self):
        pending, self._sl_pending = self._sl_pending, None
        if pending is None or not is_alive(self.robot):
            return
        j_idx, raw = pending
        sl = getObjByName(self, f"sl_jnt{j_idx + 1:02d}")
        if sl is None:
            return
        new_val = self.ctrl.set_joint_angle_clamped(j_idx, raw / sl._scale)
        self.refresh_row(j_idx, new_val, skip="slider")
        if not sl.isSliderDown():
            self._commit()

    def _on_commit(self, *_):
        self._sl_timer.stop()
        self._apply_slider()
        self._commit()

    def _commit(self):
        # commit the joints on motions
        with self.writing():
            self.ctrl.commit_joints()

    def _on_flip(self, j_idx, checked):
        with self.writing():
            self.ctrl.commit_joints()

        # cfg write -> onChanged invalidates the kin chain
        set_joint_cfg(self.robot, self.robot.Robot_joints[j_idx],
                      dir=-1 if checked else 1)

        self.ctrl.sync_joints_from_doc()
        self.refresh_row_limits(j_idx)

    def _on_step_changed(self, value):
        self.ctrl.j_step = float(value)

    def _on_set_home(self):
        with self.writing():
            self.ctrl.commit_joints()
        save_home(self.robot)

    def _on_set_zero(self):
        self._commit()         # pose -> Offset2 first
        set_zero_pose(self.robot)         # snapshot raw as zero pose
        self.ctrl.sync_joints_from_doc()  # j_vals re-read -> ~0.0
        for j_n in range(self.ctrl.j_num):
            self.refresh_row_limits(j_n)  # re-center sliders/spinboxes

    def _on_go_home(self):
        with self.writing():
            self.ctrl.go_home_pos()
        for idx in range(self.ctrl.j_num):
            self.refresh_row(idx, self.ctrl.j_vals[idx])

    # --------------------------------------------
    #           Robot state interface
    # --------------------------------------------
    def read_joints_data(self, dbg_s=False):
        """Read joint data."""
        # dbg_s = True  # DBG
        p_wid = self.findChild(QGroupBox, "tp_gb0_wd")
        if p_wid is None:
            return
        else:
            # fcl_msg(wid.children())  # DBG
            pass

        for j_n, jnt in enumerate(self.ctrl.robot.Robot_joints):
            if dbg_s:
                fcl_msg(f"{j_n} {jnt.Label}")

            # skip fixed joints in the joint sliders
            if joint_type_FC2WB(jnt.JointType) == FIXED:
                continue

            # shorten the joint names to J<idx> to save UI space
            # set_wid_text(p_wid, f"lbl_jnt{j_n + 1:02d}", QLabel,
            #              f"<b>{jnt.Label}</b>")
            lbl = p_wid.findChild(QLabel, f"lbl_jnt{j_n + 1:02d}")
            if lbl is not None:
                lbl.setText(f"<b>J{j_n}</b>")
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
            sb.editingFinished.connect(self._on_commit)

            sl = getObjByName(p_wid, f"sl_jnt{nm}")
            sl.valueChanged.connect(lambda raw,
                                    idx=j_n: self._on_slider(idx, raw))
            sl.sliderReleased.connect(self._on_commit)

            ck = getObjByName(p_wid, f"chk_flip{nm}")
            ck.setChecked(joint_dirs(self.robot)[j_n] == -1)
            ck.toggled.connect(lambda c, idx=j_n: self._on_flip(idx, c))

    def on_doc_changed(self, obj, prop: str) -> None:
        if self._writing or not is_alive(self.robot):
            return
        if prop == "Offset2" and obj in self.robot.Robot_joints:
            QTimer.singleShot(0, self.sync_panel_from_doc)


class AnimationTaskPanel:
    """
    Single-robot control shell around RobotControlWidget
    """
    def __init__(self, robot_obj) -> None:
        self.form = RobotControlWidget(robot_obj)

    def getStandardButtons(self):
        return QtGui.QDialogButtonBox.Close

    def reject(self) -> bool:
        try:
            if is_alive(self.form.robot):
                with self.form.writing():
                    self.form.ctrl.commit_joints()
        finally:
            Gui.Control.closeDialog()
        return True


def switch_document(doc_name):
    """Switch a FreeCAD document."""
    App.setActiveDocument(doc_name)
    App.ActiveDocument = App.getDocument(doc_name)
    Gui.ActiveDocument = Gui.getDocument(doc_name)


def run(robot=None):
    fnt = QApplication.font("QMessageBox")
    if robot is None:
        sel = Gui.Selection.getSelection()
        if len(sel) != 1 \
                or sel[0].TypeId != "App::FeaturePython" \
                or not is_robot(sel[0]):

            msg_box(Gui.getMainWindow(), "Robot", fnt,
                    "<b>Robot Selection</b>"
                    "<br><br>"
                    "You must selecta 'Robot_FPO' from the tree")
            # fcl_err(f"sel:{str(len(sel))}, typeID:{sel[0].TypeId},
            # name:{sel[0].Name}")
            return

        # current user selected robot fpo
        robot = sel[0]

    if not is_robot(robot):
        msg_box(Gui.getMainWindow(), "Robot", fnt,
                "<b>Robot Missing Properties</b>"
                "<br><br>"
                "You must recreate 'Robot_FPO'")
        return

    if Gui.Control.activeDialog():
        # skip if other dialogs are open
        return

    switch_document(robot.Document.Name)
    Gui.Control.showDialog(AnimationTaskPanel(robot))
