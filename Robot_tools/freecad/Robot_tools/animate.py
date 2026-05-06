"""Robot Animator.

Name: animator.py

See Changelog after import statements.

Author: Carlo Dormeletti
Copyright: 2026
Licence: All right reserved
"""
__version__ = "0.07"
__build__ = "20260428_1248"


import pathlib

import FreeCAD as App
import FreeCADGui as Gui

# Assembly imports
import UtilsAssembly
import JointObject

from PySide import QtGui, QtCore  # noqa  # QtWidgets
from PySide.QtWidgets import (  # noqa
    QApplication, QCheckBox, QFrame, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSpinBox, QTextEdit,  # Widgets
    QDialog, QFileDialog, QInputDialog, QMessageBox,  # Dialogs
    QGridLayout, QVBoxLayout, QSizePolicy)  # Layouts and Policy

from PySide.QtCore import QObject, Qt  # noqa

from freecad.Robot_tools.rbt_constants import ap_clr

from freecad.Robot_tools.rbt_helpers_ui import (
    cm_chb, cm_gbx, cm_btn,
    cm_lbl, cm_ledit, cm_txt,
    getObjByName,set_wid_text, set_wid_en,
    msg_box, get_file, get_dir
)

from freecad.Robot_tools.rbt_helpers_doc import (
    clear_doc, ensure_document, setview, switch_document
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


def create_link_row(dlg, gbx_l, row, fnt, jr, joint_nm):
    """Create a link of buttons for a link."""
    txt_wd = cm_lbl(dlg, f"lbl_j{jr}", joint_nm, fnt, 0)
    gbx_l.addWidget(txt_wd, row, 0, 1, 1)

    lbl_jnt = cm_lbl(dlg, f"lbl_jnt{jr}", "", fnt, 0)
    lbl_jnt.setFrameShape(QFrame.Shape.Panel)
    lbl_jnt.setFrameShadow(QFrame.Shadow.Sunken)
    lbl_jnt.setStyleSheet("QLabel {background-color: palette(base); color: palette(text);}")
    lbl_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    gbx_l.addWidget(lbl_jnt, row, 1, 1, 1)

    txt_jnta = cm_ledit(dlg, f"txt_jnta{jr}", fnt, 0)
    txt_jnta.setReadOnly(True)
    gbx_l.addWidget(txt_jnta, row, 2, 1, 1)

    btn_jnt_m = cm_btn(dlg, f"btn_jnt_m{jr}", "-", fnt, 0)
    btn_jnt_m.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    btn_jnt_m.setMinimumWidth(30)
    gbx_l.addWidget(btn_jnt_m, row, 3, 1, 1)

    btn_jnt_p = cm_btn(dlg, f"btn_jnt_p{jr}", "+", fnt, 0)
    btn_jnt_p.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
    btn_jnt_p.setMinimumWidth(30)
    gbx_l.addWidget(btn_jnt_p, row, 4, 1, 1)

    txt_jnts = cm_ledit(dlg, f"txt_jnts{jr}", fnt, 0)
    txt_jnts.setReadOnly(True)
    gbx_l.addWidget(txt_jnts, row, 5, 1, 1)

    lbl_jdir = cm_lbl(dlg, f"lbl_jdir{jr}", "", fnt, 0)
    lbl_jdir.setFrameShape(QFrame.Shape.Panel)
    lbl_jdir.setFrameShadow(QFrame.Shadow.Sunken)
    lbl_jdir.setStyleSheet("QLabel {background-color: palette(base); color: palette(text);}")
    gbx_l.addWidget(lbl_jdir, row, 6, 1, 1)


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


class O2PDialog(QDialog):
    """Show a dialog for Robot Controller."""
    #
    ui_title = "Robot Controller"
    rob_obj = None
    # Default values for testing, assigned in set_working_robot.
    j_num = 0
    j_dirs = []
    j_nms = []
    j_step = []
    j_vals = []
    doc_obj_nm = ""
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

    def __init__(self, parent=Gui.getMainWindow()):
        """Canonical Init."""
        super().__init__(parent)
        self.initUI()

    def initUI(self):
        """Init UI."""
        global work_dir
        # get dimensions for available space on screen
        av_wid = QtGui.QGuiApplication.primaryScreen().availableGeometry().width()
        av_hei = QtGui.QGuiApplication.primaryScreen().availableGeometry().height()
        w_wid = av_wid * 0.25
        w_hei = av_hei * 0.50
        x_loc = (av_wid - w_wid) * 0.5
        y_loc = (av_hei - w_hei) * 0.5
        # define window xLoc,yLoc,xDim,yDim
        self.setGeometry(x_loc, y_loc, w_wid, w_hei)
        self.setWindowTitle(" ")  # MacOS has no title in some cases
        self.setWindowFlags(
            self.windowFlags() | Qt.WindowStaysOnTopHint
        )

        fnt = QApplication.font("QMessageBox")
        self.fnt = fnt
        self.form_lay = QGridLayout(self)

        mcn = 4  # fake number of column to pre adapt panel width
        row = 0
        self.lbl_sw = cm_lbl(
            self, "lbl_sw",
            f"<b>{pg_name} - {self.ui_title}</b> - Build: {__build__}", self.fnt, 0)
        self.form_lay.addWidget(self.lbl_sw, row, 0, 1, mcn)

        row +=1

        raw_sel = Gui.Selection.getSelection("", 2, True)
        esn = len(raw_sel)

        if esn == 0:
            msg_box(
                self, " ", self.fnt,
                (f"<b>{pg_name}</b> - <b>Robot Selection</b><br><br>"
                 "You must select a Robot FPO"))
            self.close()
            return
        elif esn == 1:
            obj = raw_sel[0]
            obj_typ = obj.TypeId
            # fcl_msg(f"Object Name: {obj.Name[:9]}\n")
            if obj_typ == "App::FeaturePython" and obj.Name[:9] == "Robot_FPO":
                # fcl_msg("Robot OBJ OK\n")
                lbl_rob_t = cm_lbl(
                    self, "lbl_rob_t", "<b>Robot:</b>", self.fnt, 0)
                self.form_lay.addWidget(lbl_rob_t, row, 0, 1, 1)

                lbl_rob_id = cm_lbl(
                    self, "lbl_rob_id", f"{obj.Name}", self.fnt, 0)

                lbl_rob_id.setFrameShape(QFrame.Shape.Panel)
                lbl_rob_id.setFrameShadow(QFrame.Shadow.Sunken)
                lbl_rob_id.setStyleSheet("QLabel {background-color: white;}")
                lbl_rob_id.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
                self.form_lay.addWidget(lbl_rob_id, row, 1, 1, 1)
                row += 1
                self.rob_obj = obj
                self.set_working_rbt()
                tp_gb0 = self.create_joint_ui()
                self.form_lay.addWidget(tp_gb0, row, 0, 1, 4)
                self.read_joints_data()
                switch_document(obj.Document.Name)
                self.set_pose()
                self.set_defaults()
            else:
                msg_box(
                    self, " ", self.fnt,
                    (f"<b>{pg_name}</b> - <b>Robot Selection</b><br><br>"
                     "You must select a Robot FPO"))
                # FIXME: close the dialog?
                # NOTE:  it seems OK to simply return!
                return

        row += 1
        self.form_lay.setRowStretch(row, 1)
        self.show()

    def create_joint_ui(self):
        """Create Joint UI."""
        tp_gb0, tp_gb0l = cm_gbx(self, "tp_gb0", "Control")
        tp_gb0.setStyleSheet(self.grb_ss)

        lbl_1_wd = cm_lbl(self, "lbl_c0", "<b>Joint</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_1_wd, 0, 0, 1, 1)

        lbl_2_wd = cm_lbl(self, "lbl_c1", "<b>Joint name</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_2_wd, 0, 1, 1, 1)

        lbl_3_wd = cm_lbl(self, "lbl_c2", "<b>Angle</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_3_wd, 0, 2, 1, 1)

        lbl_4_wd = cm_lbl(self, "lbl_c3", "<b>Decrease</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_4_wd, 0, 3, 1, 1)

        lbl_4_wd = cm_lbl(self, "lbl_c4", "<b>Increase</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_4_wd, 0, 4, 1, 1)

        lbl_5_wd = cm_lbl(self, "lbl_c5", "<b>Step</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_5_wd, 0, 5, 1, 1)

        lbl_6_wd = cm_lbl(self, "lbl_c6", "<b>Dir</b>", self.fnt, 0)
        tp_gb0l.addWidget(lbl_6_wd, 0, 6, 1, 1)

        brow = 1

        for idx, jnm in enumerate(self.j_nms):
            create_link_row(self, tp_gb0l, brow, self.fnt, f"{idx + 1:02d}", jnm)
            brow += 1

        brow += 1
        # Service buttons: (Reset joints)
        btn_jnts_res = cm_btn(self, "btn_jnts_res", "Reset Joints", self.fnt, 0)
        tp_gb0l.addWidget(btn_jnts_res, brow, 0, 1, 2)

        btn_jnts_res.clicked.connect(self.reset_joints)

        btn_jnts_rld = cm_btn(self, "btn_jnts_rld", "Reload FPO Data", self.fnt, 0)
        btn_jnts_rld.setToolTip("Reload FPO data ('joints directions')")
        tp_gb0l.addWidget(btn_jnts_rld, brow, 2, 1, 2)

        btn_jnts_rld.clicked.connect(self.reload_fpo_data)

        btn_dbg_div = cm_btn(self, "btn_dbg_div", "Dbg IntVar", self.fnt, 0)
        tp_gb0l.addWidget(btn_dbg_div, brow, 4, 1, 2)

        btn_dbg_div.clicked.connect(self.dump_intvar)

        tp_gb0l.setColumnStretch(0, 1)
        tp_gb0l.setColumnStretch(1, 3)
        tp_gb0l.setColumnStretch(2, 1)
        tp_gb0l.setColumnStretch(3, 0)
        tp_gb0l.setColumnStretch(4, 0)
        tp_gb0l.setColumnStretch(5, 1)
        tp_gb0l.setColumnStretch(6, 1)

        tp_gb0.setLayout(tp_gb0l)

        return tp_gb0

    def change_angle(self, dbg_s=False):
        dbg_s = True  # DBG
        call_id = self.sender().objectName()
        # fcl_msg(f"{call_id} button clicked")  # DBG

        wid = self.findChild(QGroupBox, "tp_gb0_wd")
        idx = int(call_id[-2:])
        obj = getObjByName(wid, f"txt_jnta{call_id[-2:]}")
        c_dir = 1 if call_id[-3:-2] == 'p' else -1
        if obj is not None:
            j_idx = idx - 1
            dir = c_dir * self.j_dirs[j_idx]
            step = self.j_step[j_idx]
            incr = dir * step
            if dbg_s:
                fcl_msg(f"j_vals prior: {self.j_vals[j_idx]}\n")
                fcl_msg(f"dir * step: {incr}\n")
            new_val = self.j_vals[j_idx] + incr
            self.j_vals[j_idx] = new_val
            obj.setText(str(new_val))
            if dbg_s:
                fcl_msg(f"j_vals after: {self.j_vals[j_idx]}\n")
            self.setJointAngle(j_idx, new_val, dbg_s)
            #
            if dbg_s:
                msg = (
                    f"----- {call_id} Id: {idx} dir: {c_dir}-----\n"
                    f"- step: {step} dir: {dir}\n"
                    f"- value: {new_val} incr: {incr}\n"
                    f"- j_vals: {self.j_vals}\n"
                )
                fcl_msg(msg)

    def read_joints_data(self, dbg_s=False):
        """Read joint data."""
        # dbg_s = True  # DBG
        p_wid = self.findChild(QGroupBox, "tp_gb0_wd")
        if p_wid is None:
            return
        else:
            # fcl_msg(wid.children())  # DBG
            pass

        for j_n, jnt in enumerate(self.rob_obj.Robot_joints):
            if dbg_s:
                fcl_msg(f"{j_n} {jnt.Label}")
            set_wid_text(p_wid, f"lbl_jnt{j_n + 1:02d}", QLabel, f"<b>{jnt.Label}</b>")
            # Assign to increase angle button the action
            btn_p_nm = f"btn_jnt_p{j_n + 1:02d}"
            btn_p = getObjByName(p_wid, btn_p_nm)
            if btn_p is not None:
                btn_p.clicked.connect(self.change_angle)
            else:
                fcl_err(f"button + for {btn_p_nm} not found!")
            # Assign to decrease angle button the action
            btn_m_nm = f"btn_jnt_m{j_n + 1:02d}"
            btn_m = getObjByName(p_wid, btn_m_nm)
            if btn_m is not None:
                btn_m.clicked.connect(self.change_angle)
            else:
                fcl_err(f"button - for {btn_m_nm} not found!")
            # set the dir label
            set_wid_text(
                p_wid, f"lbl_jdir{j_n + 1:02d}", QLabel, f"<b>{self.j_dirs[j_n]}</b>")

    def reload_fpo_data(self):
        """Reload data from the robot  model FPO."""
        self.reload_joints_dirs()
        #

    def reload_joints_dirs(self):
        """Reload joint dirs and update the control panel."""
        p_wid = self.findChild(QGroupBox, "tp_gb0_wd")
        if p_wid is None:
            return
        self.j_dirs = self.rob_obj.Robot_joints_dir
        #
        for j_n, jnt in enumerate(self.rob_obj.Robot_joints):
            set_wid_text(
                p_wid, f"lbl_jdir{j_n + 1:02d}", QLabel, f"<b>{self.j_dirs[j_n]}</b>")

    def reset_joints(self):
        """Reset joints value to 0"""
        asm_obj = self.rob_obj.Robot_assembly
        wid = self.findChild(QGroupBox, "tp_gb0_wd")
        if asm_obj is None:
            return

        for j_n, jnt in enumerate(self.rob_obj.Robot_joints):
            fl_nm = f"txt_jnta{j_n + 1:02d}"
            jnt_o2 = jnt.Offset2
            fcl_msg(f"{j_n} {jnt.Label}\n")
            fcl_msg(f"OF2:  {jnt_o2}\n")
            jnt.Offset2 = Placement(VEC0, Rotation())
            # FIXME: maybe a dirty hack but it seems to reset the overlay icon.
            jnt.purgeTouched()
            jnt.ViewObject.signalChangeIcon()
            # NOTE: following recompute seems to not work.
            asm_obj.recompute()
            # FIXME: maybe a dirty hack but it seems to reset the overlay icon.
            asm_obj.purgeTouched()
            asm_obj.ViewObject.signalChangeIcon()
            obj = getObjByName(wid, fl_nm)
            fcl_msg(f"Field name: {fl_nm}, obj: {obj}\n")  # DBG
            obj.setText(str(0.0))
            self.j_vals[j_n] = 0.0
        #
        self.dump_intvar("reset_joint")  # DBG

    def set_defaults(self):
        """Set defaults value in the Control group."""
        wid = self.findChild(QGroupBox, "tp_gb0_wd")
        for idx in range(1, self.j_num + 1):
            obj = getObjByName(wid, f"txt_jnta{idx:02d}")
            if obj is not None:
                obj.setText(str(self.j_vals[idx - 1]))
            obj_s = getObjByName(wid, f"txt_jnts{idx:02d}")
            if obj_s is not None:
                obj_s.setText(str(self.j_step[idx - 1]))

    def set_pose(self, dbg_s=False):
        """Set pose to the standard one."""
        # NOTE: this is mostly to reset the pose as actually assembly is not
        #       setting it if you don't alter the Offset2 position see below.
        # dbg_s = True  # DBG
        asm_obj = self.rob_obj.Robot_assembly
        for j_n, jnt in enumerate(self.rob_obj.Robot_joints):
            jnt_o2 = jnt.Offset2
            if dbg_s:
                fcl_msg(f"{j_n} {jnt.Label}\n")
                fcl_msg(f"OF2:  {jnt_o2}\n")
            # NOTE: setting Offset2 adding increment and then without is a trick
            #       maybe in future it will be not needed.
            jnt.Offset2 = Placement(VEC0, Rotation(1, 0, 0)).multiply(jnt_o2)
            asm_obj.recompute()
            jnt.Offset2 = jnt_o2
            asm_obj.recompute()

    def set_working_rbt(self, dbg_s=False):
        """Set working robot."""
        # dbg_s = True
        if self.rob_obj.Robot_assembly is None:
            msg_box(
                self, " ", self.fnt,
                (f"<b>{pg_name}</b> - <b>Set working Robot</b>"
                 "<br><br>You must select a complete Robot Object"))

            return
        else:
            obj = self.rob_obj.Robot_assembly
            setview(obj.Document.Name, 1)
            jnt_n = len(self.rob_obj.Robot_joints)
            jntd_n = len(self.rob_obj.Robot_joints_dir)

            if jnt_n < 1:
                msg_box(
                    self, " ", self.fnt,
                    (f"<b>{pg_name}</b> - <b>Set working Robot</b>"
                     "<br><br>You must select a complete Robot Object"))
                return
            if dbg_s:
                fcl_msg(f"Joint numbers: {jnt_n}\n")
            self.j_num = jnt_n
            self.j_dirs = []
            self.j_nms = []
            self.j_step = []
            self.j_vals = []
            #
            for n in range(jnt_n):
                jnt_n = n + 1
                self.j_nms.append(f"Joint{jnt_n:02d}")
                self.j_vals.append(0.0)
                self.j_step.append(1.0)
        #
        if not self.rob_obj.Robot_joints_dir:
            msg_box(
                self, "", self.fnt,
                (f"<b>{pg_name}</b> - <b>Set working Robot</b>"
                 "<b>WARNING</b><br><br>"
                 f"{self.rob_obj.Name} lacks of a proper <b>Robot_joints_dir</b>"
                 "<br><br>It will be populated with a 'list' "))
            dummy_lst = []
            for n in range(jnt_n):
                dummy_lst.append(1)
            #
            self.rob_obj.Robot_joints_dir = dummy_lst

        if jntd_n > jnt_n:
            msg_box(
                self, "Robot", self.fnt,
                (f"<b>{pg_name}</b> - <b>Set working Robot</b>"
                 "<b>WARNING</b><br><br>"
                 f"{self.rob_obj.Name} <b>Robot_joints_dir</b> was not "
                 "correctly populated.<br><br>There were too much data "
                 "in the 'list', it has been fixed now! <br><br>"
                 "<b>Check rotation sense correctness of each joint!</b>"))
            jntd_l = self.rob_obj.Robot_joints_dir
            self.rob_obj.Robot_joints_dir = jntd_l[:jnt_n]
        elif jntd_n < jnt_n:
            msg_box(
                self, "Robot", self.fnt,
                (f"<b>{pg_name}</b> - <b>Set working Robot</b>"
                 "<b>WARNING</b><br><br>"
                 f"{self.rob_obj.Name} <b>Robot_joints_dir</b> was not "
                 "correctly populated.<br><br>There were too few data "
                 "in the 'list', it has been fixed now! <br><br>"
                 "<b>Check rotation sense correctness of each joint!</b>"))
            jntd_l = self.rob_obj.Robot_joints_dir
            for n in range(jnt_n - jntd_n):
                jntd_l.append(1)
            self.rob_obj.Robot_joints_dir = jntd_l
            #
        self.j_dirs = self.rob_obj.Robot_joints_dir

    def setJointAngle(self, j_idx, value, dbg_s=False):
        """Set joint angle."""
        joint = self.rob_obj.Robot_joints[j_idx]
        if dbg_s:
            fcl_msg(f"{j_idx} - {joint.Label}\n")
            prev_ofs = joint.Offset2
        #
        joint.Offset2 = Placement(VEC0, Rotation(value, 0, 0))
        joint.recompute()
        # FIXME: maybe a dirty hack but it seems to reset the overlay icon.
        joint.purgeTouched()
        joint.ViewObject.signalChangeIcon()
        # not fully functional here
        # self.wk_asm.recompute()
        if dbg_s:
            fcl_msg(f"Offset2: {prev_ofs} >> {joint.Offset2}\n")

    # --------------------------------------------
    #                debug functions
    # --------------------------------------------

    def dump_intvar(self, op_nm):
        """Dump internal variables."""
        msg = (
            f"----- {op_nm} -----\n"
            f"- j num = {self.j_num}\n"
            f"- j_dirs: {self.j_dirs}\n"
            f"- j_nms: {self.j_nms}\n"
            f"- j_step: {self.j_step}\n"
            f"- j_vals: {self.j_vals}\n"
            "--------------------------------\n"
        )
        fcl_msg(msg)


def run():
    dialog = O2PDialog()
    dialog.show()

# Executed if not imported as module


if __name__ == "__main__":
    dialog = O2PDialog()
