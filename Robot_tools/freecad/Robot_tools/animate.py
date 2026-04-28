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

ap_clr = {
    "Black": [(0.0, 0.0, 0.0), "#000000"],
    "Grey75": [(0.25, 0.25, 0.25), "#404040"],
    "White": [(1.0, 1.0, 1.0), "#FFFFFF"],
    # Vibrant
    "V_Blue": [(0.0, 0.467, 0.733), "#0077BB"],
    "V_Cyan": [(0.20, 0.733, 0.933), "#33BBEE"],
    "V_Teal": [(0.0, 0.6, 0.533), "#009988"],
    "V_Orange": [(0.933, 0.467, 0.2), "#EE7733"],
    "V_Red": [(0.8, 0.2, 0.67), "#CC3311"],
    "V_Magenta": [(0.933, 0.2, 0.467), "#EE3377"],
    "V_Grey": [(0.733, 0.733, 0.733), "#BBBBBB"],
    # Bright
    "B_Blue": [(0.267, 0.467, 0.667), "#4477AA"],
    "B_Cyan": [(0.4, 0.8, 0.933), "#66CCEE"],
    "B_Green": [(0.133, 0.533, 0.200), "#228833"],
    "B_Yellow": [(0.8, 0.733, 0.267), "#CCBB44"],
    "B_Red": [(0.933, 0.4, 0.467), "#EE6677"],
    "B_Purple": [(0.667, 0.2, 0.467), "#AA3377"],
    # High Contrast
    "HC_Yellow": [(0.867, 0.667, 0.2), "#DDAA33"],
    "HC_Red": [(0.733, 0.333, 0.4), "#BB5566"],
    "HC_Blue": [(0.0, 0.267, 0.533), "#004488"],
    # Additional
    "Aqua": [(0.0, 1.0, 1.0), "#00FFFF"],
    "Orange": [(1.0, 0.647, 0.0), "#FFA500"],
}


# ------------------------------------------------
#                  General Service
# ------------------------------------------------

def clear_doc(doc_name):
    """Clear the document deleting all the objects.

    Args:
    doc_name  (str): document name
    """
    doc = App.getDocument(doc_name)
    try:
        while len(doc.Objects) > 0:
            doc.removeObject(doc.Objects[0].Name)
    except Exception as e:
        fcl_msg(f"Exception:  {e}\n")


def ensure_document(doc_name, action=0, dbg_l=0):
    """Ensure existence of document with doc_name and clear the doc.

    Args:
        doc_name (string): document name
        action (int): clear action if document exist
              0: do nothing
              1: close and reopen the document with the same name
              2: delete all objects
        dbg_l (int):

    Returns:
        doc_name (string): Document name from obj.Name
    """
    doc_root = App.listDocuments()
    doc_exist = False

    for d_name, doc in doc_root.items():

        if dbg_l > 0:
            fcl_msg(f"ED: doc name = {d_name}\n")

        if d_name == doc_name:
            if dbg_l > 0:
                fcl_msg(f"ED: Match name: {doc_name} action: {action}\n")

            if action == 1:
                # when using clear_doc() is not possible like in Part.Design
                App.closeDocument(doc_name)
                doc_exist = False
            elif action == 2:
                # debug info do not delete
                # fcl_msg("action 2")  # kp
                doc_exist = True
                clear_doc(doc_name)
                App.setActiveDocument(d_name)
                return App.getDocument(d_name).Name
            else:
                doc_exist = True
                App.setActiveDocument(d_name)
                return App.getDocument(d_name).Name

    if doc_exist is False:
        if dbg_l > 0:
            fcl_msg(f"ED: Create = {doc_name}\n")

        new_doc = App.newDocument(doc_name)
        return new_doc.Name


def setview(doc_name, t_v=0):
    """Set viewport in 3D view."""
    App.setActiveDocument("")
    App.ActiveDocument = None
    Gui.ActiveDocument = None

    switch_document(doc_name)
    VIEW = Gui.ActiveDocument.ActiveView

    if t_v == 0:
        VIEW.viewTop()
    elif t_v == 1:
        VIEW.viewFront()
    elif t_v == 99:
        VIEW.setCameraOrientation(
            Rotation(0.0, 0.0, -0.7071067811865475, 0.7071067811865475))

    else:
        VIEW.viewAxometric()

    VIEW.setAxisCross(True)
    VIEW.fitAll()
    # Gui.updateGui()


def set_txt_color(txt, col):
    """Set text color using html syntax."""
    if col == "o":
        f_clr = ap_clr['B_Green'][1]
    elif col == "w":
        f_clr = ap_clr['Orange'][1]
    elif col == "e":
        f_clr = ap_clr['B_Red'][1]
    else:
        f_clr = "black"

    return f"<span style='color: {f_clr}'> {txt} </span>"


def switch_document(doc_name):
    """Switch a FreeCAD document."""
    App.setActiveDocument(doc_name)
    App.ActiveDocument = App.getDocument(doc_name)
    Gui.ActiveDocument = Gui.getDocument(doc_name)
    #  Trick to swith the Gui to show the document
    gv = Gui.ActiveDocument.ActiveView.graphicsView()
    pw = gv.parentWidget().parentWidget().parentWidget()
    Gui.getMainWindow().centralWidget().setActiveSubWindow(pw)


# ------------------------------------------------
#               Service functions
# ------------------------------------------------

def roundvec(r_vec, prec=6):
    """Round value in vectors."""
    vlist = [round(r_vec.x, prec), round(r_vec.y, prec), round(r_vec.z, prec)]
    return vlist


def roundrot(r_rot, prec=6):
    """Round value in vectors."""
    vlist = [round(r_rot[0], prec), round(r_rot[1], prec), round(r_rot[2], prec)]
    return vlist


# ------------------------------------------------
#                   UI functions
# ------------------------------------------------


def cm_chb(parent, o_nm, o_txt, l_fnt, enable=True):
    """Create a styled QLabel."""
    chb = QCheckBox(parent)
    chb.setObjectName(o_nm)
    chb.setText(o_txt)

    if enable is True:
        chb.setEnabled(True)
    else:
        chb.setEnabled(False)

    return chb


def cm_gbx(parent, gb_nm, gb_lbl, gbl_nm="", style="bold", blay=0):
    """Create a groupbox container and return it."""
    gpbx = QGroupBox(gb_lbl)
    gpbx.setObjectName(f"{gb_nm}_wd")

    if style == "bold":
        gpbx_style = (
            "QGroupBox{"
            "    font-weight: bold;"
            "    font-style: normal;"
            "    text-decoration: none;"
            "}")
        gpbx.setStyleSheet(gpbx_style)

    if blay == 0:
        gpbxl = QGridLayout()
    else:
        gpbxl = QVBoxLayout()

    if gbl_nm == "":
        gpbxl.setObjectName(f"{gb_nm}_l")
    else:
        gpbxl.setObjectName(gbl_nm)

    return gpbx, gpbxl


def cm_btn(parent, b_nm, b_txt, b_fnt, b_tfm):
    """Create a styled QPushButton."""
    button = QPushButton()
    button.setObjectName(b_nm)
    button.setFont = b_fnt
    button.setAutoDefault(False)

    if b_tfm == 1:
        button.setTextFormat(Qt.RichText)
    else:
        pass

    button.setText(b_txt)

    if parent is None:
        pass
    else:
        button.setParent(parent)

    button.setAutoDefault(False)

    return button


def cm_lbl(parent, l_nm, l_txt, l_fnt, l_tfm, l_aln=0):
    """Create a styled QLabel.

       l_tfm (int): text format 0: Auto 1: RichText 2: bold
       l_aln (int): alignment 0: Standard  1: Center H and V
    """
    #
    label = QLabel()
    label.setObjectName(l_nm)
    label.setFont(l_fnt)
    label.setWordWrap(False)

    lab_txt = l_txt

    if l_tfm in (1, 2):
        label.setTextFormat(Qt.RichText)
        if l_tfm == 2:
            lab_txt = f"<b>{l_txt}</b>"

    if l_aln == 1:
        label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
    elif l_aln == 2:
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    else:
        pass

    label.setText(lab_txt)

    if parent is None:
        pass
    else:
        label.setParent(parent)

    return label


def cm_ledit(parent, t_nm, t_fnt, t_hei):
    """Create a styled QLineEdit.

       t_hei (int): text field height
    """
    #
    text_f = QLineEdit()
    text_f.setObjectName(t_nm)
    text_f.setFont(t_fnt)

    if t_hei > 0:
        text_f.setFixedHeight(t_hei)

    if parent is None:
        pass
    else:
        text_f.setParent(parent)

    return text_f


def get_dir(parent, fnt, pre_dir=""):
    """Get a directory from a file dialog."""
    f_name = None
    cf_dia = QFileDialog(parent)
    cf_dia.setFileMode(QFileDialog.Directory)
    #
    if pre_dir != "":
        cf_dia.setDirectory(pre_dir)

    if cf_dia.exec():
        f_nms = cf_dia.selectedFiles()
        if len(f_nms) != 1:
            msg_box(
                parent, "You must select only one file.", fnt,
                f"{pg_name}", 'WARNING:', "w")
            f_name = None
        else:
            f_name = f_nms[0]

    return f_name


def get_file(parent, fnt, ftype="fcstd", pre_dir=""):
    """Get a file from a file dialog."""
    f_name = ""
    cf_dia = QFileDialog(parent)

    if ftype == "wrl":
        cf_dia.setNameFilter("wrl (*.wrl *.WRL)")
    elif ftype == "csv":
        cf_dia.setNameFilter("csv (*.csv *.CSV)")
    elif ftype == "def":
        cf_dia.setNameFilter("json (*_def.json  *_DEF.JSON)")
    else:
        # cf_dia.setNameFilter("")
        # no namefilter
        pass

    cf_dia.setFileMode(QFileDialog.ExistingFile)
    cf_dia.setViewMode(QFileDialog.Detail)  # or List
    if pre_dir != "":
        cf_dia.setDirectory(pre_dir)

    if cf_dia.exec():
        f_nms = cf_dia.selectedFiles()
        if len(f_nms) != 1:
            msg_box(
                parent, "You must select only one file.", fnt,
                f"{pg_name}", 'WARNING:', "w")
        else:
            f_name = f_nms[0]

    return f_name


def msg_box(parent, title, fnt, msg, icon="", head="", hc=""):
    """Create a minimal message box."""
    msg_box = QMessageBox(parent)
    msg_box.setFont(fnt)
    msg_box.setWindowTitle(title)

    if icon == "e":
        m_icon = QMessageBox.Icon.Critical
    elif icon == "w":
        m_icon = QMessageBox.Icon.Warning
    elif icon == "i":
        m_icon = QMessageBox.Icon.Information
    else:
        m_icon = QMessageBox.Icon.NoIcon

    msg_box.setIcon(m_icon)
    if head != "":
        s_msg = f"{set_txt_color(head, hc)}<br><br>" + msg
    else:
        s_msg = msg

    msg_box.setText(s_msg)

    msg_box.exec()


def getObjByName(parent, btn_nm):
    """Return a button from its name."""
    wid = parent.findChild(QObject, btn_nm)
    if wid is None:
        return None
    else:
        return wid


def set_wid_text(parent, obj_nm, obj_type, txt):
    """Find an object and set it text."""
    # fcl_msg(parent.children())  # DBG
    wid = parent.findChild(obj_type, obj_nm)
    if wid is None:
        return
    else:
        wid.setText(txt)

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
    lbl_jnt.setStyleSheet("QLabel {background-color: white;}")
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
    lbl_jdir.setStyleSheet("QLabel {background-color: white;}")
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
