"""Robot Test.

Name: study_asm.py

See Changelog below.

Author: Carlo Dormeletti
Copyright: 2026
Licence: All right reserved
"""
__version__ = "0.07"
__build__ = "20260422_1307"


import sys
if sys.version_info.major >= 3:
    from importlib import reload

import pathlib

from datetime import datetime

import FreeCAD as App
import FreeCADGui as Gui
import Part

# Assembly imports
import UtilsAssembly
import JointObject

# Coin3d import
from pivy import coin

import PySide

from PySide import QtGui, QtCore  # noqa  # QtWidgets
from PySide.QtWidgets import (  # noqa
    QApplication, QCheckBox,  QFrame, QGroupBox, QLabel, QLineEdit, QPlainTextEdit,
    QPushButton, QSpinBox, QTabWidget, QTextEdit, QWidget,  # Widgets
    QDialog, QFileDialog, QInputDialog, QMessageBox,  # Dialogs
    QGridLayout, QVBoxLayout, QSizePolicy)  # Layouts and Policy

from PySide.QtCore import QObject, Qt  # noqa

from freecad.Robot_tools.rbt_objects import Robot_obj, ViewProviderRBo

"""
----------------------------------------
Changelog:
----------------------------------------
v0.01 - Initial version.
v0.02 - Added create_asm code.
v0.03 - Added add joint action (unfinished).
v0.04 - Added some refinement in UI and various.
v0.05 - Added actions needed when loading an asm doc:
         - check the asm document for "big errors" like the absence of Robot_FPO
           and Robot_Assembly object.
         - check for empty joints and signal it with a messagebox.
v0.06 - added code to add joints (grounded).
v0.07 - improved check_asm
"""

fcl_err = App.Console.PrintError
fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning

mdtsp = "%y%m%d_%H%M"
MODULE_PATH = pathlib.Path(__file__).parent

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

msg_log = []  # log message container, used to permit a dump.


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
                # print("action 2")  # kp
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
#                Assembly functions
# ------------------------------------------------


def add_asm_object(obj_doc, asm, feat_nm, link_nm, glbl):
    item = asm.newObject("App::Link", link_nm)
    item.LinkedObject = obj_doc.getObject(feat_nm)
    item.Label = glbl
    item.recompute()
    asm.recompute()
    return item


def add_grounded(asm, obj):
    """Add the grounded object."""
    joint_group = UtilsAssembly.getJointGroup(asm)
    ground = joint_group.newObject("App::FeaturePython", "GroundedJoint")
    JointObject.GroundedJoint(ground, obj)
    JointObject.ViewProviderGroundedJoint(ground.ViewObject)
    asm.recompute()


def add_revolute(asm, j_lbl, plt1, plr1, plt2, plr2, ref1, ref2,
                 ofs1=Placement(VEC0, ROT0), ofs2=Placement(VEC0, ROT0)):
    """Add the grounded object."""
    joint_group = UtilsAssembly.getJointGroup(asm)
    rev_j = joint_group.newObject("App::FeaturePython", "RevoluteJoint")
    JointObject.Joint(rev_j, 1)
    JointObject.ViewProviderJoint(rev_j.ViewObject)
    rev_j.recompute()
    rev_j.Detach1 = False
    rev_j.Detach2 = False
    rev_j.EnableAngleMax = False
    rev_j.EnableAngleMin = False
    rev_j.EnableLengthMax = False
    rev_j.EnableLengthMin = False
    rev_j.JointType = "Revolute"
    rev_j.Label = j_lbl
    rev_j.Label2 = ""
    rev_j.Offset1 = ofs1
    rev_j.Offset2 = ofs2
    rev_j.Placement1 = Placement(plt1, plr1)
    rev_j.Placement2 = Placement(plt2, plr2)
    rev_j.Reference1 = ref1
    rev_j.Reference2 = ref2
    rev_j.Suppressed = False
    rev_j.Visibility = False
    rev_j.recompute()
    return rev_j


def create_assembly(doc):
    asm = doc.addObject("Assembly::AssemblyObject", "Assembly")
    asm.Label = "Robot_Assembly"
    asm.Type = "Assembly"
    asm.newObject("Assembly::JointGroup", "Joints")
    asm.recompute()
    return asm


# ------------------------------------------------
#            Assembly helpers functions
# ------------------------------------------------


def set_pose(asm_obj):
    """Set pose to the standard one."""
    for j_n, jnt in enumerate(asm_obj.Joints):
        jnt_o2 = jnt.Offset2
        fcl_msg(f"{j_n} {jnt.Label}\n")
        fcl_msg(f"OF2:  {jnt_o2}\n")
        jnt.Offset2 = Placement(VEC0, Rotation(1, 0, 0)).multiply(jnt_o2)
        asm_obj.recompute()
        jnt.Offset2 = jnt_o2
        asm_obj.recompute()


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


def cm_txt(parent, t_nm, t_fnt, t_hei, rich=True):
    """Create a styled QTextEdit or QPlainTextEdit.

    Args:
        parent(QObject): parent widget
        t_nm (str): text name
        t_fnt (QFont): text font
        t_hei (int): text field height

    Returns:
        text (QtWidget): text
    """
    #
    if rich is False:
        text_f = QPlainTextEdit()
    else:
        text_f = QTextEdit()

    text_f.setObjectName(t_nm)
    text_f.setFont(t_fnt)
    text_f.setReadOnly(True)

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
                parent, "You must select only one file.", fnt, "w", 'WARNING:', "w")
            f_name = None
        else:
            f_name = f_nms[0]

    return f_name


def get_file(parent, fnt, ftype="fcstd", pre_dir=""):
    """Get a file from a file dialog."""
    f_name = ""
    cf_dia = QFileDialog(parent)

    if ftype == "fcstd":
        cf_dia.setNameFilter("FreeCAD Files (*.fcstd *.FCSTD)")
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
                parent, "You must select only one file.", fnt, "w", 'WARNING:', "w")
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


def set_wd_prop(wid, prop, val):
    """Set Qt Widget property."""
    wid.setProperty(prop, val)
    wid.style().unpolish(wid)
    wid.style().polish(wid)


def set_wid_text(parent, obj_nm, txt):
    """Find an object and set it text."""
    # fcl_msg(parent.children())  # DBG
    wid = parent.findChild(QLineEdit, obj_nm)
    if wid is None:
        return
    else:
        wid.setText(txt)


def set_wid_en(parent, obj_nm, state=True):
    """Find an object by name or a tuple of names and set enabled state."""
    # fcl_msg(parent.children())  # DBG
    if isinstance(obj_nm, tuple):
        obj_tpl = obj_nm
    else:
        obj_tpl = (obj_nm, )

    for elem in obj_tpl:
        wid = parent.findChild(QObject, elem)
        if wid is None:
            pass
        else:
            wid.setEnabled(state)

# ------------------------------------------------
#                 Module functions
# ------------------------------------------------


def dump_log(log_fn):
    """Dump log to a file."""
    fcl_msg(f"Log file name: {log_fn}")


# ------------------------------------------------
#                    Main Dialog
# ------------------------------------------------


class O2PDialog(QDialog):
    """Show a dialog for RobotAnimator."""

    ui_title = "Define Robot"
    IsInit = False  # flag to avoid unwanted action execution during init.
    # Flags dictionary, place here sane defaults
    flags = {"chb_cpa": False, "chb_gdj": True}
    #
    tab_data = {
        "ma": {"nm": "Main", "cn": 0}, "lg": {"nm": "Log", "cn": 0}}
    #
    wk_mod = None  # Working model
    wk_asm = None  # Robot model as Assembly DO
    wk_asm_d = None  # Robot Assembly Document
    rob_obj = None  # Robot FPO
    # Default values for testing
    s_doc = None  # Assembly service doc
    eus = None   #
    fnt = None  # Font for widgets
    lb_em = 0  #
    # Rotation axis vis data
    as_rashl = 100  # cyl len used on Assembly Service doc
    as_rarad = 3  # cyl rad used on Assembly Service doc
    as_raclr = ap_clr["Orange"][0]  # used for CRC on Assembly Service doc
    rad_rashl = 1000  # cyl len used on Robot Assembly doc
    rad_rarad = 1.5  # cyl rad used on Robot Assembly doc
    rad_raclr = ap_clr["Orange"][0]  # color used for CRC Robot ASM doc
    rad_raoclr = ap_clr["B_Purple"][0]  # color used for JCS on ASM doc
    #
    rbm_dt = {'rb_jnts': 0, }  # Robot model data  dictionary
    # --- Joint sequence control ---
    jnt_ec = 0  # joint edit counter, indicate what joint we are editing.
    jnt_fc = 1  # joint face counter it will be reset by add_jnt_asm code
    jnt_meta = {}  # joint metadata it will be reset by add_jnt_asm code
    #
    # Stylesheets
    grb_ss = (
        ""
        "QGroupBox{"
        "    font-weight: bold;"
        "    font-style: normal;"
        "    text-decoration: none;"
        "}"
    )

    lab_ss = (
        ""
        "QLabel {"
        "    font-weight: normal;"
        "    font-style: normal;"
        "    text-decoration: none;"
        "    border: 0px none;"
        "}"
        ""
        "QLabel[role='tool'] {"
        "    font-weight: bold;"
        "    font-style: normal;"
        "    text-decoration: none;"
        "    border: 0px none;"
        "}"
        ""
        "QLabel[role='done']{"
        "    font-weight: bold;"
        "    font-style: normal;"
        "    text-decoration: none;"
        "    border: 2px outset black;"
        "    border-radius: 5px;"
        "    padding-left: 10px;"
        "}"
        ""
        "QLabel[role='edit'] {"
        "    font-weight: bold;"
        "    font-style: normal;"
        "    text-decoration: none;"
        "    border: 2px outset gray;"
        "    border-radius: 5px;"
        "    padding: 5px;"
        "}"
        "QPushButton[role='step'] {"
        # " text-align: center;"
        # " margin: 2px;"
        " padding: 1px;"
        # " border: 1px solid #111111;"
        # " border-radius: 6px;"
        "}"
        ""
    )

    log_ss = (
        "QTextEdit {"
        "        font-family: monospace;"
        "}"
    )

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
        w_wid = av_wid * 0.20
        w_hei = av_hei * 0.33
        x_loc = (av_wid - w_wid) * 0.5
        y_loc = (av_hei - w_hei) * 0.5
        # define window xLoc,yLoc,xDim,yDim
        self.setGeometry(x_loc, y_loc, w_wid, w_hei)
        self.setWindowTitle(self.ui_title)
        self.setWindowFlags(
            self.windowFlags() | Qt.WindowStaysOnTopHint
        )

        self.fnt = QApplication.font("QMessageBox")
        lb_fm = QtGui.QFontMetricsF(self.fnt)
        self.lb_em = round(lb_fm.maxWidth())

        # Determine if we are running as a module of Robot Tools WB/TB
        if str(MODULE_PATH.stem) == 'Robot_tools':
            self.isWBcomp = True
        else:
            self.isWBcomp = False

        self.isInit = True
        self.tabw = QTabWidget(self)

        for tb_k in self.tab_data.keys():
            t_dat = self.tab_data[tb_k]
            tab = QWidget()
            tab.setFont = self.fnt
            tab.setObjectName(f"t_{tb_k}")
            lay = QGridLayout(tab)
            lay.setObjectName(f"l_{tb_k}")
            tab.setLayout(lay)
            self.tabw.addTab(tab, t_dat['nm'])

        self.dmw_lay = QGridLayout()

        mcn = 4  # fake number of column to pre adapt panel width
        row = 0
        self.lbl_sw = cm_lbl(self, "lbl_sw", f"Build: {__build__}", self.fnt, 2)
        self.dmw_lay.addWidget(self.lbl_sw, row, 0, 1, mcn)

        row += 1
        self.dmw_lay.addWidget(self.tabw, row, 0, 1, mcn)
        self.setLayout(self.dmw_lay)

        res = self.ma_setui()
        if res is False:
            return

        self.lg_setui()

        self.show()
        self.isInit = False

    def ma_setui(self):
        """Populate main tab."""
        ma_lay = self.tabw.findChild(QObject, "l_ma")
        # ma_tab = self.tabw.findChild(QObject, "t_ma")

        fcl_msg(f"isWBcomp {self.isWBcomp}\n ")  # DBG

        row = 0

        # Robot Files groupBox
        gb_rc, gb_rcl = cm_gbx(self, "gb_rc", "Robot Files")
        gb_rc.setStyleSheet(self.grb_ss)
        gb_rc.setLayout(gb_rcl)

        brow = 0

        self.flags['rob_fpo'] = True

        # Robot Components file
        self.txt_rbnm = cm_ledit(self, "txt_rbnm", self.fnt, 0)
        gb_rcl.addWidget(self.txt_rbnm, brow, 0, 1, 4)
        #
        brow += 1
        self.btn_sel_rbf = cm_btn(
            self, "btn_selrbf", "Select 'Component' file", self.fnt, 0)
        self.btn_sel_rbf.setToolTip(
            "Select the FCStd file that contains the robot components.")

        gb_rcl.addWidget(self.btn_sel_rbf, brow, 0, 1, 2)

        self.btn_sel_rbf.clicked.connect(self.set_working_model)

        # Assembly file
        brow += 1
        self.txt_asnm = cm_ledit(self, "txt_asnm", self.fnt, 0)
        gb_rcl.addWidget(self.txt_asnm, brow, 0, 1, 4)
        #
        brow += 1
        self.btn_sel_asm = cm_btn(
            self, "btn_selasm", "Select 'Assembly' file", self.fnt, 0)
        self.btn_sel_asm.setToolTip(
            "Select the FCStd file that contains Robot_Assembly and the Robot_FPO.")

        gb_rcl.addWidget(self.btn_sel_asm, brow, 0, 1, 2)

        self.btn_sel_asm.clicked.connect(self.set_working_asm)

        ma_lay.addWidget(gb_rc, row, 0, 1, 4)

        row += 1

        gb_bt, gb_btl = cm_gbx(self, "gb_bt", "Functions")
        gb_bt.setStyleSheet(self.grb_ss)
        gb_bt.setLayout(gb_btl)

        brow = 0

        btn_cre_asm = cm_btn(self, "btn_cre_asm", "Create ASM", self.fnt, 0)
        btn_cre_asm.setEnabled(False)
        gb_btl.addWidget(btn_cre_asm, brow, 0, 1, 2)

        btn_cre_asm.clicked.connect(self.create_asm)

        brow += 1
        btn_load_step = cm_btn(self, "btn_load_step", "Load STEP", self.fnt, 0)
        btn_load_step.setEnabled(False)
        gb_btl.addWidget(btn_load_step, brow, 0, 1, 2)

        btn_load_step.clicked.connect(self.act_load_step)

        # brow += 1
        # btn_add_jnt = cm_btn(self, "btn_add_jnt", "Add Joints", self.fnt, 0)
        # btn_add_jnt.setEnabled(False)
        # gb_btl.addWidget(btn_add_jnt, brow, 0, 1, 2)

        # btn_add_jnt.clicked.connect(self.act_add_joint)

        # ma_lay.addWidget(gb_bt, row, 0, 1, 4)

        row += 1

        tp_gb0, tp_gb0l = cm_gbx(self, "tp_gb0", "")
        tp_gb0.setStyleSheet(self.grb_ss)
        # Empty groupBox
        tp_gb0.setLayout(tp_gb0l)
        ma_lay.addWidget(tp_gb0, row, 0, 1, 4)

        row += 1
        jn_gb0, jn_gb0l = cm_gbx(self, "jn_gb0", "Joints")
        jn_gb0.setStyleSheet(self.grb_ss)

        lbl_1_wd = cm_lbl(self, "lbl_jnspc0", "<b>Joint</b>", self.fnt, 0)
        jn_gb0l.addWidget(lbl_1_wd, 0, 0, 1, 1)

        lbl_2_wd = cm_lbl(self, "lbl_jnspc1", "<b>Type</b>", self.fnt, 0)
        jn_gb0l.addWidget(lbl_2_wd, 0, 1, 1, 1)

        lbl_3_wd = cm_lbl(self, "lbl_jnspc2", "<b>Face1</b>", self.fnt, 0)
        jn_gb0l.addWidget(lbl_3_wd, 0, 2, 1, 1)

        lbl_4_wd = cm_lbl(self, "lbl_jnspc3", "<b>Face2</b>", self.fnt, 0)
        jn_gb0l.addWidget(lbl_4_wd, 0, 3, 1, 1)

        jn_gb0.setLayout(jn_gb0l)
        ma_lay.addWidget(jn_gb0, row, 0, 1, 4)

        row += 1
        ma_lay.setRowStretch(row, 1)
        return True

    def jnt_setui(self):
        """Create the add joint UI."""
        wid = self.findChild(QObject, "tp_gb0_wd")
        wid.setTitle("Set Joints")
        lay = self.findChild(QObject, "tp_gb0_l")
        row = 0
        js_gb0, js_gb0l = cm_gbx(self, "js_gb0", "Add Joint")
        js_gb0.setStyleSheet(self.grb_ss)
        # Empty groupBox
        js_gb0.setLayout(js_gb0l)
        brow = 0
        btn_jnt_s1 = cm_btn(self, "btn_jnt_s1", "Select Face1", self.fnt, 0)
        btn_jnt_s1.setEnabled(True)
        js_gb0l.addWidget(btn_jnt_s1, brow, 0, 1, 2)

        btn_jnt_s1.clicked.connect(self.select_face)

        btn_jnt_s2 = cm_btn(self, "btn_jnt_s2", "Select Face2", self.fnt, 0)
        btn_jnt_s2.setEnabled(False)
        js_gb0l.addWidget(btn_jnt_s2, brow, 2, 1, 2)

        btn_jnt_s2.clicked.connect(self.select_face)

        brow += 1
        chb_gdj = cm_chb(self, "chb_gdj", "Grounded Joint", self.fnt, True)
        js_gb0l.addWidget(chb_gdj, brow, 0, 1, 2)
        # Check the flag to determine the state
        if self.flags['chb_gdj'] is True:
            chb_gdj.setChecked(True)
        else:
            chb_gdj.setChecked(False)
            chb_gdj.setEnabled(False)

        brow += 1
        btn_jnt_add = cm_btn(self, "btn_jnt_add", "Add Joint", self.fnt, 0)
        btn_jnt_add.setEnabled(False)
        js_gb0l.addWidget(btn_jnt_add, brow, 0, 1, 2)

        btn_jnt_add.clicked.connect(self.add_jnt2asm)

        lay.addWidget(js_gb0, row, 0, 1, 4)

    def lg_setui(self):
        """Populate Log Tab."""
        lg_lay = self.tabw.findChild(QObject, "l_lg")
        lg_tab = self.tabw.findChild(QObject, "t_lg")
        self.log_win = cm_txt(lg_tab, "log_win", self.fnt, -1, True)
        self.log_win.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.log_win.setStyleSheet(self.log_ss)

        lg_lay.addWidget(self.log_win, 0, 0)

        btn_log = cm_btn(
            lg_tab, "lg_btnlog", 'Dump Log to File', self.fnt, 0)
        btn_log.clicked.connect(self.act_dump_log)
        btn_log.setToolTip("Not implementd now")
        btn_log.setEnabled(False)

        lg_lay.addWidget(btn_log, 1, 0)

        self.log_win.show()

    def closeEvent(self, event):
        """Called on Close event."""
        Gui.Selection.removeSelectionGate()

    def get_chb_content(self, par_nm, obj_nm):
        """Get QCheckBox Widget state."""
        gb_obj = self.findChild(QObject, par_nm)
        if gb_obj is not None:
            obj = gb_obj.findChild(QCheckBox, obj_nm)
            if obj is not None:
                return obj.isChecked()
            else:
                return None
        else:
            return None

    def add_joint2ui(self, jpr, j_data):
        """Add a joint to the panel.

        Parameters:
          jpr (int):  row number
          j_data (list): joint column data (see in the code)

        """
        wid = self.findChild(QObject, "jn_gb0_wd")
        lay = self.findChild(QObject, "jn_gb0_l")
        #
        # Data: [number, type, face1, face2]
        lbl_jnt = cm_lbl(self, f"lbl_jnt{jpr}", str(j_data[0]), self.fnt, 0)
        lbl_jnt.setFrameShape(QFrame.Shape.Panel)
        lbl_jnt.setFrameShadow(QFrame.Shadow.Sunken)
        lbl_jnt.setStyleSheet("QLabel {background-color: white;}")
        lbl_jnt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
        lay.addWidget(lbl_jnt, jpr, 0, 1, 1)

        lbl_jt = cm_lbl(self, f"lbl_jt{jpr}", str(j_data[1]), self.fnt, 0)
        lbl_jt.setFrameShape(QFrame.Shape.Panel)
        lbl_jt.setFrameShadow(QFrame.Shadow.Sunken)
        lbl_jt.setStyleSheet("QLabel {background-color: white;}")
        lbl_jt.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
        lay.addWidget(lbl_jt, jpr, 1, 1, 1)

        lbl_jf1 = cm_lbl(self, f"lbl_jf1{jpr}", str(j_data[2]), self.fnt, 0)
        lbl_jf1.setFrameShape(QFrame.Shape.Panel)
        lbl_jf1.setFrameShadow(QFrame.Shadow.Sunken)
        lbl_jf1.setStyleSheet("QLabel {background-color: white;}")
        lbl_jf1.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
        lay.addWidget(lbl_jf1, jpr, 2, 1, 1)

        lbl_jf2 = cm_lbl(self, f"lbl_jf2{jpr}", str(j_data[3]), self.fnt, 0)
        lbl_jf2.setFrameShape(QFrame.Shape.Panel)
        lbl_jf2.setFrameShadow(QFrame.Shadow.Sunken)
        lbl_jf2.setStyleSheet("QLabel {background-color: white;}")
        lbl_jf2.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
        lay.addWidget(lbl_jf2, jpr, 3, 1, 1)

    # --------------------------------------------
    #                Model functions
    # --------------------------------------------

    def set_working_model(self):
        """Set robot working model."""
        f_name = get_file(self, self.fnt, ftype="fcstd", pre_dir="")
        # fcl_msg(f"Set working model: {f_name}\n")
        if f_name != "":
            o_doc = App.openDocument(f_name)
            fcl_msg(f"open doc: {o_doc}\n")
            self.wk_mod = o_doc
            wid = getObjByName(self, "txt_rbnm")
            if wid is not None:
                wid.setText(str(f_name))
                set_wid_en(self, ("btn_cre_asm",), True)

    def set_working_asm(self):
        """Set robot assembly file."""
        f_name = get_file(self, self.fnt, ftype="fcstd", pre_dir="")
        # fcl_msg(f"Set working model: {f_name}\n")
        if f_name != "":
            o_doc = App.openDocument(f_name)
            fcl_msg(f"open doc: {o_doc}\n")
            self.wk_asm_d = o_doc
            wid = getObjByName(self, "txt_asnm")
            if wid is not None:
                wid.setText(str(f_name))
            else:
                return
            #
            self.check_asm()
            setview(o_doc.Name)
            self.jnt_setui()

    # --------------------------------------------
    #                Assembly functions
    # --------------------------------------------

    def create_asm(self):
        """Add Joint to assembly."""
        # dbg_s = False
        dbg_s = True

        # Step1: ask for a name?
        asm_doc_name = "Asm_doc_test"

        # Step2: create the ASM doc (and save it)
        doc_name = self.wk_mod.FileName
        if dbg_s:
            fcl_msg(f"Robot file folder: {doc_name}\n")
        #
        file_dir = pathlib.Path(doc_name).parent
        if dbg_s:
            fcl_msg(f"Assembly file folder: {file_dir}\n")
        #
        doc_rname = ensure_document(asm_doc_name, 1)
        asm_doc = App.getDocument(doc_rname)
        gui_doc = Gui.getDocument(doc_rname)

        asm_doc.saveAs(str(pathlib.Path(file_dir, doc_rname)))

        asm = create_assembly(asm_doc)
        gui_doc.ActiveView.setActiveObject('assembly', asm)

        asm_doc.recompute()

        # Step3: create the FPO in the ASM doc

        rb_obj = asm_doc.addObject(
            "App::FeaturePython", "Robot_FPO")

        Robot_obj(rb_obj)
        # if App.GuiUp:
        ViewProviderRBo(rb_obj.ViewObject)

        rb_obj.Robot_assembly = asm

        rb_obj.recompute()
        asm_doc.recompute()
        # Disable "Create ASM" button
        set_wid_en(self, ("btn_cre_asm",), False)
        # Create the "Add joint" groupbox
        self.jnt_setui()

    # def act_add_joint(self):
    #     """Add Joint panel assembly."""
    #     self.jnt_setui()
    #     return

    def act_load_step(self):
        """Load Step file."""
        sf = self.rob_obj.STEPFile
        # fcl_msg(f"STEPFile: {sf}\n")
        App.openDocument(sf)

    def check_asm(self, dbg_s=False):
        """Check the correctness of the asm file."""
        dbg_s = True  # DBG
        asm_objs = self.wk_asm_d.getObjectsByLabel("Robot_Assembly")
        ran = len(asm_objs)
        if ran > 0:
            pass
        else:
            msg_box(
                self, "Robot_tools", self.fnt,
                ("<b>There are no Robot Assembly definitions in the document</b>"
                 "<br><br> You must select a correct document or create one"))
            # FIXME: See if the activation of Create ASM button is correct here.
            set_wid_en(self, ("btn_cre_asm",), True)
            return

        if ran == 1:
            self.wk_asm = asm_objs[0]
        else:
            msg_box(
                self, "Robot_tools", self.fnt,
                (f"<b>There are: [{ran}] Robot Assembly objects "
                 "in the document</b><br><br>"
                 "You must select one"))
            # TODO: add the corresponding selection dialog.
            return

        rob_objs = self.wk_asm_d.getObjectsByLabel("Robot_FPO")
        ron = len(rob_objs)
        if ron > 0:
            if ron == 1:
                self.rob_obj = rob_objs[0]
            else:
                msg_box(
                    self, "Robot_tools", self.fnt,
                    (f"<b>There are: [{ron}] FPO in the assembly</b><br><br>"
                     "You must select the working one"))
                # TODO: create the selection dialog
                return
        else:
            msg_box(
                self, "Robot_tools", self.fnt,
                ("<b>There are no Robot FPO in the assembly</b><br><br>"
                 "You must select a valid Assembly Robot document"))
            return
        #
        # NOTE: check for an existing object name "GroundedJoint",
        #       it seems the only way to determine if a grounded joint exist.
        grd_objf = False
        grd_obj = self.wk_asm.getObject("GroundedJoint")

        if grd_obj is not None:
            if dbg_s:
                fcl_msg("Grounded joint present\n")  # DBG
            grd_objf = True
        else:
            if dbg_s:
                fcl_msg("No grounded joint!\n")  # DBG
            pass
        #
        ojl = len(self.wk_asm.Joints)

        if dbg_s:
            fcl_msg(f">> ojl: {ojl}\n")

        if ojl == 0:
            msg_box(
                self, "Robot_tools", self.fnt,
                ("<b>There are no Joints in the assembly</b><br><br>"
                 "You must assign them"))

        # deactivate appropriate buttons
        # TODO: we could deactivate also the "Select Asembly File" button too?
        set_wid_en(self, ("btn_cre_asm", "btn_selrbf"), False)

        if grd_objf is True:
            self.jnt_ec = 1
            # NOTE: use of the flag as at this point the ui is not yet created.
            self.flags["chb_gdj"] = False
            self.add_joint2ui(self.jnt_ec + 1, [0, "grounded", "--", "--"])
 
        # Check if assembly object has only a Joint group on it
        # checking Group property length > 1 will tell if there are other
        # objects
        # FIXME: it is rough guess see if there are alternatives.
        if len(self.wk_asm.Group) > 1:
            pass
            # set_wid_en(self, ("btn_add_jnt",), True)  # 
            # NOTE: no need to activate the "Load STEP" button as the Document
            # with robot components will be opened by FreeCAD for us.
        else:
            set_wid_en(self, ("btn_load_step",), True)

        if ojl > 0:
            print(f"obj: {ojl}")

        # TODO: In case of already present joints, we must:
        # 1) alter here jnt_ec value
        # 2) populate the already defined joints
        # 3) if grounded joint is already defined, we must disbale the checkbox?

        return

    # --------------------------------------------
    #                   Add joint
    # --------------------------------------------

    def add_jnt2asm(self, dbg_s=False):
        """Add a joint to the assembly and the FPO."""
        dbg_s = True  # BDG
        if dbg_s:
            fcl_msg(f"Joint meta: {self.jnt_meta}\n")

        # NOTE: correct sequence:
        # 1) validate the sequence
        # 2) reset the face counter to 1
        # 3) reset the jnt_meta dic to empty
        # 4) advance jnt_ec by 1
        # 5) set buttons Select Face1 and Select Face2 state
        dks = list(self.jnt_meta)
        jn = int(dks[0][5:7])
        jf1 = int(dks[0][7:9])
        jmo0 = self.jnt_meta[dks[0]]
        jnt_fc1_onm = jmo0['ob_nm']
        jnt_fc1_obj = self.wk_asm.getObject(jnt_fc1_onm)

        if dbg_s:
            fcl_msg(f"Joint number {jn} Face {jf1} jid: {jmo0}\n")
            fcl_msg(f"-- f1 obj name: {jnt_fc1_onm}\n")
            fcl_msg(f"-- f1 obj Label: {jnt_fc1_obj.Label}\n")
            fcl_msg(f"-- f1 obj Label2: {jnt_fc1_obj.Label2}\n")

        if len(dks) == 1:
            # one face selected, so the joint is probably the grounded one.
            if jn == 0:
                # This is probably a grounded joint as it is logically the first
                # to be set.
                add_grounded(self.wk_asm, jnt_fc1_obj)
                self.wk_asm.recompute()
                # Reset the interface
                set_wid_en(self, ("btn_jnt_s1",), True)
                set_wid_en(self, ("chb_gdj",), False)
                # TODO: populate the joint frame with a joint type.
                self.add_joint2ui(jn + 2, [0, "grounded", "--", "--"])
                # advance joint counter and reset face counter and data dict
                self.jnt_ec += 1
                self.jnt_fc = 1
                self.jnt_meta = {}
                return
            else:
                # Emit an error as two faces are needed
                return

        # At this point we could be sure that there are two objects in the list
        jf2 = int(dks[1][7:9])
        jmo1 = self.jnt_meta[dks[1]]
        jnt_fc2_onm = jmo1['ob_nm']
        jnt_fc2_obj = self.wk_asm.getObject(jnt_fc2_onm)
        #
        if dbg_s:
            fcl_msg(f"Joint number {jn} Face {jf2} jmo1: {jmo1}\n")
            fcl_msg(f"-- f1 obj Name: {jnt_fc2_onm}\n")
        #
        if jnt_fc2_obj is not None:
            if dbg_s:
                fcl_msg(f"-- f2 obj Label: {jnt_fc2_obj.Label}\n")
                fcl_msg(f"-- f2 obj Label2: {jnt_fc2_obj.Label2}\n")

        else:
            if dbg_s:
                fcl_msg(f"-- f2 obj not found: {jnt_fc2_obj}\n")
            # emit an error?
            pass

        # NOTE: following code is to add the joint to the ASM
        # add_grounded(asm, base_obj)
        # base_obj.recompute()

        # rev1 = add_revolute(
        #     asm, "Revolute001",
        #     V3(0.0, 317.0, 0.0), Rotation(*[0.0, 0.0, -90.0]),
        #     V3(0.0, 317.0, 0.0), Rotation(*[180.0, 0.0, 90.0]),
        #     [jnt1_obj, ['Face1977', 'Face1977']],
        #     [base_obj, ['Face792', 'Face792']],
        #     Placement(VEC0, Rotation(180, 0, 0)),
        #     Placement(VEC0, ROT0)
        # )

        # rev1.recompute()

    def select_face(self, dbg_s=False):
        """Select Face."""
        dbg_s = True  # DBG
        if dbg_s:
            fcl_msg(f"Select Face{self.jnt_fc}\n")
        # FIXME: getSelectionEx() seems not to work correctly with elements
        #        contained in "Part container". InList seems to not help here
        #        we must check the selection to obtain the Part Container Name

        raw_sel = Gui.Selection.getSelectionEx()
        #
        if dbg_s:
            fcl_msg(f"Raw_ sel: {raw_sel}\n")

        esn = len(raw_sel)
        if esn == 0:
            msg_box(
                self, "Robot_tools", self.fnt,
                "<b>Joint Face Selection</b><br><br>You must select a Face")
            return False
        elif esn == 1:
            s0 = raw_sel[0]
            obj = s0.Object
            sub_ent = s0.SubElementNames
            obj_typ = obj.TypeId
            # obj_il = obj.InList
            fcl_msg((
                f"Selection: {s0}\n"
                f"Obj Name: {obj.Name}\n"
                f"Obj Label: {obj.Label}\n"
                f"Obj Type: {obj_typ}\n"
                # f"Obj InList: {obj_il}\n"
                f"Obj SubElements: {sub_ent}\n"
            )
            )
            jnt_elnm = f"joint{self.jnt_ec:02d}{self.jnt_fc:02d}"
            self.jnt_meta[jnt_elnm] = {}
            self.jnt_meta[jnt_elnm]["ob_nm"] = obj.Name
            self.jnt_meta[jnt_elnm]["sub_el"] = sub_ent
            #
            if self.jnt_fc == 1:
                self.jnt_fc = 2
                set_wid_en(self, ("btn_jnt_s1",), False)
                set_wid_en(self, ("btn_jnt_s2", "btn_jnt_add"), True)
            else:
                pass

    # --------------------------------------------
    #                Other functions
    # --------------------------------------------

    def act_dump_log(self):
        """Executed when "Dump Log is pressed."""
        if self.isInit is True:
            return
        tm_st = datetime.strftime(datetime.now(), mdtsp)
        log_fn = f"study_robot_{tm_st}.log"
        self.log_fn = pathlib.Path(MODULE_PATH.joinpath(log_fn))
        dump_log(self.log_fn)


def run():
    dialog = O2PDialog()
    dialog.show()

# Executed if not imported as module


if __name__ == "__main__":
    dialog = O2PDialog()
