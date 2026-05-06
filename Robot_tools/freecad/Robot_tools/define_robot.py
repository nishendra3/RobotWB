"""Robot Define New.

Name: define_robot.py

See Changelog below.

Author: Carlo Dormeletti
Copyright: 2026
Licence: All right reserved
"""
__version__ = "0.14"
__build__ = "20260429_1534"


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
from freecad.Robot_tools.rbt_objects import Robot_obj, ViewProviderRBo
from freecad.Robot_tools.rbt_fc_observer import RbtObserver

"""
----------------------------------------
Changelog:
----------------------------------------
v0.01 - Initial version.
v0.02 - Added create_asm code.
v0.03 - Added add joint action (unfinished).
v0.04 - Added some refinement in UI and various.
v0.05 - Added actions needed when loading an asm doc:
         - Check the asm document for "big errors" like the absence of Robot_FPO
           and Robot_Assembly object.
         - Check for empty joints and signal it with a messagebox.
v0.06 - Added code to add joints (grounded).
v0.07 - Improved check_asm.
v0.08 - Solved 'Linked Part container' selection quirk.
v0.09 - Added code to add revolute joint.
v0.10 - Added code to correctly populate Robot_FPO.
v0.11 - Some fix in add joints logic.
v0.12 - Updated to be used also in robot_test.
      - Message boxes fixed to show program name and the context.
v0.13 - Updated program run detection logic.
v0.14 - Added some code to analyse faces.
"""

fcl_err = App.Console.PrintError
fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning

mdtsp = "%y%m%d_%H%M"
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

msg_log = []  # log message container, used to permit a dump.

# ------------------------------------------------
#                 Module functions
# ------------------------------------------------


def dump_log(log_fn):
    """Dump log to a file."""
    fcl_msg(f"Log file name: {log_fn}")

# ------------------------------------------------
#                Geometric functions
# ------------------------------------------------


def find_center(jnt_nm, jnt_data, dbg_s=False):
    """Find center from edges of joint mating faces."""
    ptc_f1 = jnt_data['fcc_f1']
    ptc_f2 = jnt_data['fcc_f2']
    #
    if dbg_s:
        dbg_msg = (
            f"-- {jnt_nm}\n"
            f"face1 >> {ptc_f1}\n"
            f"face2 >> {ptc_f2}\n")
        fcl_msg(dbg_msg)
    pts_f1 = sort_centers(ptc_f1)
    pts_f2 = sort_centers(ptc_f2)
    #
    sptf1 = sorted(pts_f1, key=lambda x: x[1])
    sptf2 = sorted(pts_f2, key=lambda x: x[1])
    #
    if dbg_s:
        dbg_msg = (
            f"f1: {sptf1[-1]}\n"
            f"f2: {sptf2[-1]}\n")
        fcl_msg(dbg_msg)
    return sptf1, sptf2


def sort_centers(c_lst):
    """Sort centers data."""
    # fcl_msg(f"list: {c_lst}\n")  # DBG
    tol = 0.001  # FIXME: Raise the tol value?
    pts = []
    n_pts = []
    cnt = 1
    lc = True
    mn = len(c_lst) - 1
    pts.append(V3(*c_lst[0]))
    n_pts.append(1)
    while lc:
        pt_v2 = V3(*c_lst[cnt])
        for v_idx, v in enumerate(pts):
            dt = v.distanceToPoint(pt_v2)
            if dt > tol:
                pts.append(pt_v2)
                n_pts.append(1)
            else:
                n_pts[v_idx] += 1
        # print(f"cnt: {cnt}, {mn} pts: {pts} n_pts: {n_pts}\n")
        cnt += 1
        if cnt >= mn:
            lc = False

    s_list = list(zip(pts, n_pts))

    return s_list


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
    asm.Document.recompute()


def add_revolute(asm, j_refs, j_lbl, dbg_s=False):
    """Add a revolute joint."""
    joint_group = UtilsAssembly.getJointGroup(asm)
    revj = joint_group.newObject("App::FeaturePython", "RevoluteJoint")
    revj.Label2 = j_lbl
    a_jnt = JointObject.Joint(revj, 1)
    JointObject.ViewProviderJoint(revj.ViewObject)
    #
    revj.Reference1 = j_refs[0]
    revj.Reference2 = j_refs[1]

    revj.recompute()
    asm.Document.recompute()
    #
    pl1 = a_jnt.findPlacement(revj, revj.Reference1, 0)
    pl2 = a_jnt.findPlacement(revj, revj.Reference2, 0)
    #
    if dbg_s:
        fcl_msg((
            f"Reference1: {pl1}\n"
            f"Reference2: {pl2}\n")
        )
    #
    revj.recompute()
    asm.Document.recompute()

    return revj


def create_assembly(doc):
    asm = doc.addObject("Assembly::AssemblyObject", "Assembly")
    asm.Label = "Robot_Assembly"
    asm.Type = "Assembly"
    asm.newObject("Assembly::JointGroup", "Joints")
    asm.recompute()
    return asm


def resolve_asm_ref(asm_doc, refs):
    """Resolve the asm reference returning a face."""
    obj_name = UtilsAssembly.getObject(refs)
    obj_ref = UtilsAssembly.getObjsNamesAndElement(obj_name, refs[1])
    print(f"obj_ref: {obj_ref}")

    element_name = UtilsAssembly.getElementName(obj_ref[1])
    elt_type, el_num = UtilsAssembly.extract_type_and_number(element_name)

    if elt_type == "Face":
        face = obj_name.Shape.getElement(element_name)
        return face
    else:
        return None


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
        av_geom = QtGui.QGuiApplication.primaryScreen().availableGeometry()
        av_wid = av_geom.width()
        av_hei = av_geom.height()
        w_wid = av_wid * 0.20
        w_hei = av_hei * 0.33
        x_loc = (av_wid - w_wid) * 0.5
        y_loc = (av_hei - w_hei) * 0.5
        # define window xLoc,yLoc,xDim,yDim
        self.setGeometry(x_loc, y_loc, w_wid, w_hei)
        self.setWindowTitle(" ")  # MacOS has no title in some cases
        self.setWindowFlags(
            self.windowFlags() | Qt.WindowStaysOnTopHint
        )

        self.fnt = QApplication.font("QMessageBox")
        lb_fm = QtGui.QFontMetricsF(self.fnt)
        self.lb_em = round(lb_fm.maxWidth())

        # Determine if we are running as a module of Robot Tools WB/TB
        mps = str(MODULE_PATH.stem)
        if mps == 'Robot_tools' or mps == 'Robot_test':
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
        self.lbl_sw = cm_lbl(
            self, "lbl_sw",
            f"<b>{pg_name} - {self.ui_title}</b> - Build: {__build__}", self.fnt, 0)
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

        # fcl_msg(f"isWBcomp {self.isWBcomp}\n ")  # DBG

        row = 0

        # groupBox helper
        def _make_gbx(name, desc):
            gb, gbl = cm_gbx(self, name, desc)
            gb.setStyleSheet(self.grb_ss)
            gb.setLayout(gbl)
            return gb, gbl

        # -- Robot Files --
        gb_rc, gb_rcl = _make_gbx("gb_rc", "Robot Files")
        self.flags['rob_fpo'] = True

        selection_rows = [
            # rob components file selection
            ("txt_rbnm", "btn_selrbf", "btn_sel_rbf", "Select 'Component' file",
             "Select the FCStd file that contains the robot components.", self.set_working_model),

            # rob assembly file selection
             ("txt_asnm", "btn_selasm", "btn_sel_asm", "Select 'Assembly' file",
             "Select the FCStd file that contains Robot_Assembly and the Robot_FPO.", self.set_working_asm),
        ]
        brow = 0
        for txt_name, btn_name, btn_attr, btn_lbl, tool_tip, callback in selection_rows:
            txt = cm_ledit(self, txt_name, self.fnt, 0)
            gb_rcl.addWidget(txt, brow, 0, 1, 4)
            setattr(self, txt_name, txt)
            brow += 1

            btn = cm_btn(self, btn_name, btn_lbl, self.fnt, 0)
            btn.setToolTip(tool_tip)
            btn.clicked.connect(callback)
            gb_rcl.addWidget(btn, brow, 0, 1, 2)
            setattr(self, btn_attr, btn)
            brow += 1
        
        ma_lay.addWidget(gb_rc, row, 0, 1, 4)
        row += 1

        # -- Functions --
        gb_bt, gb_btl = _make_gbx("gb_bt", "Functions")
        func_btns = [
            ("btn_cre_asm", "Create ASM", self.create_asm),
            ("btn_load_step", "Load STEP", self.act_load_step),
        ]

        brow = 0
        for btn_name, btn_lbl, callback in func_btns:
            btn = cm_btn(self, btn_name, btn_lbl, self.fnt, 0)
            btn.clicked.connect(callback)
            btn.setEnabled(False)
            gb_btl.addWidget(btn, brow, 0, 1, 2)
            brow += 1

        ma_lay.addWidget(gb_bt, row, 0, 1, 4)

        row += 1
        
        # -- Empty Spacer --
        tp_gb0, _ = _make_gbx("tp_gb0", "")

        ma_lay.addWidget(tp_gb0, row, 0, 1, 4)
        row += 1

        # -- Joints --
        jn_gb0, jn_gb0l = _make_gbx("jn_gb0", "Joints")
        headers = ["Joint", "Type", "Face1", "Face2"]
        brow = 0
        for header in headers:
            lbl = cm_lbl(self, f"lbl_jnspc{brow}", f"<b>{header}</b>", self.fnt, 0)
            jn_gb0l.addWidget(lbl, 0, brow, 1, 1)
            brow += 1
        
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
        if self.flags.get("chb_gdj", True):
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

    def refresh_joints_panel(self):
        # -- update joints panel --
        # 1. delete the widgets
        lay = self.findChild(QObject, "jn_gb0_l")
        while lay.count() > 4:
            item = lay.takeAt(lay.count() - 1)
            w = item.widget()
            if w:
                w.deleteLater()
        # 2. reset the state vars
        self.jnt_ec = 0
        self.jnt_fc = 1
        self.jnt_meta = {}
        self.flags["chb_gdj"] = True
        # 3. rebuild from start
        self.check_asm()

        # -- update grounded checkbox --
        chb = self.findChild(QObject, "chb_gdj")
        if chb is not None:
            f = self.flags.get("chb_gdj", True)
            chb.setChecked(f)
            chb.setEnabled(f)

    def closeEvent(self, event):
        """Called on close event."""
        Gui.Selection.removeSelectionGate()

        # clean up the document observer
        if getattr(self, "doc_observer", None) is not None:
            self.doc_observer.stop()
            self.doc_observer = None

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
        names = ("lbl_jnt", "lbl_jt", "lbl_jf1", "lbl_jf2")

        for idx, (name,val) in enumerate(zip(names, j_data)):
            lbl = cm_lbl(self, f"{name}{jpr}", str(val), self.fnt, 0)
            lbl.setFrameShape(QFrame.Shape.Panel)
            lbl.setFrameShadow(QFrame.Shadow.Sunken)
            lbl.setStyleSheet("QLabel {background-color: palette(base); color: palette(text);}")
            lbl.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Minimum)
            lay.addWidget(lbl, jpr, idx, 1, 1)

    # --------------------------------------------
    #                Model functions
    # --------------------------------------------

    def _open_doc(self, doc_name, attr):
        """helper for setting robot/assm. models"""
        f_name = get_file(self, self.fnt, ftype="fcstd", pre_dir="")
        # fcl_msg(f"Set working model: {f_name}\n")
        if not f_name: 
            return None
        o_doc = App.openDocument(f_name)
        fcl_msg(f"open doc: {o_doc}\n")
        setattr(self, attr, o_doc)
        wid = getObjByName(self, doc_name)
        if wid is None:
            return None
        wid.setText(str(f_name))
        return o_doc
    
    def _install_observer(self):
        """observer to monitor deletion events"""
        if getattr(self,"doc_observer", None) is not None:
            return
        self.doc_observer = RbtObserver(self)

    def set_working_model(self):
        """Set robot working model."""
        if self._open_doc("txt_rbnm", "wk_mod"):
            set_wid_en(self, ("btn_cre_asm",), True)

    def set_working_asm(self):
        """Set robot assembly file."""
        o_doc = self._open_doc("txt_asnm", "wk_asm_d")
        if o_doc:
            self.check_asm()
            self._install_observer()
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

        # save refs
        self.wk_asm_d = asm_doc
        self.wk_asm = asm
        self.rob_obj = rb_obj

        # Disable "Create ASM" button
        set_wid_en(self, ("btn_cre_asm",), False)
        # Create the "Add joint" groupbox
        self.jnt_setui()

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
                self, " ", self.fnt,
                (f"<b>{pg_name} - <b>Check Assembly</b><br><br>"
                 "There are no Robot Assembly  definitions in the document.<br>"
                 "You must select a correct document or create one"))
            # FIXME: See if the activation of Create ASM button is correct here.
            set_wid_en(self, ("btn_cre_asm",), True)
            return

        if ran == 1:
            self.wk_asm = asm_objs[0]
        else:
            msg_box(
                self, " ", self.fnt,
                (f"<b>{pg_name} - <b>Check Assembly</b><br><br>"
                 f"<b>There are: [{ran}] Robot Assembly objects in the document"
                 "</b><br><br>You must select one"))
            # TODO: add the corresponding selection dialog.
            return

        rob_objs = self.wk_asm_d.getObjectsByLabel("Robot_FPO")
        ron = len(rob_objs)
        if ron > 0:
            if ron == 1:
                self.rob_obj = rob_objs[0]
            else:
                msg_box(
                    self, " ", self.fnt,
                    (f"<b>{pg_name} - <b>Check Assembly</b><br><br>"
                     f"<b>There are: [{ron}] FPO in the assembly</b><br><br>"
                     "You must select the working one"))
                # TODO: create the selection dialog
                return
        else:
            msg_box(
                self, f"{pg_name}", self.fnt,
                (f"<b>{pg_name} - <b>Check Assembly</b><br><br>"
                 "<b>There are no Robot FPO in the assembly</b><br><br>"
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
                self, " ", self.fnt,
                (f"<b>{pg_name} - <b>Check Assembly</b><br><br>"
                 "<b>There are no Joints in the assembly</b><br><br>"
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
            self.jnt_ec += ojl
            for jn_idx, joint in enumerate(self.wk_asm.Joints):
                jnt_lb2 = joint.Label2
                jnt_typ = joint.JointType
                jnt_ref1 = joint.Reference1
                jnt_ref2 = joint.Reference2
                # NOTE: eventually tune the dbg output
                if dbg_s:
                    fcl_msg((
                        f" -- Joint Number > {jn_idx}\n"
                        f" -- Joint Name > {joint.Name}\n"
                        f" -- Joint Label > {joint.Label}\n"
                        f" -- Joint Label2 > {jnt_lb2}\n"
                        f" -- Joint TypeId > {jnt_typ}\n"
                        f" -- Joint Reference1 > {jnt_ref1}\n"
                        f" -- Joint Reference2 > {jnt_ref2}\n"
                        )
                    )
                if jnt_lb2[:6] == "rb_jnt":
                    jr1l = f"{jnt_ref1[0].Name}.{jnt_ref1[1][0]}"
                    jr2l = f"{jnt_ref2[0].Name}.{jnt_ref2[1][0]}"
                    self.add_joint2ui(
                        jn_idx + 3, [jn_idx + 1, jnt_typ, jr1l, jr2l])

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
        
        if not self.jnt_meta:
            msg_box(self, " ", self.fnt, "<b>Add Joint</b><br><br>You must select at least one face first")
            return

        dks = list(self.jnt_meta)
        jn = int(dks[0][5:7])
        jf1 = int(dks[0][7:9])
        jmo0 = self.jnt_meta[dks[0]]
        jnt_fc1_onm = jmo0['ob_nm']
        jnt_fc1_obj = jmo0['ob_obj']
        jnt_fc1_oref = jmo0['ob_ref']

        if dbg_s:
            fcl_msg(f"Joint number {jn} Face {jf1} jid: {jmo0}\n")
            fcl_msg((
                f"-- f1 obj name: {jnt_fc1_onm}\n"
                f"-- f1 obj ref: {jnt_fc1_oref}\n"
                f"-- f1 obj Label: {jnt_fc1_obj.Label}\n"
                f"-- f1 obj Label2: {jnt_fc1_obj.Label2}\n")
            )

        if len(dks) == 1:
            # one face selected, so the joint is probably the grounded one.
            chb = self.findChild(QObject, "chb_gdj")
            if chb is not None and chb.isChecked(): #old-> jn == 0:
                add_grounded(self.wk_asm, jnt_fc1_obj)
                # Reset selection state
                self.jnt_fc = 1
                self.jnt_meta = {}
                set_wid_en(self, ("btn_jnt_s1",), True)
                # refresh_joint_panel to rebuild the table ui
                self.refresh_joints_panel()
                return

                # # This is probably a grounded joint as it is logically the first
                # # to be set.
                # add_grounded(self.wk_asm, jnt_fc1_obj)
                # self.wk_asm.recompute()
                # # Reset the interface
                # set_wid_en(self, ("btn_jnt_s1",), True)
                # set_wid_en(self, ("chb_gdj",), False)
                # # TODO: populate the joint frame with a joint type.
                # self.add_joint2ui(jn + 2, [0, "grounded", "--", "--"])
                # # advance joint counter and reset face counter and data dict
                # self.jnt_ec += 1
                # self.jnt_fc = 1
                # self.jnt_meta = {}
                # return
            else:
                # Emit an error as two faces are needed
                # one face but not grounded - invalid selection
                msg_box(self, " ", self.fnt,
                        "<b>Add Joint</b><br><br>Two faces are required for a non-grounded joint.")
                return

        # At this point we could be sure that there are two objects in the list
        jf2 = int(dks[1][7:9])
        jmo1 = self.jnt_meta[dks[1]]
        jnt_fc2_onm = jmo1['ob_nm']
        jnt_fc2_obj = jmo1['ob_obj']
        jnt_fc2_oref = jmo1['ob_ref']
        #
        if dbg_s:
            fcl_msg((
                f"Joint number {jn} Face {jf2} jmo1: {jmo1}\n"
                f"-- f2 obj name: {jnt_fc2_onm}\n"
                f"-- f2 obj ref: {jnt_fc2_oref}\n")
            )
        #
        if jnt_fc2_obj is not None:
            # NOTE: operations sequence:
            # 1) validate the sequence
            if dbg_s:
                fcl_msg(f"-- f2 obj Label: {jnt_fc2_obj.Label}\n")
                fcl_msg(f"-- f2 obj Label2: {jnt_fc2_obj.Label2}\n")

            # NOTE: we must use a conventional Label2 composed as follow:
            #  rb_jntXX (robot joint) X is an integer it will permit 99 joints
            #  ee_jntXX (end effector) same as above
            #
            # this way we could sort out using Label2 immediately
            # the joint type as there is no way to distinguish them by Name
            # alternatively we could plan as example
            # rb_ro_jnt the added _ro_ will mean rotation but could also be
            # sl = sliding, or pr = prismatic. Let me know

            j_lbl = f"rb_jnt{self.jnt_ec:02d}"  # see note above
            j_ref1 = [jnt_fc1_oref, jnt_fc1_oref]
            j_ref2 = [jnt_fc2_oref, jnt_fc2_oref]
            
            revj = add_revolute(self.wk_asm, [(jnt_fc1_obj, j_ref1), (jnt_fc2_obj, j_ref2)], j_lbl)
            self.add_link2FPO(revj)
            # Reset selection state
            self.jnt_fc = 1
            self.jnt_meta = {}
            set_wid_en(self, ("btn_jnt_s1",), True)
            set_wid_en(self, ("btn_jnt_s2", "btn_jnt_add"), False)
            # refresh the joint panel to rebuild panel ui
            self.refresh_joints_panel()

            # revj = add_revolute(
            #     self.wk_asm, [(jnt_fc1_obj, j_ref1), (jnt_fc2_obj, j_ref2)], j_lbl)
            # self.wk_asm.recompute()
            # self.add_link2FPO(revj)
            # jr1l = f"{jnt_fc1_obj.Name}.{jnt_fc1_oref}"
            # jr2l = f"{jnt_fc2_obj.Name}.{jnt_fc2_oref}"
            # self.add_joint2ui(jn + 2, [self.jnt_ec, "revolute", jr1l, jr2l])
            # # advance joint counter and reset face counter and data dict
            # self.jnt_ec += 1
            # self.jnt_fc = 1
            # self.jnt_meta = {}
            # set_wid_en(self, ("btn_jnt_s1",), True)
            # set_wid_en(self, ("btn_jnt_s2",), False)
        else:
            if dbg_s:
                fcl_msg(f"-- f2 obj not found: {jnt_fc2_obj}\n")
            # emit an error?
            pass
    
    def add_link2FPO(self, joint):
        """Add the joint to the FPO."""
        link_cont = self.rob_obj.Robot_joints
        link_cont.append(joint)
        self.rob_obj.Robot_joints = link_cont
        #
        # Add the rotation dir data
        dir_cont = self.rob_obj.Robot_joints_dir
        dir_cont.append(1)
        self.rob_obj.Robot_joints_dir = dir_cont

    def select_face(self, dbg_s=False):
        """Select Face."""
        # dbg_s = True  # DBG
        if dbg_s:
            fcl_msg(f"Select Face{self.jnt_fc}\n")
        #
        raw_sel = Gui.Selection.getSelectionEx("",0)
        #
        if dbg_s:
            fcl_msg(f"Raw_ sel: {raw_sel}\n")

        esn = len(raw_sel)
        if esn == 0:
            msg_box(
                self, " ", self.fnt,
                (f"<b>{pg_name} - <b>Select Face</b><br><br>"
                 "<b>Joint Face Selection</b><br><br>You must select a Face"))
            return False
        elif esn == 1:
            s0 = raw_sel[0]
            obj = s0.Object
            obj_nm = obj.Name
            sub_ent = s0.SubElementNames
            obj_typ = obj.TypeId
            obj_par = obj.Parents
            fcl_msg((
                f"Selection: {s0}\n"
                f"Obj Name: {obj_nm}\n"
                f"Obj Label: {obj.Label}\n"
                f"Obj Type: {obj_typ}\n"
                f"Obj Parents: {obj_par}\n"
                f"Obj SubElements: {sub_ent}\n"
            )
            )
            # NOTE: try to determine if the object.Name exist in the Assembly doc
            #       if the object is not existent it is probably in a
            #       "Linked Part container"
            # FIXME: probably an hack!
            # UPDATE: Using get components reference of assembly wb //NISHI
            link_obj, face_ref = UtilsAssembly.getComponentReference(self.wk_asm, obj, sub_ent[0])

            if link_obj is None:
                msg_box(
                    self, " ", self.fnt,
                    (f"<b>{pg_name} - Select Face</b><br><br>"
                     "The selected face is not part of this assembly."))
                return False

            jnt_elnm = f"joint{self.jnt_ec:02d}{self.jnt_fc:02d}"
            self.jnt_meta[jnt_elnm] = {
                "ob_nm": link_obj.Name,
                "ob_obj": link_obj,
                "ob_ref": face_ref,
            }
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