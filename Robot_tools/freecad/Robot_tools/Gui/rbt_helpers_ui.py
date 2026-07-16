"""Robot UI helper functions.

Name: rbt_helpers_ui.py

See Changelog below.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

import os
import FreeCADGui as Gui  # type: ignore

from PySide.QtWidgets import (  # noqa # type: ignore
    QCheckBox,  QFrame, QGroupBox, QLabel,
    QDoubleSpinBox, QSlider, QToolButton,
    QPushButton,
    QFileDialog, QMessageBox,  # Dialogs
    QGridLayout, QVBoxLayout, QScrollArea)  # Layouts and Policy
from PySide.QtCore import QObject, Qt  # type: ignore # noqa
from PySide.QtGui import QIcon  # type: ignore # noqa

from freecad.Robot_tools.App.rbt_global_constants import ap_clr
from freecad.Robot_tools import rbt_locator


# ------------------------------------------------
#                   Color setters
# ------------------------------------------------


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

# ------------------------------------------------
#                   UI functions
# ------------------------------------------------


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


def cm_btn(parent, b_nm, b_txt, b_fnt):
    """Create a styled QPushButton."""
    button = QPushButton()
    button.setObjectName(b_nm)
    button.setFont(b_fnt)
    button.setAutoDefault(False)
    button.setText(b_txt)

    if parent is None:
        pass
    else:
        button.setParent(parent)

    return button


def cm_tool_btn(parent, b_nm, b_txt, b_fnt):
    """Create a styled QToolButton."""
    btn = QToolButton(parent)
    btn.setObjectName(b_nm)
    btn.setText(b_txt)
    btn.setFont(b_fnt)
    btn.setAutoRaise(True)
    return btn


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


def cm_dspb(parent, sb_nm, sb_fnt, sb_min=-180, sb_max=180,
            sb_dec=2, sb_step=0.1, sb_suf="°"):
    """Creates a styled QDoubleSpinBox"""
    sb = QDoubleSpinBox()
    sb.setObjectName(sb_nm)
    sb.setFont(sb_fnt)
    sb.setDecimals(sb_dec)
    sb.setRange(sb_min, sb_max)
    sb.setSingleStep(sb_step)
    sb.setSuffix(sb_suf)
    sb.setButtonSymbols(QDoubleSpinBox.NoButtons)
    if parent is not None:
        sb.setParent(parent)
    return sb


def cm_slider(parent, sl_nm, sl_min=-180, sl_max=180, sl_scale=100):
    """
    Creates a horizontal QSlider that links to a scale.
    - Write: sl.setValue(int(angle*sl._scale))
    - Read : sl.value() / sl._scale
    """
    sl = QSlider(Qt.Horizontal)
    sl.setObjectName(sl_nm)
    sl.setRange(int(sl_min * sl_scale), int(sl_max * sl_scale))
    sl._scale = sl_scale
    if parent is not None:
        sl.setParent(parent)
    return sl


def cm_toggle(parent, ct_nm, ct_fnt):
    """Creates circlular flip toggle"""
    ck = QCheckBox(parent)
    ck.setObjectName(ct_nm)
    ck.setFont(ct_fnt)
    return ck


def cm_scroll(parent, sa_nm, inner_wd):
    """Wrap in borderless QScrollArea"""
    sa = QScrollArea()
    sa.setObjectName(sa_nm)
    sa.setWidget(inner_wd)
    sa.setWidgetResizable(True)
    sa.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    sa.setFrameShape(QFrame.NoFrame)
    if parent is not None:
        sa.setParent(parent)
    return sa


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
                parent,
                'WARNING:',
                fnt,
                "You must select only one file.",
                "w", "w")
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


def is_alive(fc_obj):
    """
    check if the fc doc has not already
    been closed by the user
    """
    if fc_obj is None:
        return False
    try:
        fc_obj.Name
        return True
    except ReferenceError:
        return False


def load_panel_ui(filename):
    """
    Helper to load .ui files located at Gui/resources/ui/<filename>.
    """
    wb_path = os.path.dirname(rbt_locator.__file__)
    ui_path = os.path.join(wb_path, "resources", "ui", filename)
    return Gui.PySideUic.loadUi(ui_path)


JOINT_TYPE_ICONS = {
    "Fixed":    ":/icons/Assembly_CreateJointFixed.svg",
    "Revolute": ":/icons/Assembly_CreateJointRevolute.svg",
    "Slider":   ":/icons/Assembly_CreateJointSlider.svg",
    "Base":     ":/icons/Assembly_ToggleGrounded.svg",
}


def joint_type_icon(name):
    return QIcon(JOINT_TYPE_ICONS
                 .get(name, ":/icons/Assembly_CreateJoint.svg"))
