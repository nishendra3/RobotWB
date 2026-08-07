"""
g_rbt_traj_tpanel.py
task panel shell for trajectory editing
"""
import FreeCADGui as Gui  # type: ignore
from PySide import QtWidgets  # type: ignore

from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_traj import is_trajectory
from freecad.Robot_tools.Gui.g_rbt_traj_ctrl_wgt import TrajectoryControlWidget


class TrajectoryTaskPanel:
    def __init__(self, traj_obj) -> None:
        self.widget = TrajectoryControlWidget(traj_obj)
        self.form = self.widget.ui

    def getStandardButtons(self):
        return QtWidgets.QDialogButtonBox.Close

    def reject(self) -> None:
        self.widget.cleanup()
        Gui.Control.closeDialog()


def run(traj_obj=None) -> None:
    """
    entry point (toolbar command + vp doubleClicked)
    in: trajectory fpo, None -> use the selected trajectory
    """
    if Gui.Control.activeDialog():
        fcl_warn("close the active task panel first\n")
        return
    if traj_obj is None:
        sel = [o for o in Gui.Selection.getSelection() if is_trajectory(o)]
        if not sel:
            fcl_warn("select a trajectory first\n")
            return
        traj_obj = sel[0]
    Gui.Control.showDialog(TrajectoryTaskPanel(traj_obj))
