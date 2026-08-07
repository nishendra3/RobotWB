"""
g_rbt_traj_gplayer.py
global player: run several trajectories on one shared clock
"""
from __future__ import annotations

from typing import List, Optional
from dataclasses import dataclass

import FreeCADGui as Gui  # type: ignore
from PySide import QtCore  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QCheckBox, QDialogButtonBox, QHBoxLayout, QLabel, QPushButton,
    QSlider, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from freecad.Robot_tools.Gui.rbt_helpers_ui import getObjByName, load_panel_ui
from freecad.Robot_tools.App import rbt_traj, rbt_traj_plan
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_robot import all_robots
from freecad.Robot_tools.Gui.g_rbt_traj_player import (
    TrajTrack, TrajectoryPlayer, PlaybackControls)
from freecad.Robot_tools.App.rbt_traj_plan import TrajectoryPlan
from freecad.Robot_tools.App.rbt_traj_types import DocObj

COL_ON, COL_ROBOT, COL_TRAJ, COL_DUR, COL_STATUS = range(5)


@dataclass(frozen=True)
class TrajRow:
    robot: DocObj
    traj: DocObj
    plan: Optional[TrajectoryPlan]   # None -> row not playable
    status: str


class GlobalPlayerWidget(QWidget):
    """
    One row per trajectory in the document
    """

    def __init__(self) -> None:
        super().__init__()
        self.rows: List[TrajRow] = []
        self.build_ui()
        self.playback = PlaybackControls(
            self.btn_play, self.btn_stop, self.sl_time,
            self.lbl_time, self.sl_scale, self.chk_loop)
        self.reload()

    # ---------- ui ----------

    def build_ui(self) -> None:
        self.tbl = QTableWidget(0, 5)
        self.tbl.setHorizontalHeaderLabels(
            ["On", "Robot", "Trajectory", "Duration", "Status"])
        self.tbl.horizontalHeader().setStretchLastSection(True)
        self.play_ui = load_panel_ui("tp_rbt_traj_playback.ui")
        for name in ("btn_play", "btn_stop", "sl_time",
                     "lbl_time", "sl_scale", "chk_loop"):
            setattr(self, name, getObjByName(self.play_ui, name))

        col = QVBoxLayout(self)
        col.addWidget(self.tbl)
        col.addWidget(self.play_ui)

    # ---------- data ----------

    def reload(self) -> None:
        """re-scan the document and rebuild rows + player"""
        self.rows = []
        for robot in all_robots():
            for traj in rbt_traj.rbt_trajectories(robot):
                self.rows.append(self.make_row(robot, traj))
        self.fill_table()
        self.rebind()

    def make_row(self, robot: DocObj, traj: DocObj) -> TrajRow:
        plan, _, err = rbt_traj_plan.get_plan(traj)
        return TrajRow(robot, traj, plan if not err else None, err or "OK")

    def fill_table(self) -> None:
        self.tbl.setRowCount(len(self.rows))
        for i, row in enumerate(self.rows):
            chk = QCheckBox()
            chk.setChecked(row.plan is not None)
            chk.setEnabled(row.plan is not None)
            chk.toggled.connect(self.rebind)
            self.tbl.setCellWidget(i, COL_ON, chk)
            self.tbl.setItem(i, COL_ROBOT,
                             QTableWidgetItem(row.robot.Label))
            self.tbl.setItem(i, COL_TRAJ,
                             QTableWidgetItem(row.traj.Label))
            dur = f"{row.plan.duration:.2f} s" if row.plan else "-"
            self.tbl.setItem(i, COL_DUR, QTableWidgetItem(dur))
            self.tbl.setItem(i, COL_STATUS, QTableWidgetItem(row.status))

    # ---------- player ----------

    def checked_tracks(self) -> List[TrajTrack]:
        tracks = []
        for i, row in enumerate(self.rows):
            chk = self.tbl.cellWidget(i, COL_ON)
            if row.plan is not None and chk and chk.isChecked():
                tracks.append(TrajTrack(row.robot, row.plan))
        return tracks

    def rebind(self, *_args) -> None:
        tracks = self.checked_tracks()
        self.playback.bind(
            TrajectoryPlayer(tracks) if tracks else None)

    def cleanup(self) -> None:
        self.playback.bind(None)


class GlobalPlayerPanel:
    """task panel shell"""

    def __init__(self) -> None:
        self.widget = GlobalPlayerWidget()
        self.form = self.widget

    def getStandardButtons(self):
        return QDialogButtonBox.Close

    def reject(self) -> None:
        self.widget.cleanup()
        Gui.Control.closeDialog()


def run() -> None:
    """entry point for the RBT_playtraj command"""
    if Gui.Control.activeDialog():
        fcl_warn("close the active task panel first\n")
        return
    Gui.Control.showDialog(GlobalPlayerPanel())
