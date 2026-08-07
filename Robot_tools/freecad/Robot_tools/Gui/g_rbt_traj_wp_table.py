"""
g_rbt_traj_wp_table.py
waypoint table renderer for the trajectory panel
"""
from __future__ import annotations

from typing import Callable, List

from PySide import QtCore, QtGui  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QComboBox, QTableWidget, QTableWidgetItem)

from freecad.Robot_tools.App.rbt_global_constants import ap_clr
from freecad.Robot_tools.App.rbt_traj_types import (
    CARTESIAN, JOINT, SolvedWaypoint, TargetMode)

CLR_ERR = QtGui.QColor(ap_clr["B_Red"][1])
CLR_PACE = QtGui.QColor(ap_clr["HC_Yellow"][1])

COL_NUM, COL_NAME, COL_MODE, COL_POSE, COL_STATUS = range(5)


class WaypointTable:
    """
    Renders SolvedWaypoints into the QTableWidget and answers row
    queries. Never touches the document: cell edits are forwarded
    through the two callbacks.
    Contains:
        - tbl: the bound QTableWidget
        - on_rename(row, text) / on_mode(row, TargetMode)
    """

    def __init__(self, tbl: QTableWidget,
                 on_rename: Callable[[int, str], None],
                 on_mode: Callable[[int, TargetMode], None]) -> None:
        self.tbl = tbl
        self.on_rename = on_rename
        self.on_mode = on_mode
        self.updating = False    # render echo guard
        tbl.itemChanged.connect(self.item_changed)

    # ---------- queries ----------

    def sel_row(self) -> int:
        """out: selected row index, -1 when none"""
        rows = self.tbl.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    # ---------- render ----------

    def fill(self, solved: List[SolvedWaypoint], pace_row: int) -> None:
        """
        in: solved waypoints, row to tint amber (-1 = none)
        """
        self.updating = True
        try:
            self.tbl.setRowCount(len(solved))
            for row, sw in enumerate(solved):
                self.fill_row(row, sw)
                if sw.err_msg:
                    self.tint_row(row, CLR_ERR)
                elif row == pace_row:
                    self.tint_row(row, CLR_PACE)
        finally:
            self.updating = False

    def fill_row(self, row: int, sw: SolvedWaypoint) -> None:
        wp = sw.wp
        pose_txt = "{:.0f}, {:.0f}, {:.0f}".format(*wp.tcp_in_asm.Base)
        self.tbl.setItem(row, COL_NUM, self.ro_item(str(row + 1)))
        self.tbl.setItem(row, COL_NAME, QTableWidgetItem(wp.name))
        self.tbl.setItem(row, COL_POSE, self.ro_item(pose_txt))
        self.tbl.setItem(row, COL_STATUS, self.ro_item(sw.err_msg or "OK"))
        self.tbl.setCellWidget(row, COL_MODE,
                               self.mode_combo(row, wp.mode))

    @staticmethod
    def ro_item(text: str) -> QTableWidgetItem:
        """read-only cell"""
        item = QTableWidgetItem(text)
        item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
        return item

    def mode_combo(self, row: int, mode: TargetMode) -> QComboBox:
        combo = QComboBox()
        combo.addItems(["Joint", "Cartesian"])
        combo.setCurrentText("Cartesian" if mode == CARTESIAN else "Joint")
        combo.currentTextChanged.connect(
            lambda txt, r=row: self.mode_changed(r, txt))
        return combo

    def tint_row(self, row: int, clr: QtGui.QColor) -> None:
        for col in (COL_NUM, COL_NAME, COL_POSE, COL_STATUS):
            item = self.tbl.item(row, col)
            if item:
                item.setBackground(clr)

    # ---------- edit forwarding ----------

    def item_changed(self, item: QTableWidgetItem) -> None:
        if self.updating or item.column() != COL_NAME:
            return
        self.on_rename(item.row(), item.text())

    def mode_changed(self, row: int, txt: str) -> None:
        if self.updating:
            return
        self.on_mode(row, CARTESIAN if txt == "Cartesian" else JOINT)
