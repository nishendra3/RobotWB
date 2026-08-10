"""
g_rbt_traj_wp_table.py
waypoint table renderer for the trajectory panel
"""
from __future__ import annotations

from typing import Optional, Callable, List

from PySide import QtCore, QtGui  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QComboBox, QTableWidget, QTableWidgetItem)

from freecad.Robot_tools.App.rbt_global_constants import ap_clr
from freecad.Robot_tools.App.rbt_traj_types import (
    CARTESIAN, JOINT, SolvedWaypoint, TargetMode,
    LIN, PTP, MotionType)

CLR_ERR = QtGui.QColor(ap_clr["B_Red"][1])
CLR_PACE = QtGui.QColor(ap_clr["HC_Yellow"][1])

# column order
(COL_NUM, COL_NAME, COL_MODE, COL_MOTION,
 COL_PACE, COL_UNIT, COL_POSE, COL_STATUS) = range(8)


class WaypointTable:
    """
    Render a table containing the waypoints and their
    properties including mode & motion type
    """

    def __init__(self, tbl: QTableWidget,
                 on_rename: Callable[[int, str], None],
                 on_mode: Callable[[int, TargetMode], None],
                 on_motion: Callable[[int, MotionType], None],
                 on_pace: Callable[[int, Optional[float],
                                    bool], None]) -> None:
        self.tbl = tbl
        self.on_rename = on_rename
        self.on_mode = on_mode
        self.on_motion = on_motion
        self.on_pace = on_pace
        self.updating = False    # render echo guard
        tbl.itemChanged.connect(self.item_changed)

    # ---------- queries ----------

    def sel_row(self) -> int:
        """out: selected row index, -1 when none"""
        rows = self.tbl.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    # ---------- render ----------

    def fill(self, solved: List[SolvedWaypoint], pace_row: int,
             pace_msg: str = "") -> None:
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
                    self.tbl.setItem(row, COL_STATUS, self.ro_item(pace_msg))
                    self.tint_row(row, CLR_PACE)
        finally:
            self.updating = False

    def fill_row(self, row: int, sw: SolvedWaypoint) -> None:
        wp = sw.wp
        pose_txt = "{:.0f}, {:.0f}, {:.0f}".format(*wp.tcp_in_world.Base)
        val = wp.duration if wp.duration is not None else wp.speed
        val_txt = "" if val is None else f"{val:g}"

        self.tbl.setItem(row, COL_NUM, self.ro_item(str(row + 1)))
        self.tbl.setItem(row, COL_NAME, QTableWidgetItem(wp.name))
        self.tbl.setItem(row, COL_PACE, QTableWidgetItem(val_txt))
        self.tbl.setItem(row, COL_POSE, self.ro_item(pose_txt))
        self.tbl.setItem(row, COL_STATUS, self.ro_item(sw.err_msg or "OK"))

        # mode combo dorpdown
        self.tbl.setCellWidget(row, COL_MODE, self.combo(
            items=["Joint", "Cartesian"],
            current="Cartesian" if wp.mode == CARTESIAN else "Joint",
            tip="what the point stores: joint or cartesian vals",
            on_pick=lambda txt, r=row: self.on_mode(
                r, CARTESIAN if txt == "Cartesian" else JOINT)))

        # motion combo dropdown
        self.tbl.setCellWidget(row, COL_MOTION, self.combo(
            items=["PTP", "LIN"],
            current="LIN" if wp.motion == LIN else "PTP",
            tip="motion type to get here from the previous point",
            on_pick=(lambda txt, r=row: self.on_motion(
                r, LIN if txt == "LIN" else PTP))))

        # motion units dropdown
        speed_unit = "mm/s" if wp.motion == LIN else "%"
        self.tbl.setCellWidget(row, COL_UNIT, self.combo(
            items=[speed_unit, "sec"],
            current="sec" if wp.duration is not None else speed_unit,
            tip="unit for speed or duration",
            on_pick=lambda txt, r=row: self.on_pace(
                r, self.pace_val(r), txt == "sec")))

    @staticmethod
    def ro_item(text: str) -> QTableWidgetItem:
        """
        read-only cell
        """
        item = QTableWidgetItem(text)
        item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
        return item

    def combo(self, items, current: str, tip: str, on_pick) -> QComboBox:
        """
        table cell dropdown
        """
        cb = QComboBox()
        cb.addItems(items)
        cb.setCurrentText(current)
        cb.setToolTip(tip)
        cb.currentTextChanged.connect(
            lambda txt: None if self.updating else on_pick(txt))
        return cb

    def tint_row(self, row: int, clr: QtGui.QColor) -> None:
        for col in (COL_NUM, COL_NAME, COL_POSE, COL_PACE, COL_STATUS):
            item = self.tbl.item(row, col)
            if item:
                item.setBackground(clr)

    # ---------- edit forwarding ----------

    def item_changed(self, item: QTableWidgetItem) -> None:
        if self.updating:
            return
        row, col = item.row(), item.column()
        if col == COL_NAME:
            self.on_rename(row, item.text())
        elif col == COL_PACE:
            txt = item.text().strip()
            try:
                val = float(txt) if txt else None
            except ValueError:
                return

            combo = self.tbl.cellWidget(row, COL_UNIT)
            self.on_pace(row, val, combo.currentText() == "sec")

    def pace_val(self, row: int) -> Optional[float]:
        """
        number in the pace cell
        """
        item = self.tbl.item(row, COL_PACE)
        txt = item.text().strip() if item else ""
        try:
            return float(txt) if txt else None
        except ValueError:
            return None
