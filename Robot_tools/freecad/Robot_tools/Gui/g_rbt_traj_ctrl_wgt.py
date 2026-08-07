"""
g_rbt_traj_ctrl_widget.py
coordinator widget for the trajectory panel
"""
from __future__ import annotations

from typing import Optional

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore
from PySide.QtCore import QTimer  # type: ignore
from PySide.QtWidgets import QWidget  # type: ignore

from freecad.Robot_tools.App import rbt_traj
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.App.rbt_traj_types import (
    BY_DURATION, BY_SPEED, TargetMode, TimingRequest, DocObj)
from freecad.Robot_tools.Gui.g_rbt_traj_ctrl import TrajectoryController
from freecad.Robot_tools.Gui.g_rbt_traj_wp_table import WaypointTable
from freecad.Robot_tools.Gui.g_rbt_traj_player import (
    TrajTrack, TrajectoryPlayer, PlaybackControls)
from freecad.Robot_tools.Gui.rbt_fc_observer import (
    TrajDocObserver, TrajPickObserver)
from freecad.Robot_tools.Gui.rbt_helpers_ui import (
    getObjByName, load_panel_ui, warn_box, get_qsb)


class TrajectoryControlWidget(QWidget):
    """
    Control Widget: binds .ui, owns ctrl / player / observer
    """

    UI_NAMES = (
        "lbl_robot", "le_name", "tbl_wps",
        "btn_teach", "btn_pick", "btn_xyz", "grp_xyz", "btn_xyz_add",
        "qsb_x", "qsb_y", "qsb_z", "qsb_yaw", "qsb_pitch", "qsb_roll",
        "btn_up", "btn_down", "btn_del", "btn_goto", "btn_update",
        "rb_speed", "rb_dur", "dsb_speed", "dsb_dur", "btn_floor",
        "btn_play", "btn_stop", "sl_time", "lbl_time",
        "sl_scale", "chk_loop",
    )

    def __init__(self, traj_obj) -> None:
        super().__init__()
        self.ctrl = TrajectoryController(traj_obj)
        self.observer: Optional[TrajPickObserver] = None
        self.min_duration = 0.0   # exact float behind btn_floor

        self.ui = load_panel_ui("taskpanel_rbt_traj.ui")
        self.play_ui = load_panel_ui("tp_rbt_traj_playback.ui")
        getObjByName(self.ui, "lay_play").addWidget(self.play_ui)
        for name in self.UI_NAMES:
            setattr(self, name, getObjByName(self.ui, name))
        self.table = WaypointTable(self.tbl_wps,
                                   self.on_rename_wp, self.on_mode_wp)

        self.playback = PlaybackControls(
                    self.btn_play, self.btn_stop, self.sl_time,
                    self.lbl_time, self.sl_scale, self.chk_loop)

        self.init_from_doc()
        self.connect_signals()

        self.pending_refresh: bool = False
        self.pending_select: int = -1
        self.doc_obs = TrajDocObserver(self)
        App.addDocumentObserver(self.doc_obs)
        self.refresh()

    # ================= setup =================

    def init_from_doc(self) -> None:
        """
        widgets that mirror persisted state
        """
        traj = self.ctrl.traj
        self.lbl_robot.setText(f"Robot: {self.ctrl.robot.Label}")
        self.le_name.setText(traj.Label)
        (self.rb_dur if traj.Travel_mode == "Duration"
         else self.rb_speed).setChecked(True)
        self.dsb_speed.setValue(traj.Speed)
        self.dsb_dur.setValue(traj.Target_duration)

    def connect_signals(self) -> None:
        self.le_name.editingFinished.connect(self.on_rename_traj)
        self.tbl_wps.itemSelectionChanged.connect(self.on_row_selected)

        self.btn_teach.clicked.connect(self.on_teach)
        self.btn_pick.toggled.connect(self.on_pick_toggled)
        self.btn_xyz.toggled.connect(self.grp_xyz.setVisible)
        self.btn_xyz_add.clicked.connect(self.on_xyz_add)

        self.btn_up.clicked.connect(lambda: self.on_move(-1))
        self.btn_down.clicked.connect(lambda: self.on_move(+1))
        self.btn_del.clicked.connect(self.on_delete)
        self.btn_goto.clicked.connect(self.on_goto)
        self.btn_update.clicked.connect(self.on_update_wp)

        self.rb_speed.toggled.connect(self.on_timing_edited)
        self.dsb_speed.editingFinished.connect(self.on_timing_edited)
        self.dsb_dur.editingFinished.connect(self.on_timing_edited)
        self.btn_floor.clicked.connect(self.on_apply_floor)

    # ================= refresh =================

    def on_doc_changed(self, obj: DocObj, prop: str) -> None:
        is_not_traj = obj is not self.ctrl.traj
        invalid_prop = prop not in rbt_traj.SCHEMA_PROPS
        if is_not_traj or invalid_prop:
            return
        if not self.pending_refresh:
            self.pending_refresh = True
            QTimer.singleShot(0, self.do_refresh)

    def do_refresh(self) -> None:
        try:
            self.pending_refresh = False
            self.refresh()
            if self.pending_select >= 0:
                self.tbl_wps.selectRow(self.pending_select)
                self.pending_select = -1
        except RuntimeError as e:
            fcl_warn(f"Error refreshing Traj panel : {e}\n")

    def refresh(self) -> None:
        """
        rebuild table, timing status and player
        """
        plan, solved = self.ctrl.get_plan()
        pace_row = (plan.timing.pace_seg + 1
                    if plan and not plan.timing.feasible else -1)
        self.table.fill(solved, pace_row)
        self.refresh_timing(plan)
        self.rebuild_player(plan)

    def refresh_timing(self, plan) -> None:
        """
        status button + value display
        """
        self.btn_floor.setVisible(plan is not None
                                  and not plan.timing.feasible)
        if plan is None:
            return
        rep = plan.timing
        self.min_duration = rep.min_duration
        if not rep.feasible:
            self.btn_floor.setText(rep.err_msg)
        # the inactive spinbox shows the computed value, greyed
        if self.rb_speed.isChecked():
            self.dsb_dur.setValue(rep.duration)
        else:
            self.dsb_speed.setValue(rep.speed)
        self.dsb_speed.setEnabled(self.rb_speed.isChecked())
        self.dsb_dur.setEnabled(self.rb_dur.isChecked())

    def rebuild_player(self, plan) -> None:
        player = (None if plan is None else
                  TrajectoryPlayer([TrajTrack(self.ctrl.robot, plan)]))
        self.playback.bind(player)

    # ================= waypoint actions =================

    def on_rename_traj(self) -> None:
        self.ctrl.traj.Label = self.le_name.text()

    def on_rename_wp(self, row: int, text: str) -> None:
        self.ctrl.rename_wp(row, text)

    def on_mode_wp(self, row: int, mode: TargetMode) -> None:
        self.ctrl.set_wp_mode(row, mode)

    def on_row_selected(self) -> None:
        row = self.table.sel_row()
        wps = self.ctrl.wps
        uid = wps[row].uid if 0 <= row < len(wps) else ""
        vp = getattr(self.ctrl.traj.ViewObject, "Proxy", None)
        if vp:
            vp.highlight_wp(uid)

    def on_teach(self) -> None:
        self.ctrl.teach_wp()

    def on_pick_toggled(self, active: bool) -> None:
        if active:
            self.observer = TrajPickObserver(self.on_picked_point)
            Gui.Selection.addObserver(self.observer)
        else:
            self.remove_observer()

    def on_picked_point(self, point_world) -> None:
        pose = self.ctrl.pose_from_world_point(point_world)
        if self.ctrl.add_wp_at_pose(pose) is None:
            fcl_warn("picked point not reachable\n")
        Gui.Selection.clearSelection()

    def on_xyz_add(self) -> None:
        pos = App.Vector(get_qsb(self.qsb_x), get_qsb(self.qsb_y),
                         get_qsb(self.qsb_z))
        rot = App.Rotation(get_qsb(self.qsb_yaw),
                           get_qsb(self.qsb_pitch),
                           get_qsb(self.qsb_roll))
        if self.ctrl.add_wp_at_pose(App.Placement(pos, rot)) is None:
            warn_box("Point not reachable.")

    def on_move(self, delta: int) -> None:
        row = self.table.sel_row()
        if row < 0:
            return
        self.ctrl.move_wp(row, delta)
        self.pending_select = row + delta

    def on_delete(self) -> None:
        row = self.table.sel_row()
        if row >= 0:
            self.ctrl.delete_wp(row)

    def on_goto(self) -> None:
        row = self.table.sel_row()
        if row >= 0 and not self.ctrl.goto_wp(row):
            warn_box("Waypoint not reachable.")

    def on_update_wp(self) -> None:
        row = self.table.sel_row()
        if row >= 0:
            self.ctrl.update_wp(row)

    # ================= timing =================

    def on_timing_edited(self, *_args) -> None:
        if self.rb_dur.isChecked():
            req = TimingRequest(BY_DURATION, self.dsb_speed.value(),
                                self.dsb_dur.value())
        else:
            req = TimingRequest(BY_SPEED, self.dsb_speed.value())
        self.ctrl.set_timing(req)

    def on_apply_floor(self) -> None:
        """
        apply the exact float floor (display is rounded)
        """
        self.dsb_dur.setValue(self.min_duration)
        self.ctrl.set_timing(TimingRequest(
            BY_DURATION, self.dsb_speed.value(), self.min_duration))

    # ================= teardown =================

    def cleanup(self) -> None:
        self.playback.bind(None)
        self.remove_observer()
        vp = getattr(self.ctrl.traj.ViewObject, "Proxy", None)
        if vp:
            vp.highlight_wp("")

        App.removeDocumentObserver(self.doc_obs)

    def remove_observer(self) -> None:
        if self.observer is not None:
            Gui.Selection.removeObserver(self.observer)
            self.observer = None
