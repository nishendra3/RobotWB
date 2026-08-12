"""
g_rbt_traj_player.py
trajectory playback for the robot
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from PySide.QtCore import QTimer  # type: ignore

from freecad.Robot_tools.App import rbt_kine
from freecad.Robot_tools.App.rbt_global_constants import TRAJ_TICK_MS
from freecad.Robot_tools.App.rbt_traj_plan import TrajectoryPlan
from freecad.Robot_tools.App.rbt_traj_types import DocObj


@dataclass(frozen=True)
class TrajTrack:
    """
    one robot following one plan on the shared clock
    Contains:
        - robot: robot fpo
        - plan: TrajectoryPlan
    """
    robot: DocObj
    plan: TrajectoryPlan


class TrajectoryPlayer:
    """
    Trajectory playback for the robot path.
    Contains:
        - tracks / duration
        - scale: sim speed multiplier
        - loop: restart at the end
        - on_tick(t_sec): slider/label refresh hook
        - on_state(state): "playing|paused|stopped|finished"
    """
    def __init__(self, tracks: List[TrajTrack],
                 on_tick: Optional[Callable[[float], None]] = None,
                 on_state: Optional[Callable[[str], None]] = None
                 ) -> None:
        self.tracks = list(tracks)
        self.duration: float = max(
            (trk.plan.duration for trk in self.tracks), default=0.0)
        self.scale: float = 1.0
        self.loop: bool = False
        self.on_tick = on_tick
        self.on_state = on_state

        self.t_now: float = 0.0   # elapsed trajectory time, sec
        self._t_ref: float = 0.0  # monotonic stamp of the last tick

        self._timer = QTimer()
        self._timer.setInterval(TRAJ_TICK_MS)
        self._timer.timeout.connect(self.on_timer)

    # ---------- state ----------

    @property
    def playing(self) -> bool:
        return self._timer.isActive()

    def feasible(self) -> bool:
        """
        out: True when every track's plan can run
        """
        return all(trk.plan.timing.feasible for trk in self.tracks)

    # ---------- player ----------

    def play(self) -> None:
        """
        start/resume, none when empty or any track infeasible
        """
        if not self.tracks or not self.feasible() or self.playing:
            return
        if self.t_now >= self.duration:   # replay from the start
            self.t_now = 0.0
        self._t_ref = time.monotonic()
        self._timer.start()
        self.emit_state("playing")

    def pause(self) -> None:
        """
        freeze in place
        """
        if not self.playing:
            return
        self._timer.stop()
        self.commit_all()
        self.emit_state("paused")

    def stop(self) -> None:
        """
        halt and rewind to t=0
        """
        self._timer.stop()
        self.t_now = 0.0
        self.jog_all(0.0)
        self.commit_all()
        self.emit_state("stopped")

    def scrub(self, t_sec: float, dragging: bool) -> None:
        """
        slider scrub
        in: time to show, dragging=True while the slider is held
            (jog only), False on release
        """
        self.t_now = min(max(t_sec, 0.0), self.duration)
        self.jog_all(self.t_now)
        if not dragging:
            self.commit_all()
        if self.on_tick:
            self.on_tick(self.t_now)

    # ---------- internals ----------

    def on_timer(self) -> None:
        """
        advance by real elapsed wall time
        """
        now = time.monotonic()
        self.t_now += (now - self._t_ref) * self.scale
        self._t_ref = now

        if self.t_now >= self.duration:
            if self.loop:
                self.t_now %= max(self.duration, 1e-9)
            else:
                self.t_now = self.duration
                self._timer.stop()
                self.jog_all(self.t_now)
                self.commit_all()
                if self.on_tick:
                    self.on_tick(self.t_now)
                self.emit_state("finished")
                return

        self.jog_all(self.t_now)
        if self.on_tick:
            self.on_tick(self.t_now)

    def jog_all(self, t_sec: float) -> None:
        """
        fk preview on every track
        """
        for trk in self.tracks:
            rbt_kine.set_q(trk.robot, trk.plan.q_at_time(t_sec), preview=True)

    def commit_all(self) -> None:
        """
        Offset2 write on every track at the current time
        """
        for trk in self.tracks:
            rbt_kine.set_q(
                trk.robot, trk.plan.q_at_time(self.t_now))

    def emit_state(self, state: str) -> None:
        if self.on_state:
            self.on_state(state)


class PlaybackControls:
    """
    Binds playback widgets (play/stop buttons, time slider + label,
    scale slider, loop checkbox) to a TrajectoryPlayer
    """

    SLIDER_MAX = 1000

    def __init__(self, btn_play, btn_stop, sl_time, lbl_time,
                 sl_scale, chk_loop) -> None:
        self.btn_play = btn_play
        self.btn_stop = btn_stop
        self.sl_time = sl_time
        self.lbl_time = lbl_time
        self.sl_scale = sl_scale
        self.chk_loop = chk_loop
        self.player: Optional[TrajectoryPlayer] = None

        btn_play.clicked.connect(self.on_play)
        btn_stop.clicked.connect(self.on_stop)

        sl_time.setMaximum(self.SLIDER_MAX)
        sl_time.sliderPressed.connect(self.on_scrub_start)
        sl_time.sliderMoved.connect(self.on_scrub_move)
        sl_time.sliderReleased.connect(self.on_scrub_end)

        sl_scale.setRange(10, 200)
        sl_scale.setValue(100)
        sl_scale.valueChanged.connect(self.on_scale)

        chk_loop.toggled.connect(self.on_loop)

    def bind(self, player: Optional[TrajectoryPlayer]) -> None:
        """
        swap in a freshly built player
        """
        if self.player is not None and self.player.playing:
            self.player.pause()
        self.player = player
        if player is not None:
            player.on_tick = self.show_time
            player.on_state = self.on_player_state
            player.scale = self.sl_scale.value() / 100.0
            player.loop = self.chk_loop.isChecked()
        self.btn_play.setEnabled(
            player is not None and player.feasible())
        self.show_time(0.0)

    # ================= playback =================

    def on_play(self) -> None:
        if self.player is None:
            return
        if self.player.playing:
            self.player.pause()
        else:
            self.player.play()

    def on_stop(self) -> None:
        if self.player:
            self.player.stop()

    def on_scrub_start(self) -> None:
        if self.player and self.player.playing:
            self.player.pause()

    def on_scrub_move(self, val: int) -> None:
        if self.player:
            self.player.scrub(self.slider_to_sec(val), dragging=True)

    def on_scrub_end(self) -> None:
        if self.player:
            self.player.scrub(self.slider_to_sec(self.sl_time.value()),
                              dragging=False)

    def slider_to_sec(self, val: int) -> float:
        """
        in: slider position 0..SLIDER_MAX
        out: trajectory sec
        """
        dur = self.player.duration if self.player else 0.0
        return val / self.SLIDER_MAX * dur

    def on_scale(self, val: int) -> None:
        if self.player:
            self.player.scale = val / 100.0

    def on_loop(self, active: bool) -> None:
        if self.player:
            self.player.loop = active

    def on_player_state(self, state: str) -> None:
        self.btn_play.setText("Pause" if state == "playing" else "Play")

    def show_time(self, t_sec: float) -> None:
        dur = self.player.duration if self.player else 0.0
        self.lbl_time.setText(f"{t_sec:.2f} / {dur:.2f} s")
        self.sl_time.blockSignals(True)     # echo guard
        self.sl_time.setValue(
            int(t_sec / dur * self.SLIDER_MAX) if dur > 0 else 0)
        self.sl_time.blockSignals(False)
