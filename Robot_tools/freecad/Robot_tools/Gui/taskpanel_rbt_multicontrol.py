"""taskpanel_rbt_multicontrol.py — taskpanel for multirobot control"""

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

from PySide import QtCore  # type: ignore
from PySide.QtCore import QTimer  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QWidget, QComboBox, QStackedWidget, QVBoxLayout, QDialogButtonBox,
)

from freecad.Robot_tools.Gui.rbt_helpers_ui import is_alive
from freecad.Robot_tools.Gui.taskpanel_rbt_animate import RobotControlWidget
from freecad.Robot_tools.App.rbt_robot import all_robots, is_robot
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.Gui.rbt_fc_observer import RbtMultiCtrlObserver


def robot_key(rb) -> tuple[str, str]:
    """
    identitifier that stays unique across documents
    """
    return (rb.Document.Name, rb.Name)


class MultiRobotControlPanel:
    """
    unified controller for multi-robots
    """
    def __init__(self) -> None:
        self.form = MultiRobotControlWidget()

    def getStandardButtons(self):
        return QDialogButtonBox.Close

    def reject(self) -> bool:
        """
        commit visible page, detach observer & close
        """
        try:
            page = self.form.stack.currentWidget()
            if page is not None and is_alive(page.robot):
                with page.writing():
                    page.ctrl.commit_joints()
        finally:
            self.form.teardown()
            Gui.Control.closeDialog()
        return True


def run(robot: App.DocumentObject | None = None) -> None:
    """
    opens the multi-robot control panel
    """
    if Gui.Control.activeDialog():
        fcl_warn("close the active task panel first")
        return
    panel = MultiRobotControlPanel()
    Gui.Control.showDialog(panel)
    if robot:
        panel.form.select_robot(robot)


class MultiRobotControlWidget(QWidget):
    """
    Multi-robot wrapper: picker + one RobotControlWidget page per robot
    """
    def __init__(self) -> None:
        super().__init__()
        self.picker = QComboBox()
        self.stack = QStackedWidget()

        lay = QVBoxLayout(self)
        lay.addWidget(self.picker)
        lay.addWidget(self.stack)

        # panel refresh timeout
        self._refresh_timer = QtCore.QTimer(self)
        self._refresh_timer.setSingleShot(True)
        self._refresh_timer.setInterval(50)
        self._refresh_timer.timeout.connect(self.refresh_picker)

        self.picker.currentIndexChanged.connect(self._on_pick)
        self.refresh_picker()

        self._observer = RbtMultiCtrlObserver(self)
        App.addDocumentObserver(self._observer)

    def teardown(self) -> None:
        if self._observer is not None:
            App.removeDocumentObserver(self._observer)
            self._observer = None
        while self.stack.count():
            w = self.stack.widget(0)
            self.stack.removeWidget(w)
            w.deleteLater()

    def closeEvent(self, ev) -> None:
        self.teardown()
        super().closeEvent(ev)

    def current_robot(self):
        return self.picker.currentData()

    def select_robot(self, robot) -> None:
        """
        set picker to robot
        """
        for i in range(self.picker.count()):
            if self.picker.itemData(i) is robot:
                self.picker.setCurrentIndex(i)
                return

    def find_page(self, key: tuple[str, str]):
        """
        stack == single page registry for the widget
        """
        for i in range(self.stack.count()):
            w = self.stack.widget(i)
            if w.page_key == key:
                return w
        return None

    def drop_page(self, key: tuple[str, str]) -> None:
        """
        Deletion hook - remove deleted rob's control widget
        """
        page = self.find_page(key)
        if page is not None:
            self.stack.removeWidget(page)
            page.deleteLater()
        self.refresh_picker()

    def _on_pick(self, _: int) -> None:
        rb = self.current_robot()
        if rb is None or not is_alive(rb):
            return
        page = self.find_page(robot_key(rb))
        if page is None:
            page = RobotControlWidget(rb)
            self.stack.addWidget(page)

        self.stack.setCurrentWidget(page)
        page.sync_panel_from_doc()

        # select the correct robot when
        # another rob is selected from dropdown

        Gui.Selection.clearSelection()
        Gui.Selection.addSelection(rb)

    def refresh_picker(self) -> None:
        prev = self.current_robot()
        prev_key = (robot_key(prev)
                    if prev is not None and is_alive(prev)
                    else None)

        self.picker.blockSignals(True)

        self.picker.clear()
        robs = all_robots()
        for r in robs:
            self.picker.addItem(r.Label, r)
        idx = next((i for i, r in enumerate(robs)
                    if robot_key(r) == prev_key), 0)
        if robs:
            self.picker.setCurrentIndex(idx)

        self.picker.blockSignals(False)

        if not robs:
            self.stack.setCurrentIndex(-1)
            return
        self._on_pick(idx)

    def on_doc_changed(self, obj, prop: str) -> None:
        if prop == "RobotAssembly":
            self._refresh_timer.start()
            return
        page = self.stack.currentWidget()
        if page is not None:
            page.on_doc_changed(obj, prop)

    def on_doc_deleted(self, o) -> None:
        if not is_robot(o):
            return
        key = (o.Document.Name, o.Name)
        QTimer.singleShot(0, lambda: self.drop_page(key))
