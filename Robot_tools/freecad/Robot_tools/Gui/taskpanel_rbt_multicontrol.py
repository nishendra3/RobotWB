"""taskpanel_rbt_multicontrol.py — taskpanel for multirobot control"""

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

from PySide import QtCore  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QWidget, QComboBox, QStackedWidget, QVBoxLayout, QDialogButtonBox,
)

from freecad.Robot_tools.Gui.rbt_helpers_ui import is_alive
from freecad.Robot_tools.Gui.taskpanel_rbt_animate import RobotControlWidget
from freecad.Robot_tools.App.rbt_robot import all_robots
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.Gui.rbt_fc_observer import RbtMultiCtrlObserver

class MultiRobotControlPanel:
    """
    unified controller for multi-robots
    """
    def __init__(self) -> None:
        self.form = MultiRobotControlWidget()

    def getStandardButtons(self):
        return QDialogButtonBox.Close

    def reject(self) -> None:
        """
        commit visible page, detach observer & close
        """
        page = self.form.stack.currentWidget()
        if page is not None:
            with page.writing():
                page.ctrl.commit_joints()
        self.form.teardown()
        Gui.Control.closeDialog()


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

        self.picker.currentIndexChanged.connect(self._on_pick)
        self.refresh_picker()

        self._observer = RbtMultiCtrlObserver(self)
        App.addDocumentObserver(self._observer)

    def teardown(self) -> None:
        App.removeDocumentObserver(self._observer)

    def current_robot(self):
        return self.picker.currentData()

    def select_robot(self, robot) -> None:
        """
        set picker to robot
        """
        i = self.picker.findData(robot)
        if i >= 0:
            self.picker.setCurrentIndex(i)

    def find_page(self, name: str):
        """
        stack == single page registry for the widget
        """
        for i in range(self.stack.count()):
            w = self.stack.widget(i)
            if w.robot_name == name:
                return w
        return None

    def drop_page(self, name: str) -> None:
        """
        Deletion hook - remove deleted rob's control widget
        """
        page = self.find_page(name)
        if page is not None:
            self.stack.removeWidget(page)
            page.deleteLater()
        self.refresh_picker()

    def _on_pick(self, _: int) -> None:
        rb = self.current_robot()
        if rb is None:
            return
        page = self.find_page(rb.Name)
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
        """
        refresh the picker panel
        """
        prev = self.picker.currentText()
        self.picker.blockSignals(True)
        self.picker.clear()
        for r in all_robots():
            self.picker.addItem(r.Label, r)
        self.picker.blockSignals(False)
        i = self.picker.findText(prev)
        self.picker.setCurrentIndex(max(i, 0))
        self._on_pick(0)
