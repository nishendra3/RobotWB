""" Robot_tools Custom TB

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

import os
import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

# service import
from freecad.Robot_tools import rbt_locator

from freecad.Robot_tools.App.rbt_robot import is_robot

wb_path = os.path.dirname(rbt_locator.__file__)


class CommandAnimateRobot:
    """Opens the joint jogging animation panel"""

    def GetResources(self):
        """Resources."""
        return {
            'Pixmap': os.path.join(wb_path,
                                   'resources/icons/rbt_animateRobot.svg'),
            'MenuText': "Animate Robot",
            'ToolTip': "<b>Animate Robot</b>"
                }

    def Activated(self):
        """Activated."""
        from freecad.Robot_tools.Gui import taskpanel_rbt_animate
        taskpanel_rbt_animate.run()

    def IsActive(self):
        """IsActive."""
        return True


class CommandCreateRobot:
    """Opens the robot creator taskpanel"""

    def GetResources(self):
        """Resources."""
        return {
            'Pixmap': os.path.join(wb_path,
                                   'resources/icons/rbt_createRobot.svg'),
            'MenuText': "Define Robot",
            'ToolTip': "<b>Define a Robot from CAD elements</b>"
                }

    def Activated(self):
        """Activated."""
        from freecad.Robot_tools.Gui import taskpanel_rbt_creator
        taskpanel_rbt_creator.run()

    def IsActive(self):
        """IsActive."""
        return not Gui.Control.activeDialog()


class CommandCreateTool:
    """Opens the tool creator taskpanel"""

    def GetResources(self):
        return {"Pixmap":   os.path.join(wb_path,
                                         'resources/icons/rbt_defineTool.svg'),
                "MenuText": "Define Tool",
                "ToolTip":  "Define a Tool and TCP on the active robot"}

    def IsActive(self):
        doc = App.ActiveDocument
        return (bool(doc) and
                not Gui.Control.activeDialog() and
                any(is_robot(o) for o in doc.Objects))

    def Activated(self):
        from freecad.Robot_tools.Gui import taskpanel_rbt_tool
        taskpanel_rbt_tool.run()


class CommandMultiControl:
    """
    opens the multi-robot control panel
    """
    def GetResources(self):
        return {"Pixmap": os.path.join(wb_path,
                                       'resources/icons/rbt_multiControl.svg'),
                "MenuText": "Multi-Robot Control",
                "ToolTip": "Control all robots in the document from one panel"}

    def IsActive(self) -> bool:
        return bool(App.ActiveDocument) and not Gui.Control.activeDialog()

    def Activated(self) -> None:
        from . import taskpanel_rbt_multicontrol
        taskpanel_rbt_multicontrol.run()


class CommandCreateTrajectory:
    """create/open a trajectory for the selected robot"""

    def GetResources(self):
        return {"Pixmap": os.path.join(wb_path,
                                       'resources/icons/rbt_createTraj.svg'),
                "MenuText": "Robot Trajectory",
                "ToolTip": "Teach waypoints and play them as a trajectory"}

    def IsActive(self):
        return App.ActiveDocument is not None

    def Activated(self):
        from freecad.Robot_tools.App.rbt_traj import (
            create_trajectory, is_trajectory)
        from freecad.Robot_tools.Gui import g_rbt_traj_tpanel
        from freecad.Robot_tools.Gui.rbt_helpers_ui import find_rob

        # a selected trajectory just re-opens its panel
        sel = [o for o in Gui.Selection.getSelection() if is_trajectory(o)]
        if sel:
            g_rbt_traj_tpanel.run(sel[0])
            return

        robot = find_rob()          # selected robot | only robot | None
        if robot is None:
            return
        doc = App.ActiveDocument
        doc.openTransaction("Create trajectory")
        try:
            traj = create_trajectory(robot)
            doc.commitTransaction()
        except Exception:
            doc.abortTransaction()
            raise
        g_rbt_traj_tpanel.run(traj)


class CommandTrajectoryPlayer:
    """global player: run several trajectories in sync"""

    def GetResources(self):
        return {"Pixmap": os.path.join(wb_path,
                                       'resources/icons/rbt_multiPanel.svg'),
                "MenuText": "Trajectory Player",
                "ToolTip": "Play trajectories of all robots on one clock"}

    def IsActive(self):
        doc = App.ActiveDocument
        return doc is not None and any(
            getattr(o, "Waypoint_count", 0) > 1 for o in doc.Objects)

    def Activated(self):
        from freecad.Robot_tools.Gui import g_rbt_traj_gplayer
        g_rbt_traj_gplayer.run()


commands = {
    "RBT_anrob": CommandAnimateRobot(),
    "RBT_defrob": CommandCreateRobot(),
    "RBT_deftool": CommandCreateTool(),
    "RBT_multictrl": CommandMultiControl(),
    "RBT_deftraj": CommandCreateTrajectory(),
    "RBT_playtraj": CommandTrajectoryPlayer(),
}

COMMAND_NAMES = list(commands.keys())

for name, command in commands.items():
    Gui.addCommand(name, command)
