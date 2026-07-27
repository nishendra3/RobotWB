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
        return bool(doc) and any(is_robot(o) for o in doc.Objects)

    def Activated(self):
        from freecad.Robot_tools.Gui import taskpanel_rbt_tool
        taskpanel_rbt_tool.run()


class CommandMultiControl:
    """
    opens the multi-robot control panel
    """
    def GetResources(self):
        return {"Pixmap": os.path.join(wb_path,
                                       'resources/icons/rbt_animateRobot.svg'),
                "MenuText": "Multi-Robot Control",
                "ToolTip": "Control all robots in the document from one panel"}

    def IsActive(self) -> bool:
        return bool(App.ActiveDocument) and not Gui.Control.activeDialog()

    def Activated(self) -> None:
        from . import taskpanel_rbt_multicontrol
        taskpanel_rbt_multicontrol.run()


commands = {
    "RBT_anrob": CommandAnimateRobot(),
    "RBT_defrob": CommandCreateRobot(),
    "RBT_deftool": CommandCreateTool(),
    "RBT_multictrl": CommandMultiControl(),
}

COMMAND_NAMES = list(commands.keys())

for name, command in commands.items():
    Gui.addCommand(name, command)
