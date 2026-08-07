"""
vp_rbt_robot.py - view provider for Robot FreeCAD python object (FPO)
"""

from pivy import coin  # type: ignore

import FreeCADGui as Gui  # type: ignore
from freecad.Robot_tools.App.rbt_placement import base_link
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err


class ViewProviderRobot:

    def __init__(self, obj):
        """
        Set this object to the proxy object of the actual view provider
        """
        obj.Proxy = self

    def attach(self, obj):
        """
        Setup the scene sub-graph of the view provider,
        this method is mandatory
        """
        self.ViewObject = obj
        self.Object = obj.Object
        self.standard = coin.SoGroup()
        obj.addDisplayMode(self.standard, "Standard")
        return

    def updateData(self, fp, prop):
        """
        If a property of the handled feature has changed.
          we have the chance to handle this here
        """
        return

    def getDisplayModes(self, obj):
        """
        Return a list of display modes.
        """
        return ["Standard"]

    def getDefaultDisplayMode(self):
        """
        Return the name of the default display mode.
          It must be defined in getDisplayModes.
        """
        return "Standard"

    def claimChildren(self):
        """
        populates the sub-elements (links, tools etc)
        under the robot fpo
        """
        obj = self.Object
        kids = []
        if getattr(obj, "Robot_assembly", None):
            kids.append(obj.Robot_assembly)
        kids.extend(getattr(obj, "Tools", []) or [])
        kids.extend(getattr(obj, "Trajectories", []) or [])
        return kids

    def doubleClicked(self, vobj):
        from freecad.Robot_tools.Gui import taskpanel_rbt_animate
        taskpanel_rbt_animate.run(vobj.Object)
        return True

    def setEdit(self, vobj, mode=0):
        return self.doubleClicked(vobj)

    def unsetEdit(self, vobj, mode=0):
        Gui.Control.closeDialog()
        return True

    def setupContextMenu(self, vobj, menu):
        act = menu.addAction("Place Robot (3D)")
        act.triggered.connect(lambda:
                              start_rbt_placement(vobj.Object))

    def onChanged(self, vp, prop):
        """

        """
        pass

    def dumps(self):
        """
        Called during document saving.
        """
        return None

    def loads(self, state):
        """
        Called during document restore.
        """
        return None


# robot placement
def start_rbt_placement(robot):
    """
    Using native transform dragger on the assembly
    """
    asm, bl = getattr(robot, "Robot_assembly", None), base_link(robot)

    if asm is None or bl is None:
        fcl_err("robot has no assembly or base link to move")
        return

    # another dialog open
    if Gui.Control.activeDialog():
        return

    doc = Gui.getDocument(robot.Document.Name)
    if doc.getInEdit():
        doc.resetEdit()

    # anchor the gizmo on base frame
    asm.ViewObject.TransformOrigin = bl.Placement.multiply(robot.Base_offset)
    doc.setEdit(asm.Name, 1)  # 1 = Transform mode
