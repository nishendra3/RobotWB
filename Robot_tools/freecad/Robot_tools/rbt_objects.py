"""Robot Objects.

Name: rbt_objects.py

See Changelog below.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""
__version__ = "0.02"
__build__ = "20260507_1256"

import FreeCADGui as Gui
import FreeCAD as App

# Coin3d import
from pivy import coin

"""
----------------------------------------
Changelog:
----------------------------------------
v0.01 - Initial version.
v0.02 - Added the STEPFile property.
"""

fcl_err = App.Console.PrintError
fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning

# ------------------------------------------------
#                   Robot Objects
# ------------------------------------------------


class Robot_obj:
    def __init__(self, obj):
        '''Add some custom properties to our box feature'''
        obj.addProperty(
            "App::PropertyLink", "Robot_assembly", "Robot", "Robot_assembly")
        obj.addProperty(
            "App::PropertyLinkListGlobal", "Robot_joints", "Robot", "Robot joints list")
        obj.addProperty(
            "App::PropertyPlacementList", "Robot_links", "Robot", "Robot link lists")
        obj.addProperty(
            "App::PropertyIntegerList", "Robot_joints_dir", "Robot",
            "Robot joints direction")
        obj.addProperty(
            "App::PropertyLinkListGlobal", "Tools_joints", "Tools", "Tools joints list")
        # TODO: this is useful in case of unfinished editing
        obj.addProperty(
            "App::PropertyFile", "STEPFile", "General",
            "File from where elements have been loaded.")

        obj.Proxy = self

    def onChanged(self, fp, prop):
        '''Do something when a property has changed'''
        fcl_msg("Change property: " + str(prop) + "\n")

    def execute(self, fp):
        '''Do something when doing a recomputation, this method is mandatory'''
        fcl_msg("Execute reached\n")
        #
        fp.recompute()


class ViewProviderRBo:

    def __init__(self, obj):
        """
        Set this object to the proxy object of the actual view provider
        """

        obj.Proxy = self

    def attach(self, obj):
        """
        Setup the scene sub-graph of the view provider, this method is mandatory
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

    def onChanged(self, vp, prop):
        """
        Print the name of the property that has changed
        """

        App.Console.PrintMessage("Change property: " + str(prop) + "\n")

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
