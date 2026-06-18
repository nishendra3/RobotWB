"""Robot Objects.

Name: rbt_objects.py

See Changelog below.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""
__version__ = "0.03"
__build__ = "20260508_1337"

import FreeCADGui as Gui
import FreeCAD as App
from freecad.Robot_tools.App.rbt_kine import invalidate

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
            "App::PropertyLinkList", "Robot_joints", "Robot", "Robot joints list")
        obj.addProperty(
            "App::PropertyPlacementList", "Robot_links", "Robot", "Robot link lists")
        obj.addProperty(
            "App::PropertyIntegerList", "Robot_joints_dir", "Robot",
            "Robot joints direction")
        obj.addProperty(
            "App::PropertyLinkList", "Tools", "Tools",
            "Tool FPOs attached to the robot")
        obj.addProperty(
           "App::PropertyLink", "Active_tool", "Tools", "Currently active tool")
        # TODO: this is useful in case of unfinished editing
        obj.addProperty(
            "App::PropertyFile", "STEPFile", "General",
            "File from where elements have been loaded.")
        # Added a Version property
        obj.addProperty(
            "App::PropertyString", "Version", "Base",
            "Object version")
        obj.Version = __version__  # "0.01"

        obj.addProperty(
            "App::PropertyFloatList", "Robot_home_pos", "Robot",
            "Robot home position angles")

        obj.addProperty(
            "App::PropertyEnumeration", "Kinematics_lib", "Kinematics",
            "Solver to use for FK/IK")
        # add more in future
        obj.Kinematics_lib = ["pinocchio", "tesseract", "ikpy", "numpy_dls"]
        obj.Kinematics_lib = "numpy_dls"

        obj.addProperty(
            "App::PropertyFloatList", "Last_q", "Kinematics",
            "last valid robot joints (deg) used as seed for next IK")

        obj.Proxy = self

    def _migrate_from_001(self, obj):
        """Migrate from version 0.01"""
        fcl_msg("Migrating FPO to v0.02\n")  # DBG

    def _migrate_from_002(self, obj):
        """Migrate from version 0.02"""
        fcl_msg("Migrating FPO to v0.03\n")  # DBG

    def onChanged(self, fp, prop):
        '''Do something when a property has changed'''
        fcl_msg("Change property: " + str(prop) + "\n")
        if prop in ("Robot_joints", "Robot_joints_dir", "Active_tool",
                "Kinematics_lib"):
            try:
                # TODO: verify if invalidation is really needed
                invalidate(fp)
            except Exception:
                pass

    def onDocumentRestored(self, obj):
        if hasattr(obj, "Version") and obj.Version:
            fcl_msg("FPO version check\n")  # DBG
            if obj.Version == "0.01":
                fcl_msg("FPO is already at v0.01\n")  # DBG
                pass  # do nothing
            if obj.Version == "0.02":
                self._migrate_from_001(obj)
            elif obj.Version == "0.03":
                self._migrate_from_002(obj)
        else:
            fcl_msg("FPO has no version bumping to v0.01\n")  # DBG
            obj.addProperty(
                "App::PropertyString", "Version", "Base",
                "Object version")
            obj.Version = "0.01"

        # when restoring the document, create a default tool
        # if no active tool is found. TODO: Check if this is
        # really necessary or not
        if (App.GuiUp and obj.Robot_joints
                and obj.Active_tool is None):

            from PySide.QtCode import QTimer  # type: ignore

            def _mk(o=obj):
                if getattr(o, "Active_tool", None) is None:

                    from freecad.Robot_tools.Gui.define_tool \
                        import create_default_tool

                    create_default_tool(o, name="Default_Tool")

            QTimer.singleShot(0, _mk)

        # -------------------------------------------------
        #   add missing props for back compatability
        # -------------------------------------------------

        if not hasattr(obj, "Robot_home_pos"):
            obj.addProperty(
                "App::PropertyFloatList", "Robot_home_pos", "Robot",
                "Robot home position angles")
        if not hasattr(obj, "Tools"):
            obj.addProperty(
                "App::PropertyLinkList", "Tools", "Tools",
                "Tool FPOs attached to the robot")
        if not hasattr(obj, "Active_tool"):
            obj.addProperty(
                "App::PropertyLink", "Active_tool", "Tools",
                "Currently active tool")
        if not hasattr(obj, "Kinematics_lib"):
            obj.addProperty(
                "App::PropertyEnumeration", "Kinematics_lib", "Kinematics",
                "Solver to use for FK/IK")
            obj.Kinematics_lib = ["pinocchio", "tesseract", "ikpy", "numpy_dls"]
            obj.Kinematics_lib = "numpy_dls"
        elif hasattr(obj, "Kinematics_lib") and (len(obj.Kinematics_lib) < 4):
            obj.Kinematics_lib = ["pinocchio", "tesseract", "ikpy", "numpy_dls"]
            obj.Kinematics_lib = "numpy_dls"

        if not hasattr(obj, "Last_q"):
            obj.addProperty(
                "App::PropertyFloatList", "Last_q", "Kinematics",
                "last valid robot joints (deg) used as seed for next IK")

    def execute(self, fp):
        '''Do something when doing a recomputation, this method is mandatory'''

        pass
        # fcl_msg("Execute reached\n")  # DBG
        #
        # if (App.GuiUp
        #         and fp.Robot_joints
        #         and fp.Active_tool is None):

        #     from freecad.Robot_tools.Gui.define_tool import create_default_tool
        #     create_default_tool(fp, name="Default_Tool")

        # fp.recompute()


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

    def claimChildren(self):
        """
        populates the sub-elements (links, tools etc)
        under the robot fpo
        """
        obj = self.Object
        kids = []
        if getattr(obj, "Robot_assembly", None):
            kids.append(obj.Robot_assembly)
        # kids.extend(getattr(obj, "Robot_joints", []) or [])
        kids.extend(getattr(obj, "Tools", []) or [])
        return kids

    def onChanged(self, vp, prop):
        """

        """
        fcl_msg("Change property: " + str(prop))

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
