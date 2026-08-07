"""
FreeCAD slot event obeserver for Robot UI.

Name: rbt_fc_observer.py

See Changelog below.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""
__version__ = "0.01"
__build__ = "20260507_1255"

from typing import TYPE_CHECKING

import FreeCAD as App  # type: ignore
from PySide.QtCore import QTimer  # type: ignore

from freecad.Robot_tools.Gui.rbt_helpers_ui import is_alive
from freecad.Robot_tools.App.rbt_traj_types import DocObj
if TYPE_CHECKING:
    from freecad.Robot_tools.Gui.g_rbt_traj_ctrl_wgt import (
        TrajectoryControlWidget)

"""
----------------------------------------
Changelog:
----------------------------------------
v0.01 - Initial version.
"""


class RbtObserver:
    """Observer to handle panel refresh on modifications"""

    def __init__(self, dialog):
        self.dialog = dialog
        App.addDocumentObserver(self)

    def stop(self):
        App.removeDocumentObserver(self)

    def slotDeletedObject(self, obj):
        """
        Refresh the panel after joint deletion
        """
        d = self.dialog
        if d is None or getattr(d, "assembly_doc", None) is None:
            return
        if obj.Document is not d.assembly_doc:
            return

        asm = getattr(d.creator, "assembly", None)
        if asm is None or not is_alive(asm):
            # nothing built or connected yet
            return

        if obj is not asm and obj not in asm.OutListRecursive:
            # deletion belongs to another robot
            return

        is_joint = hasattr(obj, "ObjectToGround") or hasattr(obj, "JointType")
        link_nm = obj.Name if obj.isDerivedFrom("App::Link") else None
        QTimer.singleShot(0, lambda: d.on_obj_deleted(is_joint, link_nm))

    def slotDeletedDocument(self, doc):
        """
        close the panel when working doc is closed
        """
        d = self.dialog
        if d is None or doc is not getattr(d, "doc", None):
            return
        QTimer.singleShot(0, d.on_doc_closed)


class RbtSelectionObserver:
    """
    refreshes the 3D selection for faces for joint creation
    in the robot creator panel
    """

    def __init__(self, dialog):
        self.dialog = dialog

    def on_changed(self, *_):
        QTimer.singleShot(0, self.dialog.refresh_pending_faces)

    def addSelection(self, *a):
        self.on_changed(*a)

    def clearSelection(self, *a):
        self.on_changed(*a)

    def removeSelection(self, *a):
        self.on_changed(*a)


class RbtMultiCtrlObserver:
    """
    Mirror external joint/base changes into the panel
    """
    def __init__(self, w): self.w = w      # MultiRobotControlWidget

    def slotChangedObject(self, obj, prop: str) -> None:
        try:
            self.w.on_doc_changed(obj, prop)
        except RuntimeError:
            App.removeDocumentObserver(self)
        except Exception:
            pass

    def slotDeletedObject(self, o) -> None:
        try:
            self.w.on_doc_deleted(o)
        except RuntimeError:
            App.removeDocumentObserver(self)
        except Exception:
            pass


class TrajPickObserver:
    """
    Selection observer for the trajectory panel's 'Pick 3D' mode.
    """
    def __init__(self, on_pick) -> None:
        self.on_pick = on_pick

    def addSelection(self, doc, obj, sub, pos) -> None:
        if pos == (0.0, 0.0, 0.0):
            return   # tree click, no 3d point
        self.on_pick(App.Vector(*pos))


class TrajDocObserver:
    """
    mirror trajectory property changes into the traj panel
    """
    def __init__(self, w: "TrajectoryControlWidget") -> None:
        self.w = w

    def slotChangedObject(self, obj: DocObj, prop: str) -> None:
        try:
            self.w.on_doc_changed(obj, prop)
        except RuntimeError:
            App.removeDocumentObserver(self)
