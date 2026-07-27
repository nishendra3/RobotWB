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

import FreeCAD as App  # type: ignore
from PySide.QtCore import QTimer  # type: ignore

from freecad.Robot_tools.Gui.rbt_helpers_ui import is_alive

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
        if asm is None:
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
        page = self.w.stack.currentWidget()
        if page is None or page._writing:
            return
        rb = page.robot
        if not is_alive(rb):
            return
        if prop == "Offset2" and obj in rb.Robot_joints:
            QTimer.singleShot(0, page.sync_panel_from_doc)

    def slotCreatedObject(self, o):
        QTimer.singleShot(0, self.w.refresh_picker)

    def slotDeletedObject(self, o):
        name = o.Name
        QTimer.singleShot(0, lambda: self.w.drop_page(name))
