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
