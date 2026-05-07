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

import FreeCAD as App
from PySide.QtCore import QTimer


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
        if (d.wk_asm_d is None) or (obj.Document is not d.wk_asm_d):
            return
        if hasattr(obj, "ObjectToGround") or hasattr(obj, "JointType"):
            # deferred call,  d.refresh_joints_panel()
            QTimer.singleShot(0, d.refresh_joints_panel)
