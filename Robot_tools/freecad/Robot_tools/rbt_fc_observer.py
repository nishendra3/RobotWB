"""
FreeCAD slot event obeserver for Robot UI.

Name: rbt_fc_observer.py

See Changelog below.

Author: Nishendra Singh
Copyright: 2026
Licence: All right reserved
"""
__version__ = "0.01"
__build__ = "20260506_0736"

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
            QTimer.singleShot(0, d.refresh_joints_panel) #deferred call,  d.refresh_joints_panel()

