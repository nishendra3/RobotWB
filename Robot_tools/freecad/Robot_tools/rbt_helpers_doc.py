"""Robot app document helper functions.

Name: rbt_helpers_doc.py

See Changelog below.

Author: Nishendra Singh
Copyright: 2026
Licence: All right reserved
"""
__version__ = "0.01"
__build__ = "20260505_0828"

import FreeCAD as App
import FreeCADGui as Gui

fcl_err = App.Console.PrintError
fcl_msg = App.Console.PrintMessage
fcl_warn = App.Console.PrintWarning

V3 = App.Vector
Rotation = App.Rotation
Placement = App.Placement

def clear_doc(doc_name):
    """Clear the document deleting all the objects.

    Args:
    doc_name  (str): document name
    """
    doc = App.getDocument(doc_name)
    try:
        while len(doc.Objects) > 0:
            doc.removeObject(doc.Objects[0].Name)
    except Exception as e:
        fcl_msg(f"Exception:  {e}\n")


def ensure_document(doc_name, action=0, dbg_l=0):
    """Ensure existence of document with doc_name and clear the doc.

    Args:
        doc_name (string): document name
        action (int): clear action if document exist
              0: do nothing
              1: close and reopen the document with the same name
              2: delete all objects
        dbg_l (int):

    Returns:
        doc_name (string): Document name from obj.Name
    """
    doc_root = App.listDocuments()
    doc_exist = False

    for d_name, doc in doc_root.items():

        if dbg_l > 0:
            fcl_msg(f"ED: doc name = {d_name}\n")

        if d_name == doc_name:
            if dbg_l > 0:
                fcl_msg(f"ED: Match name: {doc_name} action: {action}\n")

            if action == 1:
                # when using clear_doc() is not possible like in Part.Design
                App.closeDocument(doc_name)
                doc_exist = False
            elif action == 2:
                # debug info do not delete
                # print("action 2")  # kp
                doc_exist = True
                clear_doc(doc_name)
                App.setActiveDocument(d_name)
                return App.getDocument(d_name).Name
            else:
                doc_exist = True
                App.setActiveDocument(d_name)
                return App.getDocument(d_name).Name

    if doc_exist is False:
        if dbg_l > 0:
            fcl_msg(f"ED: Create = {doc_name}\n")

        new_doc = App.newDocument(doc_name)
        return new_doc.Name


def setview(doc_name, t_v=0):
    """Set viewport in 3D view."""
    App.setActiveDocument("")
    App.ActiveDocument = None
    Gui.ActiveDocument = None

    switch_document(doc_name)
    VIEW = Gui.ActiveDocument.ActiveView

    if t_v == 0:
        VIEW.viewTop()
    elif t_v == 1:
        VIEW.viewFront()
    elif t_v == 99:
        VIEW.setCameraOrientation(
            Rotation(0.0, 0.0, -0.7071067811865475, 0.7071067811865475))

    else:
        VIEW.viewAxometric()

    VIEW.setAxisCross(True)
    VIEW.fitAll()
    # Gui.updateGui()


def switch_document(doc_name):
    """Switch a FreeCAD document."""
    App.setActiveDocument(doc_name)
    App.ActiveDocument = App.getDocument(doc_name)
    Gui.ActiveDocument = Gui.getDocument(doc_name)
    #  Trick to swith the Gui to show the document
    gv = Gui.ActiveDocument.ActiveView.graphicsView()
    pw = gv.parentWidget().parentWidget().parentWidget()
    Gui.getMainWindow().centralWidget().setActiveSubWindow(pw)
