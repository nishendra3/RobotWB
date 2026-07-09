"""Create new robot assembly from FreeCAD parts"""

import FreeCAD as App   # type: ignore
import FreeCADGui as Gui  # type: ignore

from enum import IntEnum

from PySide.QtCore import Qt  # type: ignore
from PySide.QtWidgets import (  # type: ignore
    QInputDialog, QTableWidgetItem, QPushButton,
    QTreeWidgetItem, QHeaderView)

from freecad.Robot_tools.App.rbt_creator import RobotCreator
from freecad.Robot_tools.Gui.rbt_fc_observer import (
    RbtObserver, RbtSelectionObserver)
from freecad.Robot_tools.App.rbt_helpers_log import fcl_warn
from freecad.Robot_tools.Gui.rbt_helpers_ui import (
    load_panel_ui, get_file, msg_box, set_txt_color)

# --------------------------------------------------------------


class CreationStep(IntEnum):
    IMPORT_PARTS = 0
    CREATE_ASSEMBLY = 1
    ADD_JOINTS = 2
    REVIEW = 3


CREATION_STEPS = {
    CreationStep.IMPORT_PARTS:
        "Step 1 of 4 : Import components",
    CreationStep.CREATE_ASSEMBLY:
        "Step 2 of 4 : Insert parts",
    CreationStep.ADD_JOINTS:
        "Step 3 of 4 : Define joints",
    CreationStep.REVIEW:
        "Step 4 of 4 : Review & finish",
}

GUIDANCE = {
    CreationStep.IMPORT_PARTS:
        "Pick the .FCStd that holds the robot's part bodies",
    CreationStep.CREATE_ASSEMBLY:
        "left-click on a part to insert it "
        "|| right-click to remove inserted part",

    # CreationStep.ADD_JOINTS:
    #    => built dynamically later

    CreationStep.REVIEW:
        "review the joints. 'Finish' commits & 'Cancel' discards everything",
}

SHOWN_FC_TYPES = ("Part::Feature", "App::Part", "App::DocumentObjectGroup")


def _is_shown(obj):
    return any(obj.isDerivedFrom(t) for t in SHOWN_FC_TYPES)


def _is_container(obj):
    return (obj.isDerivedFrom("App::Part") or
            obj.isDerivedFrom("App::DocumentObjectGroup"))


def _is_insertable(obj):
    return (_is_shown(obj) and not
            obj.isDerivedFrom("App::DocumentObjectGroup"))
# --------------------------------------------------------------


class DefineRobot:
    """
    TaskPanel for building a robot from CAD parts
    """
    def __init__(self):
        self.creator = RobotCreator()
        self.doc, self.doc_owner = self.pick_working_doc()
        self.imported_names = set()
        self.doc.openTransaction("Create Robot")
        try:
            # UI state variables for joints
            self.joint_index = 0  # which joint we are on
            self.face_slot = 1  # 1/2 : which face we are picking for joint
            self.pending_faces = {}  # metadata of the selected faces

            self.doc_observer = None
            self.selection_observer = None

            self.form = self.cm_form()
            self.connect()

            self.install_observer()
            self.curr_step = CreationStep.IMPORT_PARTS

            if self.creator.candidate_assemblies():
                # case: reopened existing robot file
                self.check_asm()
                self.joint_index = self.creator.next_joint_index()
                self.curr_step = (CreationStep.ADD_JOINTS
                                  if self.creator.part_count()
                                  else CreationStep.CREATE_ASSEMBLY)

            self.set_curr_step(self.curr_step)
        except Exception:
            self.doc.abortTransaction()
            self.teardown_observer()
            raise

    @property
    def assembly_doc(self):
        return self.creator.asm_doc

    def pick_working_doc(self):
        """
        fresh builds go into a new doc & we
        reuse the doc when robot fpo pre-exists
        or the doc is empty (no Objects present)
        """
        doc = App.ActiveDocument

        # reuse case : resuming rbt file of empty doc
        if doc is not None:
            self.creator.asm_doc = doc
            # check if the doc contains is rbt or empty
            if (self.creator.candidate_assemblies()
                    or not doc.Objects):
                return doc, False

        # fresh build case
        doc = App.newDocument("Robot")
        self.creator.asm_doc = doc
        return doc, True

    def cm_form(self):
        w = load_panel_ui("taskpanel_rbt_creator.ui")
        self.imported_parts_file = w.importPartsEdit
        self.grounded_check = w.groundedJointCheck
        self.joints_table = w.jointsTable
        self.status_label = w.statusLabel

        # parts import from components file
        self.parts_list = w.partsList
        self.parts_filter = w.partsFilterEdit
        self.setup_joints_table_header()

        return w

    def connect(self):
        """
        Connect the buttons to function callbacks
        """
        f = self.form
        connections = {
            f.importPartsButton: self.on_import_parts,
            f.selectFace1Button: self.on_select_face,
            f.selectFace2Button: self.on_select_face,
            f.addJointButton: self.on_add_joint,
            f.backButton: self.on_back,
            f.nextButton: self.on_next,
            f.clearFacesButton: self.on_clear_faces,
        }

        for button, slot in connections.items():
            button.clicked.connect(slot)

        self.parts_list.itemClicked.connect(self.on_parts_clicked)
        self.parts_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.parts_list.customContextMenuRequested.connect(
            self.on_parts_right_clicked)

        self.parts_filter.textChanged.connect(self.on_parts_filter)

    def install_observer(self):
        """
        adds the selection observer
        """
        if self.doc_observer is None:
            self.doc_observer = RbtObserver(self)

        if self.selection_observer is None:
            self.selection_observer = RbtSelectionObserver(self)
            Gui.Selection.addObserver(self.selection_observer)

    # file pickers
    def on_import_parts(self):
        f_name = get_file(self.form, self.form.font(), ftype="fcstd")
        if not f_name:
            return

        before = {o.Name for o in self.doc.Objects}
        self.doc.mergeProject(f_name)
        self.doc.recompute()

        self.imported_names |= self.creator.track_imported(before)
        self.imported_parts_file.setText("Parts: "+str(f_name))

        self.group_imported()

        # Auto-advance on componenet file selection
        self.set_curr_step(CreationStep.CREATE_ASSEMBLY)

    def group_imported(self):
        """Stash merged source parts in a hidden 'Elements' group."""

        objs = [
            obj
            for obj in (
                self.doc.getObject(name)
                for name in self.imported_names
            )
            if obj is not None
            and not obj.isDerivedFrom("App::DocumentObjectGroup")
        ]

        roots = [
            obj
            for obj in objs
            if not any(
                parent.Name in self.imported_names
                for parent in obj.InList
            )
        ]

        grp = self.doc.getObject("Elements")
        if grp is None:
            grp = self.doc.addObject(
                "App::DocumentObjectGroup",
                "Elements",
            )
            grp.Label = "Elements"
            self.imported_names.add(grp.Name)  # so reject() cleans it up too

        grp.Group = list(grp.Group) + [
            obj
            for obj in roots
            if obj not in grp.Group
        ]

        for obj in roots:
            obj.Visibility = False

        grp.Visibility = False

    # assembly wb helpers

    def check_asm(self):
        """
        Resolve assembly in creator.asm_doc and sync UI
        """
        how = self.creator.resolve()
        if how is None:
            return self.disambiguate_asm()
        if self.creator.fpo is None:
            msg_box(self.form, " ", self.form.font(),
                    "No Robot_FPO in document.")
            return
        if how != "fpo":
            # TODO: Clear this part's usage
            fcl_warn(f"Assm resolved via '{how}, FPO relinked")

        self.refresh_status()
        self.refresh_joints_table()

    def disambiguate_asm(self):
        """
        If multiple assms are open, ask the user to select
        the correct one
        """
        cands = self.creator.candidate_assemblies()
        if not cands:
            msg_box(self.form, " ", self.form.font(),
                    "No robot assembly in document.")
            return

        items = [f"{a.Label}({a.Name}): {self.creator.part_count()} part(s)"
                 for a in cands]

        choice, ok = QInputDialog.getItem(self.form,
                                          "Select working assembly for robot",
                                          "Multiple found:", items, 0, False)
        if ok:
            self.creator.bind(cands[items.index(choice)])
            self.refresh_status()
            self.refresh_joints_table()

    def ensure_asm(self):
        """
        Idempotently create the robot assembly
        """
        if self.creator.assembly is not None:
            return
        asm = self.creator.build_assembly(self.doc)
        Gui.getDocument(
            self.doc.Name).ActiveView.setActiveObject("assembly", asm)

    def on_parts_filter(self, text):
        query = (text or "").strip().lower()
        lst = self.parts_list
        for i in range(lst.topLevelItemCount()):
            self.filter_item(lst.topLevelItem(i), query)

    def filter_item(self, item, query):
        match = query in item.text(0).lower()
        for i in range(item.childCount()):
            match = self.filter_item(item.child(i), query) or match
        item.setHidden(not match)
        return match

    def on_parts_clicked(self, item):
        obj = item.data(0, Qt.UserRole)
        if obj is None:
            item.setExpanded(not item.isExpanded())
            return
        self.ensure_asm()
        links = self.creator.insert_parts([obj]) or []
        for link in links:
            self.place_link(link)
            Gui.Selection.clearSelection()
            Gui.Selection.addSelection(self.doc.Name, link.Name, "")
        item.setSelected(False)
        if links:
            self.update_ui()
            self.fit_view()

    def on_parts_right_clicked(self, pos):
        # remove the inserted part
        item = self.parts_list.itemAt(pos)
        obj = item.data(0, Qt.UserRole) if item else None
        link = self.creator.curr_last_link(obj) if obj is not None else None
        if link is None:
            return
        if any(f["obj"] is link
               for f in self.pending_faces.values()):
            # clear if the removed link has any face picks
            self.pending_faces = {}
            self.face_slot = 1
        Gui.Selection.clearSelection()
        self.creator.remove_link(link)
        self.update_ui()

    def place_link(self, link):
        """
        Insert new link in the style of FreeCAD
        """
        view = Gui.activeView()
        if view is None or not hasattr(view, "getPointOnFocalPlane"):
            return
        x, y = view.getSize()
        center = view.getPointOnFocalPlane(x//2, y//2)
        corner = view.getPointOnFocalPlane(x, y)
        translation = self.creator.stack_translation(link, center, corner)
        ox, oy = view.getPointOnViewport(App.Vector() + translation)
        if 0 < ox < x and 0 < oy < y:
            link.Placement.Base = self.creator.total_translation
        else:
            bbc = link.ViewObject.getBoundingBox().Center
            link.Placement.Base = center - bbc + self.creator.total_translation

    def fit_view(self):
        """
        run fit-all command after insertion or deletion of a part
        """
        view = Gui.activeView()
        if view is not None and hasattr(view, "fitAll"):
            view.fitAll()

    # joints
    def on_select_face(self):
        raw = Gui.Selection.getSelectionEx("", 0)
        if not raw:
            self.refresh_guidance(
                "Select a face in the 3D view first", "e")
            return

        # select the first pick in case of multiple selections
        sel = raw[0]

        obj, sub = sel.Object, sel.SubElementNames[0]
        link_obj, face_ref = self.creator.resolve_face(obj, sub)
        if link_obj is None:
            owner = self.creator.assembly_owner(obj, sub)
            rob_asm = getattr(self.creator.fpo, "Robot_assembly", None)
            if owner is None or owner is not rob_asm:
                # selected face belongs to another assembly
                # TODO: promt the user to switch to current assembly ?
                self.refresh_guidance(
                    "Face is not in the active robot assembly", "e")
                return

            self.creator.bind(owner)
            link_obj, face_ref = self.creator.resolve_face(obj, sub)

        key = f"face{self.joint_index:02d}{self.face_slot:02d}"
        self.pending_faces[key] = {"obj": link_obj,
                                   "ref": face_ref}
        if self.face_slot == 1:
            self.face_slot = 2  # switch to second face selection

        self.update_ui()

    def on_add_joint(self):
        if not self.pending_faces:
            self.refresh_guidance(
                    "Pick face(s) before adding a joint", "e")
            return

        faces = list(self.pending_faces.values())
        if len(faces) == 1:
            if not self.grounded_check.isChecked():
                self.refresh_guidance(
                    "Two faces are required for a revolute joint", "e")
                return

            # add grounded joint (fixed base joint)
            self.creator.insert_joint("grounded",
                                      [(faces[0]["obj"],
                                        faces[0]["ref"])])
        else:
            # check if faces belong to different links
            if faces[0]["obj"] is faces[1]["obj"]:
                self.refresh_guidance("Pick faces on two different links", "e")
                return
            # add revolute joint
            refs = [(faces[0]["obj"], faces[0]["ref"]),
                    (faces[1]["obj"], faces[1]["ref"])]
            self.creator.insert_joint("revolute",
                                      refs,
                                      label=f"rb_jnt{self.joint_index:02d}")
            self.joint_index += 1

            # TODO: extend to prismatic & fixed joints

        self.refresh_joints_panel()

    # refresh states / panels

    def refresh_joints_panel(self):
        """
        reset the pending face pick and resync the panel
        """
        self.pending_faces = {}
        self.face_slot = 1
        self.update_ui()

    def refresh_status(self):
        a = self.creator.assembly
        lbl_txt = ""
        if a is None:
            lbl_txt = "No working assembly"
        else:
            lbl_txt = f"{a.Label}({a.Name}):\
                    {self.creator.part_count()} part(s)"
        self.status_label.setText(lbl_txt)

    def refresh_joints_table(self):
        t = self.joints_table
        t.setRowCount(0)
        asm = self.creator.assembly
        if asm is None:
            return

        grounded = self.creator.grounded_joint()
        self.grounded_check.setEnabled(grounded is None)
        self.grounded_check.setChecked(grounded is None)
        if grounded is not None:
            self.add_joint_row(["0", "grounded", "--", "--"],
                               grounded,
                               True)

        for idx, j in enumerate(asm.Joints):
            if j.Label2[:6] != "rb_jnt":
                continue
            r1 = f"{j.Reference1[0].Name}.{j.Reference1[1][0]}"
            r2 = f"{j.Reference2[0].Name}.{j.Reference2[1][0]}"
            self.add_joint_row([str(idx+1), str(j.JointType), r1, r2],
                               j,
                               False)

    def refresh_available_parts(self):
        self.parts_list.clear()
        for obj in self.part_root_objs():
            self.add_part_node(obj,
                               self.parts_list.invisibleRootItem())

        self.parts_list.expandAll()
        self.on_parts_filter(self.parts_filter.text())

    def part_root_objs(self):
        """top-level imported objects from component file"""
        roots = Gui.getDocument(self.doc.Name).TreeRootObjects
        return [o for o in roots
                if _is_shown(o) and not
                o.isDerivedFrom("Assembly::AssemblyObject")]

    def add_part_node(self, obj, parent):
        """add object as a node and see its children parts"""
        if not _is_shown(obj):
            return
        node = self.make_part_item(obj, parent)
        if _is_container(obj):
            for child in obj.ViewObject.claimChildren():
                self.add_part_node(child, node)

    def make_part_item(self, obj, parent):
        node = QTreeWidgetItem(parent, [self.part_label(obj)])
        node.setToolTip(0, f"{obj.Label}({obj.Name})")
        if _is_insertable(obj):
            node.setData(0, Qt.UserRole, obj)
        return node

    def part_label(self, obj):
        n = self.creator.link_count(obj)
        return obj.Label + (f" : {n} inserted" if n else "")

    def setup_joints_table_header(self):
        t = self.joints_table
        h = t.horizontalHeader()
        for c in (0, 1, 4, 5):
            h.setSectionResizeMode(c, QHeaderView.ResizeToContents)
        for c in (2, 3):
            h.setSectionResizeMode(c, QHeaderView.Stretch)
        h.setStretchLastSection(False)
        t.setTextElideMode(Qt.ElideRight)
        t.setWordWrap(False)

    def add_joint_row(self, cells, obj, grounded):
        t = self.joints_table
        row = t.rowCount()
        t.insertRow(row)
        for col, val in enumerate(cells):
            t.setItem(row, col, QTableWidgetItem(str(val)))

        if not grounded:
            fbtn = QPushButton("Flip")
            fbtn.setToolTip("Flip the part to mate from otherside")
            fbtn.clicked.connect(lambda _=False,
                                 o=obj: self.creator.flip_joint(o))
            t.setCellWidget(row, 4, fbtn)

        btn = QPushButton("X")
        btn.clicked.connect(
            lambda _=False, o=obj, g=grounded:
                self.creator.delete_joint(o, g))
        t.setCellWidget(row, t.columnCount() - 1, btn)

    # UI state handlers

    def set_curr_step(self, step):
        self.curr_step = step
        self.update_ui()

    def update_ui(self):
        s = self.curr_step
        f = self.form
        f.stepHeader.setText(CREATION_STEPS[s])
        f.backButton.setEnabled(s > CreationStep.IMPORT_PARTS)
        f.nextButton.setEnabled(self.can_advance(s) and
                                s < CreationStep.REVIEW)

        f.filesGroup.setVisible(s == CreationStep.IMPORT_PARTS)
        f.partsGroup.setVisible(s == CreationStep.CREATE_ASSEMBLY)

        # f.jointsGroup.setVisible(s == CreationStep.ADD_JOINTS)
        picking = (s == CreationStep.ADD_JOINTS)
        f.jointsGroup.setVisible(s >= CreationStep.ADD_JOINTS)
        f.jointsGroup.setEnabled(picking or s == CreationStep.REVIEW)

        if s == CreationStep.CREATE_ASSEMBLY:
            self.ensure_asm()
            self.refresh_available_parts()

        # dynamically handle the joint-creation steps
        f.selectFace1Button.setEnabled(picking and self.face_slot == 1)
        f.selectFace2Button.setEnabled(picking and self.face_slot == 2)
        f.addJointButton.setEnabled(picking and bool(self.pending_faces))
        f.clearFacesButton.setEnabled(picking and bool(self.pending_faces))

        self.refresh_guidance()
        self.refresh_status()
        self.refresh_joints_table()
        self.refresh_pending_faces()
        self.form.adjustSize()

    def on_next(self):
        if self.can_advance(self.curr_step):
            self.set_curr_step(CreationStep(self.curr_step + 1))

    def on_back(self):
        if self.curr_step > CreationStep.IMPORT_PARTS:
            self.set_curr_step(CreationStep(self.curr_step - 1))

    def on_clear_faces(self):
        Gui.Selection.clearSelection()
        self.refresh_joints_panel()

    def can_advance(self, step):
        # at each step check if we can proceed to next
        # creation step or finish
        if step == CreationStep.IMPORT_PARTS:
            has_parts = any(o.isDerivedFrom("Part::Feature")
                            or o.isDerivedFrom("App::Part")
                            for o in self.doc.Objects)
            return (bool(self.imported_names) or has_parts or
                    bool(self.creator.candidate_assemblies()))

        if step == CreationStep.CREATE_ASSEMBLY:
            return (self.creator.assembly is not None and
                    self.creator.fpo is not None and
                    self.creator.part_count() >= 1)

        if step == CreationStep.ADD_JOINTS:
            return self.creator.is_valid_robot()

        return True

    def refresh_guidance(self, override=None, level="o"):
        """
        hints for users to guide them to the correct steps
        level: 'w'/'e' for inline warning/errors
        """
        if override is not None:
            self.form.guidanceLabel.setText(
                set_txt_color(override, level)
            )
            return

        if self.curr_step == CreationStep.ADD_JOINTS:
            if self.creator.grounded_joint() is None:
                text = """Pick one face on the base link, then
                        click 'Add Joint' to add it as ground link"""
            else:
                text = """Pick face 1 & 2 on adjacent links to add
                            a new joint"""
        else:
            text = GUIDANCE[self.curr_step]

        self.form.guidanceLabel.setText(set_txt_color(text, "o"))

    def refresh_pending_faces(self):
        """Update the faces already picked for the joint
        and the face selected currently in the 3D view"""

        picked = [
            f"{face['obj'].Label}.{face['ref']}"
            for face in self.pending_faces.values()
            ]

        sel = Gui.Selection.getSelectionEx("", 0)
        if sel and sel[0].SubElementNames:
            in_view = (
                f"{sel[0].Object.Label}"
                + "."
                + f"{sel[0].SubElementNames[0]}"
            )
        else:
            in_view = "-"

        next_slot = "face 1" if self.face_slot == 1 else "face 2"

        self.form.pendingFacesLabel.setText(
            set_txt_color("picked: " + (", ".join(picked) or "none"), "o")
            + "<br>"
            + set_txt_color(f"current selection: {in_view}", "w")
            + "<br>"
            + set_txt_color(f"next pick → {next_slot}", "o"))

    # task panel lifecylce

    def accept(self):
        if not self.creator.is_valid_robot():
            msg_box(self.form, " ", self.form.font(),
                    """
                <b>Cannot Create Robot</b><br>
                Robot needs a base (grounded) link and at least one joint.
                    """)
            return False

        Gui.Selection.removeSelectionGate()
        self.teardown_observer()
        self.doc.commitTransaction()
        self.doc.recompute()
        Gui.Control.closeDialog()
        return True

    def reject(self):
        Gui.Selection.removeSelectionGate()
        self.teardown_observer()
        self.doc.abortTransaction()

        # if new file was created (owner=True), but not
        # saved - drop file. If it was saved by user
        # then it needs to be deleted by the user on their own

        if self.doc_owner and not self.doc.FileName:
            App.closeDocument(self.doc.Name)
        else:
            for name in list(self.imported_names):
                if self.doc.getObject(name):
                    self.doc.removeObject(name)

            self.doc.recompute()

        Gui.Control.closeDialog()
        return True

    def teardown_observer(self):
        if self.doc_observer is not None:
            self.doc_observer.stop()
            self.doc_observer = None
        if self.selection_observer is not None:
            Gui.Selection.removeObserver(self.selection_observer)
            self.selection_observer = None

    def on_obj_deleted(self, was_joint, link_name=None):
        if self.doc_observer is None:
            # case: session already closing down
            return
        if self.creator.assembly is None:
            # case: no assembly built yet
            return
        if self.creator.resolve() is None:
            # case: assembly undone by undo or cancel press
            self.reject()
        elif was_joint:
            # case: a joint was removed (using in-panel delete)
            self.refresh_joints_panel()
        elif link_name is not None:
            # case: a part link was removed (from treeview or panel)
            self.creator.on_link_removed(link_name)
            self.update_ui()
            # self.fit_view() -> enables fitAll after deletion


def run():
    if Gui.Control.activeDialog():
        fcl_warn("close the active task panel first \n")
        return
    Gui.Control.showDialog(DefineRobot())
