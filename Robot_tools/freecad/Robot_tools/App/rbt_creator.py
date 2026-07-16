"""
Create a new robot from FreeCAD parts
"""

import FreeCAD as App  # type: ignore
import UtilsAssembly   # type: ignore


from freecad.Robot_tools.App.rbt_robot import Robot
from freecad.Robot_tools.App.rbt_creator_geom import add_base_frame
from freecad.Robot_tools.App.rbt_creator_asm import (
    create_assembly, add_asm_object, resolve_asm_ref,
    find_assemblies)
from freecad.Robot_tools.App.rbt_creator_jnt import add_joint
from freecad.Robot_tools.App.rbt_global_constants import (
    ROBOT_FPO_NAME, BASE_FRAME_NAME, RBT_PREFS,
    DEFAULT_INSERT_STAGGER_PCT)
from freecad.Robot_tools.App.rbt_placement import (
    is_grounded_datum, chain_root, find_grounded_joint)
from freecad.Robot_tools.App.rbt_kine_chain import joint_value_doc


class RobotCreator:
    """
    Builds a new robot from FC Parts
    Create Assm -> Insert Parts -> Insert Joints
    """

    def __init__(self):
        # self context
        self.asm_doc = self.assembly = self.fpo = None

        # insert-stacking state for links in the robot
        # link Name -> stagger offset at insert time
        self.insert_offsets = {}

        self.total_translation = App.Vector()

    def part_count(self):
        """
        Count of linked parts in the working assembly
        """
        asm = self.assembly
        return 0 if asm is None else (
            sum(o.isDerivedFrom('App::Link') for o in asm.Group))

    def link_count(self, obj):
        """
        count many App::Link in the  for the inserted counter
        """
        asm = self.assembly
        if asm is None:
            return 0
        return sum(1 for o in asm.Group
                   if o.isDerivedFrom("App::Link") and
                   o.LinkedObject is obj)

    # REMOVE: Legacy Path ----------
    def grounded_joint(self):
        """The assembly's GroundedJoint object, or None."""
        return find_grounded_joint(self.assembly)
    # ------------------------------

    def has_grounded_datum(self):
        return is_grounded_datum(chain_root(self.fpo), self.assembly)

    def is_valid_robot(self):
        """
        Valid Robot : BaseFrame Datum or legacy grounded joint
        At least one valid joint
        """
        a, f = self.assembly, self.fpo

        if a is None or f is None:
            return False

        has_base_ref = (self.grounded_joint() is not None
                        or self.has_grounded_datum())

        has_enough_joints = len(f.Robot_joints) >= 1

        return (has_base_ref and has_enough_joints)

    def bind(self, asm):
        """
        Sets 'asm' as the curr working assembly
        and fixes FPO to Assembly link
        """
        self.assembly = asm
        fpos = asm.Document.getObjectsByLabel('Robot_FPO')
        if len(fpos) == 1:
            self.fpo = fpos[0]
            ra = getattr(self.fpo, 'Robot_assembly', None)
            if ra is not asm:
                self.fpo.Robot_assembly = asm

    def resolve(self):
        """
        resolve the current assembly from doc
        """
        asm, fpo, how = resolve_asm_ref(self.asm_doc)
        if asm is None:
            return None
        self.bind(asm)
        return how

    def insert_parts(self, objs):
        """
        links 'objs' into the curr assembly
        returns parts count or None when no
        assembly is present in current doc
        """
        if self.resolve() is None:
            return None

        return [add_asm_object(o.Document,
                self.assembly,
                o.Name, o.Label, o.Label) for o in objs]

    def build_assembly(self, doc=None):
        """
        Adds Robot_Assembly + Robot_FPO into the
        working document
        """
        self.asm_doc = doc or self.asm_doc or App.ActiveDocument
        asm = create_assembly(self.asm_doc)
        fpo = self.asm_doc.addObject("App::FeaturePython", ROBOT_FPO_NAME)
        Robot(fpo)
        if App.GuiUp:
            # ^This is needed to nest the Robot_assembly and Toools
            # under the main FPO tree node
            from freecad.Robot_tools.Gui.vp_rbt_robot \
                import ViewProviderRobot
            ViewProviderRobot(fpo.ViewObject)
        fpo.Robot_assembly = asm
        self.asm_doc.recompute()
        self.bind(asm)
        return asm

    def insert_base(self, jtype, pick, label=""):
        """
        BaseFrame + joint 0 of type 'jtype' into
        the user picked selection
        """
        lcs = add_base_frame(self.assembly, pick)
        z_axis = next(f for f in lcs.OriginFeatures if f.Role == "Z_Axis")
        base_joint = self.insert_joint(jtype,
                                       [(lcs, z_axis.Name+"."), pick], label)
        base_joint.Label = "Base_joint"
        return base_joint

    def insert_joint(self, jtype, refs, label=""):
        """
        Creates and adds joint of the type 'jtype'
        in the assembly & registers them with FPO
        """
        j = add_joint(self.assembly, jtype, refs, label)
        self.fpo.Robot_zero_pose = (list(self.fpo.Robot_zero_pose)
                                    + [joint_value_doc(j, 1)])
        self.fpo.Robot_joints = list(self.fpo.Robot_joints) + [j]
        self.fpo.Robot_joints_dir = list(self.fpo.Robot_joints_dir) + [1]
        return j

    def next_joint_index(self):
        """Next free rb_jnt index (max existing + 1)."""
        js = self.fpo.Robot_joints if self.fpo else []
        idxs = [int(j.Label2[6:]) for j in js if j.Label2[6:].isdigit()]
        return max(idxs, default=-1) + 1

    def flip_joint(self, joint):
        """
        re-mate the joint's moving part on the other side of the face
        """
        joint.Proxy.flipOnePart(joint)
        self.assembly.Document.recompute()
        if self.fpo is not None:
            # invalidate and recreate kinematic chain
            # incase the joint dir is flipped
            from freecad.Robot_tools.App import rbt_kine
            rbt_kine.invalidate(self.fpo)

    def delete_joint(self, obj):
        """Remove a joint and keep robot joints/directions in sync."""
        doc = self.assembly.Document

        if self.fpo and obj in self.fpo.Robot_joints:
            joints = list(self.fpo.Robot_joints)
            dirs = list(self.fpo.Robot_joints_dir)

            idx = joints.index(obj)
            joints.pop(idx)
            if idx < len(dirs):
                dirs.pop(idx)

            self.fpo.Robot_joints = joints
            self.fpo.Robot_joints_dir = dirs

            zeros = list(self.fpo.Robot_zero_pose)
            if idx < len(zeros):
                zeros.pop(idx)
                self.fpo.Robot_zero_pose = zeros

        # read frame before the joint deletion
        refs = getattr(obj, "Reference1", None)
        root = refs[0] if refs else None

        doc.removeObject(obj.Name)

        if root and BASE_FRAME_NAME in root.Name:
            joints = self.fpo.Robot_joints if self.fpo else ()
            if not any(j.Reference1
                       and j.Reference1[0] is root
                       for j in joints):
                doc.removeObject(root.Name)

        doc.recompute()

    def stack_translation(self, link):
        """
        accumulated insert offset
        first link stays at its CAD file pose, the later ones
        offset by the user-set stagger percent (0 = disabled)
        """
        translation = (App.Vector() if not self.insert_offsets
                       else self.translation_vec(link))
        self.insert_offsets[link.Name] = translation
        self.total_translation += translation
        return App.Vector(self.total_translation)

    def translation_vec(self, link):
        """
        offset for a newly inserted part
        InsertStaggerPct % of its bbox, 10mm fallback
        """
        pct = App.ParamGet(RBT_PREFS).GetFloat(
            "InsertStaggerPct", DEFAULT_INSERT_STAGGER_PCT
        )
        if pct <= 0:
            return App.Vector()
        shape = getattr(link, "Shape", None)
        bb = shape.BoundBox if shape is not None else None
        t = ((bb.XMax + bb.YMax + bb. ZMax) * pct / 100.0
             if bb is not None and bb.isValid() else 10)
        return App.Vector(t, t, t)

    def curr_last_link(self, obj):
        """
        most recently inserted App::Link
        """
        if self.assembly is None:
            return None
        links = [o for o in self.assembly.Group
                 if o.isDerivedFrom("App::Link") and
                 o.LinkedObject is obj]
        return links[-1] if links else None

    def joints_ref(self, link):
        """all joints attached to the link"""
        out = []
        for joint in self.assembly.Joints:
            if hasattr(joint, "ObjectToGround"):   # legacy, skip
                continue
            refs = (getattr(joint, "Reference1", None),
                    getattr(joint, "Reference2", None))
            if any(ref and ref[0] is link for ref in refs):
                out.append(joint)
        return out

    def remove_link(self, link):
        """
        delete all attached joints to the link
        and then the link itself
        """
        for joint in self.joints_ref(link):
            self.delete_joint(joint)

        doc = link.Document
        self.on_link_removed(link.Name)
        doc.removeObject(link.Name)
        doc.recompute()

    def on_link_removed(self, name):
        """
        forget the removed link's stagger offset
        """
        self.total_translation -= self.insert_offsets.pop(name, App.Vector())

    def track_imported(self, before_names):
        """
        Tracks part names that are added to the assembly
        """
        return {o.Name for o in self.asm_doc.Objects} - set(before_names)

    def resolve_face(self, obj, sub):
        """
        Find the link for the selected face on object in the
        working assembly. Returns (link_obj, face_ref)
        """
        return UtilsAssembly.getComponentReference(self.assembly,
                                                   obj, sub)

    def assembly_owner(self, obj, sub):
        """
        Return the Robot_Assembly that owns the given subelement of `obj`.

        Returns:
            Assembly object if the subelement belongs to a Robot_Assembly,
            otherwise None.
        """
        return next(
            (
                asm
                for asm in find_assemblies(self.asm_doc)
                if UtilsAssembly.getComponentReference(asm,
                                                       obj, sub)[0] is not None
            ),
            None)
