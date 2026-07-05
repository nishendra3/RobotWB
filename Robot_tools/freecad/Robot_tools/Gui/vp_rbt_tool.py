"""vp_rbt_tool.py — View Provider for Tool & TCP."""
__version__ = "0.01"

import time
from enum import IntEnum


import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

from pivy import coin  # type: ignore
from freecad.Robot_tools.App.rbt_logging import fcl_err, fcl_msg
from freecad.Robot_tools.App.rbt_frames import jog_rotation
from freecad.Robot_tools.Gui.define_tool import tool_parent, DefineTCP

# import kinematic library functions
from freecad.Robot_tools.App import rbt_kine

from PySide.QtCore import Qt  # type: ignore
from PySide.QtWidgets import QApplication  # type: ignore
from PySide.QtCore import QTimer  # type: ignore

MARKER_RADIUS_MM = 10
MARKER_GRAB_R_MM = 1.5 * MARKER_RADIUS_MM  # Grabale region around TCP

AXIS_UNITS = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
AXIS_COLORS = [(1, 0, 0), (0, .8, 0), (.2, .4, 1)]


class DragMode(IntEnum):
    X = 0
    Y = 1
    Z = 2
    FREE = 3


class ViewProviderTool:
    def __init__(self, vobj):
        vobj.Proxy = self

    # bypass freeCAD's proxy state saving

    def dumps(self):
        return None

    def loads(self, state):
        return None

    def attach(self, vobj):

        self.vobj = vobj
        self.Object = vobj.Object

        # -- drag state --
        self.f_dragging = False
        self.f_solving_ik = False  # flag to guard IK
        self.drag_start_tcp = None
        self.robot = None
        self._grab_action = None  # tcp sphere grab
        self._pending_pos = None  # pending tcp pose for ik solution

        # -- axis-handle state --
        # (filled by cm_axis_handle later)
        self._active_axis = None
        self._axis_sep = [None, None, None]
        self._axis_mat = [None, None, None]

        # -- jog frame state --
        self._drag_frame_rot = App.Rotation()

        # -- TCP pose (pos+orientation) --
        # (updated in refresh_tx)
        self.axes_tx = coin.SoTransform()

        # -- TCP sphere marker --
        marker = coin.SoSeparator()
        pick = coin.SoPickStyle()
        pick.style.setValue(coin.SoPickStyle.BOUNDING_BOX)

        self._marker_mat = coin.SoMaterial()
        self._marker_mat.diffuseColor.setValue(1.0, 0.5, 0.0)

        sphere = coin.SoSphere()
        sphere.radius.setValue(MARKER_RADIUS_MM)

        invis = coin.SoDrawStyle()  # makes the larger grab sphere invisible
        invis.style.setValue(coin.SoDrawStyle.INVISIBLE)

        grab = coin.SoSphere()
        grab.radius.setValue(MARKER_GRAB_R_MM)

        for n in (pick, self._marker_mat, sphere, invis, grab):
            marker.addChild(n)

        self._marker_sep = marker  # store for use by pick filter

        # -- mouse callbacks on the drag sphere --
        self._sphere_cb = coin.SoEventCallback()
        self._sphere_cb.addEventCallback(
            coin.SoMouseButtonEvent.getClassTypeId(),
            self._on_sphere_click,
            self
        )
        self._sphere_cb.addEventCallback(
            coin.SoLocation2Event.getClassTypeId(),
            self._on_sphere_motion,
            self
        )
        marker.addChild(self._sphere_cb)

        # -- gizmo for drag along axis --
        gizmo = coin.SoSeparator()
        for i in range(3):
            gizmo.addChild(self.cm_axis_handle(i))
        gizmo.addChild(marker)

        self.scale_kit = coin.SoType.fromName("SoShapeScale").createInstance()
        self.scale_kit.setPart("shape", gizmo)
        self.scale_kit.scaleFactor = 1.0  # tunable
        self.scale_kit.active = 1

        # -- add together with pose --
        axes_sep = coin.SoSeparator()
        axes_sep.addChild(self.axes_tx)
        axes_sep.addChild(self.scale_kit)

        root = coin.SoSeparator()
        root.addChild(axes_sep)
        vobj.addDisplayMode(root, "Standard")

        self.refresh_tx(vobj.Object)

    def claimChildren(self):
        """
        joins the tool CAD as a sub-child in the tree
        """
        obj = self.Object
        return ([obj.Tool_shape]
                if getattr(obj, "Tool_shape", None)
                else [])

    def doubleClicked(self, vobj):
        # Find the robot that owns this tool.
        robot = tool_parent(vobj.Object)
        if robot is None:
            fcl_err("No parent robot obj found for selected tool")
            return True

        Gui.Control.showDialog(DefineTCP(robot, tool=vobj.Object))
        return True

    def setEdit(self, vobj, mode=0):
        return self.doubleClicked(vobj)

    def unsetEdit(self, vobj, mode=0):
        Gui.Control.closeDialog()
        return True

    def onDelete(self, vobj, subelements):
        return True

    def cm_axis_handle(self, i):
        """
        make the x, y, z triade axes for tcp
        """
        shaft_len = 30.0
        shaft_rad = 1.0
        tip_height = 10.0
        tip_radius = 4.0

        sep = coin.SoSeparator()

        rot = coin.SoRotation()
        rot.rotation.setValue(
            coin.SbRotation(
                coin.SbVec3f(0, 1, 0),
                coin.SbVec3f(*AXIS_UNITS[i])
            )
        )

        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(*AXIS_COLORS[i])
        self._axis_mat[i] = mat

        pk = coin.SoPickStyle()
        pk.style.setValue(coin.SoPickStyle.BOUNDING_BOX)

        # Shaft
        shaft_grp = coin.SoSeparator()
        shaft_tx = coin.SoTranslation()
        shaft_tx.translation.setValue(0, shaft_len / 2, 0)

        shaft = coin.SoCylinder()
        shaft.radius = shaft_rad
        shaft.height = shaft_len

        shaft_grp.addChild(shaft_tx)
        shaft_grp.addChild(shaft)

        # Tip
        tip_grp = coin.SoSeparator()
        tip_tx = coin.SoTranslation()
        tip_tx.translation.setValue(0, shaft_len + tip_height / 2, 0)

        tip = coin.SoCone()
        tip.bottomRadius = tip_radius
        tip.height = tip_height

        tip_grp.addChild(tip_tx)
        tip_grp.addChild(tip)

        for n in (rot, mat, pk, shaft_grp, tip_grp):
            sep.addChild(n)

        self._axis_sep[i] = sep  # cache for later use
        return sep

    def updateData(self, fp, prop):
        # TODO
        # "TCP_offset", "Tool_offset", "Flange_link"
        if prop in ("TCP_placement", "TCP_drag_frame"):
            self.refresh_tx(fp)

    def refresh_tx(self, fp):
        """
        move the drag gizmo to the tcp & orient it to active joint frame
        """
        if getattr(self, "axes_tx", None) is None:
            return
        w = getattr(fp, "TCP_placement", None)
        if w is None:
            return
        self.axes_tx.translation = (w.Base.x, w.Base.y, w.Base.z)

        # keep the rotation frozen while dragging

        # orient the rotation based on the jog-frame selected
        if not self.f_dragging:
            self.axes_tx.rotation = jog_rotation(fp).Q

    def push_joints(self, robot, q_deg):
        """FK preview during drag"""
        t0 = time.perf_counter()
        rbt_kine.apply_joint_angles(robot, q_deg)   # FK
        t_set = time.perf_counter() - t0
        if getattr(self, "_t_split", None) is not None:
            self._t_split.append((t_set, 0.0, 0.0))

    def getDisplayModes(self, vobj):
        return ["Standard"]

    def getDefaultDisplayMode(self):
        return "Standard"

    def getIcon(self):
        import os
        from freecad.Robot_tools import tb_locator
        wb_path = os.path.dirname(tb_locator.__file__)
        return os.path.join(wb_path,
                            "resources/icons/rbt_defineTool.svg")

    # -- event slots --
    def _on_drag_start(self, userdata, dragger):
        # fcl_msg("-- on drag start --\n")
        self.f_solving_ik = False
        fp = self.Object
        self.drag_start_tcp = App.Placement(fp.TCP_placement)
        self.robot = tool_parent(fp)
        self._pending_pos = None

        if self.robot is None:
            fcl_err("[tool vp] no parent robot found")
            return

        self._view = Gui.getDocument(fp.Document.Name).ActiveView

        self._drag_frame_rot = jog_rotation(fp)
        self.axes_tx.rotation = self._drag_frame_rot.Q

        self._drag_axis_dir = (
            None if self._active_axis == DragMode.FREE
            else self._drag_frame_rot.multVec(
                App.Vector(*AXIS_UNITS[self._active_axis])))

        # ==== time profiling ====
        self._t_ik = []
        self._t_apply = []
        self._t_split = []
        self._n_reject = 0
        self._drag_t0 = time.perf_counter()
        self._q_seed = rbt_kine.current_q_deg(self.robot)


    def _on_drag_motion(self, userdata, event_cb):
        # fcl_msg("-- on drag motion --")

        # if self.f_solving_ik:
        #     return
        if self.drag_start_tcp is None:
            return
        if self.robot is None:
            return
        # if event_cb is None:
        #     return
        if self._active_axis is None:
            return

        if self._active_axis == DragMode.FREE:
            # free drag
            # get 3D projection of mouse position
            mouse_3d = self._mouse_on_tcp_plane(event_cb)
        else:
            # contrained drag
            mouse_3d = self._mouse_on_axis(event_cb, self._drag_axis_dir)

        if mouse_3d is None:
            return

        if self.f_solving_ik:
            self._pending_pos = mouse_3d
            return

        self._solve_to(mouse_3d)

    def _solve_to(self, mouse_3d):

        target = App.Placement(
            mouse_3d,
            self.drag_start_tcp.Rotation,
        )

        self.f_solving_ik = True
        sol = None

        t0 = time.perf_counter()

        try:
            sol = rbt_kine.ik(self.robot, target,
                              q_seed_deg=self._q_seed)
        except Exception as e:
            fcl_err(f"failed to solve IK {e}\n")

        self._t_ik.append(time.perf_counter() - t0)

        if sol is not None:
            # solving_ik flag is reset in _apply_solution()
            # after joint values have been pushed
            self._q_seed = sol  # start next ik from curr q
            r = self.robot
            QTimer.singleShot(0, lambda s=sol: self._apply_solution(s, r))
        else:
            self._n_reject += 1
            self.f_solving_ik = False

    def _apply_solution(self, q_deg, robot):
        if not self.f_dragging:
            self.f_solving_ik = False
            return

        t0 = time.perf_counter()
        try:
            self.push_joints(robot, q_deg)
        except Exception as e:
            fcl_err(f"Error changing joints: {e}\n")
        finally:
            # reset the flag after solution has been pushed
            self._t_apply.append(time.perf_counter() - t0)
            self.f_solving_ik = False

            pending, self._pending_pos = self._pending_pos, None
            if pending is not None and self.f_dragging:
                self._solve_to(pending)

    # temp function for performance logging
    def _print_drag_stats(self):
        n = len(getattr(self, "_t_ik", []))
        if n == 0:
            return
        total = time.perf_counter() - self._drag_t0
        ik_ms = [t * 1000 for t in self._t_ik]
        ap_ms = [t * 1000 for t in self._t_apply]

        def mean(v):
            return sum(v) / len(v) if v else 0

        fcl_msg(
            f"[drag] {n} solves in {total:.2f}s ({n / total:.1f} fps) | "
            f"ik mean {mean(ik_ms):.1f} / max {max(ik_ms):.1f} ms | "
            f"apply mean { mean(ap_ms):.1f} / max {max(ap_ms) if ap_ms else 0:.1f} ms | "
            f"rejected {self._n_reject}\n"
        )
        if getattr(self, "_t_split", None):
            n2 = len(self._t_split)
            s = [sum(v[i] for v in self._t_split) / n2 * 1000 for
                 i in (0, 1, 2)]
            fcl_msg(
                f"[apply] set {s[0]:.1f} | asm {s[1]:.1f} | "
                f"tool {s[2]:.1f} ms (means over {n2})\n"
            )

    def _on_drag_finish(self, userdata, dragger):
        # fcl_msg("-- on drag finish --\n")
        act = getattr(self, "_grab_action", None)

        if act is not None:
            try:
                act.releaseGrabber()
            except Exception as e:
                fcl_err(f"Failed to release grabber: {e}\n")
            self._grab_action = None

        if self.robot is not None and self._q_seed is not None:
            rbt_kine.resolve_offsets(self.robot, self._q_seed)

        self._print_drag_stats()
        self._q_seed = None
        self._active_axis = None

        self.f_solving_ik = False
        self.drag_start_tcp = None
        self.robot = None

        self.refresh_tx(self.Object)

    def _on_sphere_click(self, userdata, event_cb):
        """
            Callback for when the TCP sphere is clicked & released
        """

        event = event_cb.getEvent()

        if (coin.SoMouseButtonEvent
                .isButtonPressEvent(event, coin.SoMouseButtonEvent.BUTTON1)):
            pick = event_cb.getPickedPoint()
            if pick is None:
                return

            # route the click if it is relevant for us
            path = pick.getPath()

            if path.containsNode(self._marker_sep):
                # sphere: free camera-plane drag
                self._active_axis = DragMode.FREE

            else:
                # axes handlers
                self._active_axis = None
                for i, axis_sep in enumerate(self._axis_sep):
                    if path.containsNode(axis_sep):
                        self._active_axis = i
                        break

            # only start dragging when pick is on marker sphere
            if self._active_axis is None:
                return

            event_cb.setHandled()

            # route all events to above node until release
            self._grab_action = event_cb.getAction()
            self._grab_action.setGrabber(self._sphere_cb)
            self.f_dragging = True
            self._on_drag_start(userdata, None)

        elif (coin.SoMouseButtonEvent
              .isButtonReleaseEvent(event, coin.SoMouseButtonEvent.BUTTON1)):
            event_cb.setHandled()
            self.f_dragging = False
            self._on_drag_finish(userdata, None)

    def _on_sphere_motion(self, userdata, event_cb):
        """
            Callback for dragging motion over the TCP Sphere
        """
        if not self.f_dragging:
            return

        # handle edge cases for dragger release
        # ESC, focus loss, pop-ups
        # bitwise "and" of mouse buttons with left button ->
        if not (QApplication.mouseButtons() & Qt.LeftButton):
            self.f_dragging = False
            self._on_drag_finish(userdata, None)
            return

        event_cb.setHandled()
        self._on_drag_motion(userdata, event_cb)

    def _mouse_on_tcp_plane(self, event_cb):
        """
            Finds the intersection of mouse ray with a
            plane that lies in the TCP-Plane & returns
            the intersection.
        """

        px, py = event_cb.getEvent().getPosition().getValue()

        ray_start, ray_end = self._view.projectPointToLine(px, py)  # world ray in mm
        ray = ray_end - ray_start

        # get a plane through tcp & camera plane normal
        plane_origin = self.drag_start_tcp.Base  # plane through TCP
        plane_normal = App.Vector(*self._view.getViewDirection())  # camera pl norm.

        ray_dot_normal = ray.dot(plane_normal)
        if abs(ray_dot_normal) < 1e-9:
            return None

        dist = (plane_origin - ray_start).dot(plane_normal) / ray_dot_normal
        return ray_start + ray * dist

    def _mouse_on_axis(self, event_cb, axis_dir):
        """
        Return the point on the axis through the TCP that is closest
        to the mouse ray. Returns None if the ray and axis are parallel.
        """

        px, py = event_cb.getEvent().getPosition().getValue()

        ray_start, ray_end = self._view.projectPointToLine(px, py)  # world ray in mm
        axis_origin = self.drag_start_tcp.Base

        axis = App.Vector(axis_dir)
        axis.normalize()

        ray = ray_end - ray_start
        ray_len = ray.Length
        if ray_len < 1e-9:
            return None
        ray.normalize()

        offset = axis_origin - ray_start

        axis_dot_ray = axis.dot(ray)
        denom = 1.0 - axis_dot_ray * axis_dot_ray
        if abs(denom) < 1e-9:
            return None

        offset_dot_axis = offset.dot(axis)
        offset_dot_ray = offset.dot(ray)

        axis_param = (
            axis_dot_ray * offset_dot_ray - offset_dot_axis
        ) / denom

        return axis_origin + axis * axis_param
