"""vp_rbt_tool.py — View Provider for Tool & TCP."""

from enum import IntEnum


import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

from pivy import coin  # type: ignore
from freecad.Robot_tools.App.rbt_helpers_log import fcl_err, fcl_msg
from freecad.Robot_tools.App.rbt_helpers_frames import jog_rotation
from freecad.Robot_tools.App.rbt_tool import tool_parent
from freecad.Robot_tools.Gui.taskpanel_rbt_tool import DefineTCP
from freecad.Robot_tools.Gui import so_helpers as so

# import kinematic library functions
from freecad.Robot_tools.App import rbt_kine

from PySide.QtCore import Qt  # type: ignore
from PySide.QtWidgets import QApplication  # type: ignore
from PySide.QtCore import QTimer  # type: ignore

MARKER_RADIUS_MM = 10
MARKER_GRAB_R_MM = 1.5 * MARKER_RADIUS_MM  # Grabale region around TCP

AXIS_UNITS = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
AXIS_COLORS = [(1, 0, 0), (0, .8, 0), (.2, .4, 1)]

# AXIS DRAGGERS DIMENSIONS
SHAFT_LEN, SHAFT_R = 30.0, 1.0
TIP_H, TIP_R = 10.0, 4.0


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

        marker = so.sep(pick, self._marker_mat,
                        sphere, invis, grab)

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
        axes_sep = so.group(coin.SoAnnotation(),
                            self.axes_tx,
                            self.scale_kit)

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
        make the x, y, z drag handle for the tcp: shaft + tip
        material & pick nodes stay per-axis for pick routing
        """
        mat = so.material(AXIS_COLORS[i])
        self._axis_mat[i] = mat

        pk = coin.SoPickStyle()
        pk.style.setValue(coin.SoPickStyle.BOUNDING_BOX)

        sep = so.sep(mat,
                     pk,
                     so.arrow(AXIS_UNITS[i], TIP_R, TIP_H,
                              shaft_len=SHAFT_LEN, shaft_r=SHAFT_R))

        self._axis_sep[i] = sep  # pick routing: path.containsNode(sep)
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
        rbt_kine.apply_joint_angles(robot, q_deg)   # FK

    def getDisplayModes(self, vobj):
        return ["Standard"]

    def getDefaultDisplayMode(self):
        return "Standard"

    def getIcon(self):
        import os
        from freecad.Robot_tools import rbt_locator
        wb_path = os.path.dirname(rbt_locator.__file__)
        return os.path.join(wb_path,
                            "resources/icons/rbt_defineTool.svg")

    # -- event slots --
    def _on_drag_start(self, userdata, dragger):
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

        # filter out the degenrate case & fallback to FreeRotation
        if self._drag_axis_dir is not None:
            view_dir = App.Vector(*self._view.getViewDirection())
            # check if the axis points out of the screen
            if abs(self._drag_axis_dir.dot(view_dir)) > 0.95:
                fcl_msg("[tool_vp] drag axis parallel to "
                        "view - free drag fallback\n")
                self._active_axis = DragMode.FREE
                self._drag_axis_dir = None

        self._q_seed = rbt_kine.curr_joint_vals_doc(self.robot)

    def _on_drag_motion(self, userdata, event_cb):
        if self.drag_start_tcp is None:
            return
        if self.robot is None:
            return
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

        try:
            sol = rbt_kine.ik(self.robot, target,
                              q_seed_deg=self._q_seed)
        except Exception as e:
            fcl_err(f"failed to solve IK {e}\n")

        if sol is not None:
            # solving_ik flag is reset in _apply_solution()
            # after joint values have been pushed
            self._q_seed = sol  # start next ik from curr q
            r = self.robot
            QTimer.singleShot(0, lambda s=sol: self._apply_solution(s, r))
        else:
            self.f_solving_ik = False

    def _apply_solution(self, q_deg, robot):
        if not self.f_dragging:
            self.f_solving_ik = False
            return

        try:
            self.push_joints(robot, q_deg)
        except Exception as e:
            fcl_err(f"Error changing joints: {e}\n")
        finally:
            # reset the flag after solution has been pushed
            self.f_solving_ik = False

            pending, self._pending_pos = self._pending_pos, None
            if pending is not None and self.f_dragging:
                self._solve_to(pending)

    def _on_drag_finish(self, userdata, dragger):
        act = getattr(self, "_grab_action", None)

        if act is not None:
            try:
                act.releaseGrabber()
            except Exception as e:
                fcl_err(f"Failed to release grabber: {e}\n")
            self._grab_action = None

        if self.robot is not None and self._q_seed is not None:
            rbt_kine.resolve_offsets(self.robot, self._q_seed)

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

        # world ray in mm
        ray_start, ray_end = self._view.projectPointToLine(px, py)
        ray = ray_end - ray_start

        # get a plane through tcp & camera plane normal
        plane_origin = self.drag_start_tcp.Base  # plane through TCP

        # camera pl norm.
        plane_normal = App.Vector(*self._view.getViewDirection())

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

        # world ray in mm
        ray_start, ray_end = self._view.projectPointToLine(px, py)
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
