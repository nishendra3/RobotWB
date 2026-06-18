"""vp_rbt_tool.py — View Provider for Tool & TCP."""
__version__ = "0.01"

import time

# import math
import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore

from pivy import coin  # type: ignore
from pivy.coin import SbMatrix, SbVec3f, SbRotation  # type: ignore
from freecad.Robot_tools.Gui.define_tool import tool_parent, DefineTCP

# import kinematic library functions
from freecad.Robot_tools.App import rbt_kine

from PySide.QtCore import Qt  # type: ignore
from PySide.QtWidgets import QApplication  # type: ignore
from PySide.QtCore import QTimer  # type: ignore

fcl_msg = App.Console.PrintMessage
fcl_err = App.Console.PrintError

MARKER_RADIUS_MM = 1


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

        self.f_dragging = False
        self.f_solving_ik = False  # flag to guard IK
        self.drag_start_tcp = None
        self.robot = None
        self._grab_action = None  # tcp sphere grab

        # Axes
        self.axes_tx = coin.SoTransform()
        self.axes = self.cm_axes()
        axes_sep = coin.SoSeparator()
        axes_sep.addChild(self.axes_tx)
        axes_sep.addChild(self.axes.Node)

        # Sphere Marker
        marker = coin.SoSeparator()
        pick = coin.SoPickStyle()
        # pick.style = coin.SoPickStyle.UNPICKABLE
        marker_mat = coin.SoMaterial()
        marker_mat.diffuseColor.setValue(1.0, 0.5, 0.0)
        marker_sphere = coin.SoSphere()
        marker_sphere.radius.setValue(MARKER_RADIUS_MM)
        marker.addChild(pick)
        marker.addChild(marker_mat)
        marker.addChild(marker_sphere)
        self._marker_sep = marker  # store for use by pick filter

        # -- callback functions on the sphere --
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

        axes_sep.addChild(marker)

        # Dragger
        self._cb_finish = self._on_drag_finish
        # self.dragger = self.cm_dragger()

        root = coin.SoSeparator()
        root.addChild(axes_sep)
        # root.addChild(self.dragger)
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

    def cm_axes(self):
        """
        Freecad's built in axes triad
        """
        axes = Gui.AxisOrigin()
        axes.AxisLength = 6
        axes.Scale = 1
        return axes

    def cm_dragger(self):
        """
        Draggable axes triad based on legacy roboWB
        """
        d = coin.SoJackDragger()
        d.ref()
        d.addFinishCallback(self._cb_finish, self)
        return d

    def updateData(self, fp, prop):
        # TODO
        # "TCP_offset", "Tool_offset", "Flange_link"
        if prop in ("TCP_placement",):
            self.refresh_tx(fp)

    def refresh_tx(self, fp):
        """
        update self.tx from curr TCP placement
        """
        if getattr(self, "axes_tx", None) is None:
            return
        w = getattr(fp, "TCP_placement", None)
        if w is None:
            return
        q = w.Rotation.Q
        self.axes_tx.translation = (w.Base.x, w.Base.y, w.Base.z)
        self.axes_tx.rotation = (q[0], q[1], q[2], q[3])

        # if the tcp moves from joint-panel, move the dragger too
        if getattr(self, "dragger", None) is None:
            return
        if not self.f_solving_ik:
            m = SbMatrix()
            m.setTransform(
                SbVec3f(w.Base.x, w.Base.y, w.Base.z),
                SbRotation(q[0], q[1], q[2], q[3]),
                SbVec3f(150.0, 150.0, 150.0))
            self.dragger.setMotionMatrix(m)

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
        if self.robot is None:
            fcl_err("[tool vp] no parent robot found")
            return

        # ==== time profiling ====
        self._t_ik = []
        self._t_apply = []
        self._t_split = []
        self._n_reject = 0
        self._drag_t0 = time.perf_counter()
        self._last_target = None
        self._q_seed = rbt_kine.current_q_deg(self.robot)

    def _on_drag_motion(self, userdata, event_cb):
        # fcl_msg("-- on drag motion --")

        if self.f_solving_ik:
            return
        if self.drag_start_tcp is None:
            return
        if self.robot is None:
            return
        if event_cb is None:
            return

        # get 3D projection of mouse position
        mouse_3d = self._mouse_on_tcp_plane(event_cb)
        if mouse_3d is None:
            return

        target = App.Placement(
            mouse_3d,
            self.drag_start_tcp.Rotation,
        )

        self.f_solving_ik = True
        sol = None

        t0 = time.perf_counter()

        try:
            sol = rbt_kine.ik(self.robot, target,
                              q_seed_deg=self._q_seed,
                              pos_tol_mm=0.5)
        except Exception as e:
            fcl_err(f"failed to solve IK {e}\n")

        self._t_ik.append(time.perf_counter() - t0)
        self._last_target = target

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
        t0 = time.perf_counter()
        try:
            self.push_joints(robot, q_deg)
        except Exception as e:
            fcl_err(f"Error changing joints: {e}\n")
        finally:
            # reset the flag after solution has been pushed
            self._t_apply.append(time.perf_counter() - t0)
            self.f_solving_ik = False

    # temp function for performance logging
    def _print_drag_stats(self):
        n = len(getattr(self, "_t_ik", []))
        if n == 0:
            return
        total = time.perf_counter() - self._drag_t0
        ik_ms = [t *1000 for t in self._t_ik]
        ap_ms = [t *1000 for t in self._t_apply]

        def mean(v):
            return sum(v) / len(v) if v else 0

        fcl_msg(
            f"[drag] {n} solves in {total:.2f}s ({n / total:.1f} fps) | "
            f"ik mean {mean(ik_ms):.1f} / max {max(ik_ms):.1f} ms | "
            f"apply mean {mean(ap_ms):.1f} / max {max(ap_ms) if ap_ms else 0:.1f} ms | "
            f"rejected {self._n_reject}\n"
        )
        if getattr(self, "_t_split", None):
            n2 = len(self._t_split)
            s = [sum(v[i] for v in self._t_split) / n2 * 1000 for i in (0, 1, 2)]
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
            except:
                pass
            self._grab_action = None

        if self.robot is not None and self._q_seed is not None:
            rbt_kine.resolve_offsets(self.robot, self._q_seed)

        self._print_drag_stats()
        self._last_target = None
        self._q_seed = None

        self.f_solving_ik = False
        self.drag_start_tcp = None
        self.robot = None

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

            # only start dragging when pick is on marker sphere
            if not pick.getPath().containsNode(self._marker_sep):
                return

            event_cb.setHandled()

            # route all events to above node until release
            self._grab_action = event_cb.getAction()
            self._grab_action.setGrabber(self._sphere_cb)
            self.f_dragging = True
            self._on_drag_start(userdata, None)

        elif (coin.SoMouseButtonEvent
              .isButtonReleaseEvent(event, coin.SoMouseButtonEvent.BUTTON1)):
            if self._grab_action is not None:
                self._grab_action.releaseGrabber()
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

        view = Gui.getDocument(self.Object.Document.Name).ActiveView
        px, py = event_cb.getEvent().getPosition().getValue()
        p1, p2 = view.projectPointToLine(px, py)  # world ray in mm
        n = App.Vector(*view.getViewDirection())  # camera plane normal
        p0 = self.drag_start_tcp.Base  # plane through TCP
        d = p2-p1
        denom = d.dot(n)
        if (abs(denom)) < 1e-9:
            return None
        t = (p0 - p1).dot(n) / denom
        return p1 + d * t
