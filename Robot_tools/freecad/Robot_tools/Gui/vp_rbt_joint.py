"""
vp_rbt_joint.py
File for handing display icons for joints
"""

from pivy import coin  # type: ignore
from JointObject import ViewProviderJoint  # type: ignore
from freecad.Robot_tools.App.rbt_placement import is_base_joint, joint_dir
from freecad.Robot_tools.Gui.so_jnt_marker import SoJointMarker


class ViewProviderBaseJoint(ViewProviderJoint):
    """
        Redraw the Joint Coordinate System (JCS)
        icons in the viewport
    """
    def getIcon(self):
        j = getattr(self, "app_obj", None)
        is_base = (j is not None and
                   is_base_joint(j, j.Proxy.getAssembly(j)))
        if is_base:
            icon_tg = ":/icons/Assembly_ToggleGrounded.svg"
            return icon_tg

        return super().getIcon()

    def attach(self, vobj):
        super().attach(vobj)
        self.marker = SoJointMarker(vobj)
        self.display_mode.addChild(self.marker)

    def redrawJointPlacement(self, jcs, plc, ref):
        jcs.whichChild = coin.SO_SWITCH_NONE      # stock triad: never show
        if jcs is not getattr(self, "switch_JCS1", None):
            return
        if not ref:
            self.marker.whichChild = coin.SO_SWITCH_NONE
            return
        j = self.app_obj
        self.marker.set_kind(str(j.JointType),
                             is_base_joint(j, j.Proxy.getAssembly(j)),
                             joint_dir(j))
        self.setJCSPosition(self.marker, plc, ref)
        self.marker.whichChild = coin.SO_SWITCH_ALL

    def setPickableState(self, state):
        super().setPickableState(state)
        self.marker.setPickableState(state)
