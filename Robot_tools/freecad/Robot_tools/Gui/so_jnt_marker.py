"""
so_jnt_marker.py
Per-joint-type 3D glyphs: rotation arc (revolute), double arrow (slider),
diamond (fixed), ground symbol (base). Replaces the stock JCS triad.
"""
import math
from enum import IntEnum
from typing import Sequence

from pivy import coin  # type: ignore
from SoSwitchMarker import SoSwitchMarker  # type: ignore

from freecad.Robot_tools.Gui import so_helpers as so
from freecad.Robot_tools.Gui.so_helpers import Vec

# glyph dims (marker-local units; SoShapeScale maps 1.0 -> scaleFactor px)
AXIS_LEN = 0.55                     # revolute axis half-length
RAIL_LEN = 0.8                      # slider axis half-length
ARC_R = 0.45                        # rotation arc radius
ARC_SPAN = (15.0, 315.0)            # arc start/end angles, deg
ARC_PTS = 24                        # arc smoothness
DIAMOND_R = 0.18                    # fixed-joint diamond radius
CONE_R, CONE_H = 0.07, 0.2          # arrowhead size
GROUND_Z, GROUND_W = -0.6, 0.35     # ground baseline height / half-width
GROUND_HATCH = (-0.22, 0.0, 0.22)   # hatch x positions
HATCH_DX, HATCH_DZ = 0.13, 0.17     # hatch stroke offsets
GLYPH_RGB = (0.91, 0.63, 0.09)      # accent color (arc/arrows/diamond)


class Kind(IntEnum):
    """glyph index == child index of the kinds switch"""
    REV_CCW = 0
    REV_CW = 1
    SLIDER = 2
    FIXED = 3


def kind_of(jtype: str, direction: int) -> Kind:
    """map FC JointType + jog dir to a glyph"""
    if jtype == "Revolute":
        return Kind.REV_CCW if direction >= 0 else Kind.REV_CW
    return Kind.SLIDER if jtype == "Slider" else Kind.FIXED


# pure geometry (no coin) ----------------------------------------------

def _polar(a: float, r: float) -> Vec:
    """point on the XY circle of radius r at angle a"""
    return (r * math.cos(a), r * math.sin(a), 0.0)


def arc_points(sign: int, r: float = ARC_R, n: int = ARC_PTS) -> list[Vec]:
    """rotation arc in XY; sign +1 = CCW around +Z, -1 = CW"""
    start, end = (math.radians(d) for d in ARC_SPAN)
    step = (end - start) / n
    return [_polar(sign * (start + i * step), r) for i in range(n + 1)]


def arc_tip(sign: int, r: float = ARC_R) -> tuple[Vec, Vec]:
    """(point, travel direction) at the arc end — where the arrow sits"""
    end = sign * math.radians(ARC_SPAN[1])
    point = _polar(end, r)
    travel = (-sign * math.sin(end), sign * math.cos(end), 0.0)  # tangent
    return point, travel


def ngon_points(r: float, n: int) -> list[Vec]:
    """closed regular n-gon in XY (n=4 -> diamond)"""
    pts = [_polar(2 * math.pi * i / n, r) for i in range(n)]
    return pts + pts[:1]                             # close the loop


def ground_strokes() -> list[list[Vec]]:
    """mechanics ground symbol: baseline + hatch strokes (in XZ)"""
    baseline = [(-GROUND_W, 0.0, GROUND_Z), (GROUND_W, 0.0, GROUND_Z)]
    hatches = [[(x, 0.0, GROUND_Z),
                (x - HATCH_DX, 0.0, GROUND_Z - HATCH_DZ)]
               for x in GROUND_HATCH]
    return [baseline] + hatches


# marker ----------------------------------------------------------------

class SoJointMarker(SoSwitchMarker):
    """Stock marker scaffold (transform/pick/scale/colors); our geometry."""

    def __init__(self, vobj) -> None:
        super().__init__(vobj)      # builds transform, pick, draw_style, colors
        self.removeAllChildren()    # drop the stock triad
        self.colors = {"axis": self.z_axis_so_color,   # follows user prefs
                       "glyph": so.color(GLYPH_RGB)}
        # one child per Kind, in Kind order
        self.kinds = self._switch(self._revolute(+1),
                                  self._revolute(-1),
                                  self._slider(),
                                  self._fixed())
        self._ground_align = coin.SoRotation()   # cancels world rot -> world axes
        self.ground = self._switch(so.sep(self._ground_align, self._ground()))
        self.addChild(so.group(coin.SoAnnotation(),    # render on top
                               self.transform,
                               self.pick,
                               self.kinds,
                               self.ground))
        self.whichChild = coin.SO_SWITCH_NONE

    def set_kind(self, jtype: str, is_base: bool, direction: int) -> None:
        """pick glyph + ground overlay; placement stays stock's job"""
        self.kinds.whichChild = kind_of(jtype, direction)
        self.ground.whichChild = (coin.SO_SWITCH_ALL if is_base
                                  else coin.SO_SWITCH_NONE)

    # scene graph, on so_helpers ------------------------------------------

    def _switch(self, *glyphs: coin.SoNode) -> coin.SoSwitch:
        """switch of screen-scaled glyphs, initially all hidden"""
        sw = so.group(coin.SoSwitch(),
                      *(so.scaled(g, self.scaleFactor) for g in glyphs))
        sw.whichChild = coin.SO_SWITCH_NONE
        return sw

    def _line(self, pts: Sequence[Vec], color: str = "glyph") -> coin.SoSeparator:
        """polyline in stock line style; color = key into self.colors"""
        return so.polyline(pts, self.draw_style, self.colors[color])

    def _arrowhead(self, at: Vec, along: Vec) -> coin.SoSeparator:
        """glyph arrowhead (shaftless arrow)"""
        return so.sep(self.colors["glyph"],
                      so.arrow(along, CONE_R, CONE_H, at=at))

    def _axis(self, half: float) -> coin.SoSeparator:
        """joint axis segment along local Z"""
        return self._line([(0, 0, -half), (0, 0, half)], "axis")

    # glyphs ---------------------------------------------------------------

    def _revolute(self, sign: int) -> coin.SoSeparator:
        """axis + rotation arc + travel arrow; sign flips CCW/CW"""
        tip, travel = arc_tip(sign)
        return so.sep(self._axis(AXIS_LEN),
                      self._line(arc_points(sign)),
                      self._arrowhead(tip, travel))

    def _slider(self) -> coin.SoSeparator:
        """rail axis with an arrowhead at each end"""
        return so.sep(self._axis(RAIL_LEN),
                      self._arrowhead((0, 0, RAIL_LEN), (0, 0, 1)),
                      self._arrowhead((0, 0, -RAIL_LEN), (0, 0, -1)))

    def _fixed(self) -> coin.SoSeparator:
        """small diamond = rigid"""
        return self._line(ngon_points(DIAMOND_R, 4))

    def _ground(self) -> coin.SoSeparator:
        """ground symbol under the axis marks the base joint"""
        return so.sep(*(self._line(s, "axis")
                        for s in ground_strokes()))

    def set_marker_placement(self, placement, ref):
        """stock placement; ground glyph stays world-aligned (world -Z = down)"""
        super().set_marker_placement(placement, ref)
        if self.ground.whichChild == coin.SO_SWITCH_NONE:
            return                              # not a base joint
        world_q = self.transform.rotation.getValue()
        self._ground_align.rotation.setValue(world_q.inverse())
