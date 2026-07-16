"""
so_helpers.py
Shared Coin3D scene-graph builders (joint glyphs + TCP dragger).
Pure construction — callers own all color/material/pick state nodes.
"""
from typing import Optional, Sequence

from pivy import coin

Vec = tuple[float, float, float]


def group(node: coin.SoGroup, *children: coin.SoNode) -> coin.SoGroup:
    """fill any group-ish node with children, return it"""
    for child in children:
        node.addChild(child)
    return node


def sep(*children: coin.SoNode) -> coin.SoSeparator:
    """separator around children"""
    return group(coin.SoSeparator(), *children)


def color(rgb: Vec) -> coin.SoBaseColor:
    """flat color (lines)"""
    col = coin.SoBaseColor()
    col.rgb.setValue(*rgb)
    return col


def material(rgb: Vec) -> coin.SoMaterial:
    """shaded material (3D solids)"""
    mat = coin.SoMaterial()
    mat.diffuseColor.setValue(*rgb)
    return mat


def scaled(node: coin.SoNode, factor: float) -> coin.SoNode:
    """constant-screen-size wrapper (stock SoShapeScale trick)"""
    kit = coin.SoType.fromName("SoShapeScale").createInstance()
    kit.setPart("shape", node)
    kit.scaleFactor = factor
    return kit


def transform(at: Vec = (0, 0, 0),
              rot_to: Optional[Vec] = None) -> coin.SoTransform:
    """translation + optional rotation taking local +Y onto rot_to"""
    tr = coin.SoTransform()
    tr.translation.setValue(*at)
    if rot_to is not None:
        y_axis = coin.SbVec3f(0, 1, 0)
        tr.rotation.setValue(coin.SbRotation(y_axis, coin.SbVec3f(*rot_to)))
    return tr


def polyline(pts: Sequence[Vec], *state: coin.SoNode) -> coin.SoSeparator:
    """one line strip through pts; state = caller's style/color nodes"""
    coords = coin.SoCoordinate3()
    coords.point.setValues(0, list(pts))
    strip = coin.SoLineSet()
    strip.numVertices.setValue(len(pts))
    return sep(*state, coords, strip)


def _at_y(shape: coin.SoNode, y: float) -> coin.SoSeparator:
    """isolate shape in its own separator, centered at height y on local +Y"""
    return sep(transform((0, y, 0)), shape)


def arrow(along: Vec, tip_r: float, tip_h: float,
          shaft_len: float = 0.0, shaft_r: float = 0.0,
          at: Vec = (0, 0, 0)) -> coin.SoSeparator:
    """arrow from `at` along `along`: optional shaft [0..len], cone tip after.
    shaft_len=0 -> bare glyph arrowhead; >0 -> dragger axis handle."""
    sep_ = sep(transform(at, along))
    if shaft_len > 0:
        shaft = coin.SoCylinder()
        shaft.radius, shaft.height = shaft_r, shaft_len
        sep_.addChild(_at_y(shaft, shaft_len / 2))    # SoCylinder is centered
    tip = coin.SoCone()
    tip.bottomRadius, tip.height = tip_r, tip_h
    sep_.addChild(_at_y(tip, shaft_len + tip_h / 2))  # sits right after shaft
    return sep_