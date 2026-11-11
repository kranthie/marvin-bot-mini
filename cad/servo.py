"""Servo mounting features, drawn in a servo's frame (cad/layout.py) and placed by its matrix.

Every joint is built from the same three features, so the whole robot shares one language:

- `cradle`: on the link carrying the case, a printed cup round it: two side walls, two end
  walls and (optionally) a floor, open on the horn side. Two M2 screws through each end wall go
  into the case's mounting holes; the far end wall is notched for the case's two bus
  connectors. Lightening windows in the side walls. With `idler`, a boss on the floor's outside,
  on the shaft axis, takes an M3 heat-set insert: the pin the far side of the joint turns on.
- `horn_plate`: on the link the joint drives, a plate on the horn's face: four M2 screws in
  counterbores into the horn, and a hole over the horn's centre screw.
- `idler_plate`: the same link's plate on the far side, with a 683 bearing (3 x 7 x 3) pressed
  in, turning on an M3 x 8 screw into the cradle's insert. A joint with both plates is a yoke,
  carried on both sides; one with only the horn plate hangs off the servo's own bearing.

The horn hole pattern and the case's mounting holes are taken from the datasheet outline and
are to be checked against a servo before printing (docs/assembly.md).
"""

from __future__ import annotations

import math

import manifold3d as mf

from .layout import CASE_D, CASE_W, CASE_X, HORN_Z, Servo
from .lib import box, circle2, cyl, hull2, plate, rect2, rounded, span, transform, union

WALL = 2.4  # cradle side walls and floor
END = 4.0  # cradle end walls: the case levers on them under torque (python -m cad.fea)
END_LIGHT = 3.0  # the STS3215s' (arms, waist, neck), at 60% of the STS3250's stall torque
FIT = 0.2  # clearance between a case and its cradle
T = 3.0  # horn and idler plates
UNDER_HEAD = 3.2  # mm of plate under a horn screw's head at most (an M2 x 6 into the horn)
GAP = 1.0  # between a cradle's floor and the idler plate (the insert boss's height)
HORN_D = 20.0  # the round horn's diameter
HORN_PCD, HORN_SCREWS = 14.0, 4  # its screw circle (to check against the horn)
EAR_Y, EAR_Z = 7.0, -15.0  # the case's mounting holes in its end faces (to check)
CONN_W, CONN_H = 14.0, 9.0  # the notch for the bus connectors in the far end wall (round-cornered)

X0, X1 = CASE_X
W2 = CASE_W / 2
IDLER_Z = -CASE_D - WALL - GAP  # the idler plate's top face

M2 = dict(clear=2.4, head=4.2, head_h=1.8, insert_d=3.2, insert_l=3.0)
M3 = dict(clear=3.4, head=6.0, head_h=2.2, insert_d=4.0, insert_l=3.0)
BEARING = dict(od=7.0, id=3.0, w=3.0)


def case(clear: float = 0.0) -> mf.Manifold:
    """The servo itself (with `clear` added all round): case, spline and horn."""
    c = clear
    body = span((X0 - c, X1 + c), (-W2 - c, W2 + c), (-CASE_D - c, c))
    horn = cyl(HORN_D + 2 * c, HORN_Z, "z", (0, 0, HORN_Z / 2))
    return union(body, horn)


def cradle(
    floor: bool = True, idler: bool = False, top: float = -1.5, end: float = END
) -> tuple[mf.Manifold, dict]:
    """The cup round a case (servo frame) and the hardware it takes."""
    w = WALL
    z0 = -CASE_D - (w if floor else 0)
    e = end
    outer = span((X0 - FIT - e, X1 + FIT + e), (-W2 - FIT - w, W2 + FIT + w), (z0, top))
    cup = outer - span((X0 - FIT, X1 + FIT), (-W2 - FIT, W2 + FIT), (-CASE_D - FIT, 50))
    # lightening windows in the side walls, between the end walls
    win = rounded(rect2(X0 + 8, X1 - 8, -CASE_D + 6, top - 5), 3.0)
    cup = cup - plate(win, "y", -50, 100)
    # the connectors' notch in the far end wall, from the floor up
    notch = rounded(rect2(-CONN_W / 2, CONN_W / 2, z0 - 8, -CASE_D + CONN_H), 3.0)
    cup = cup - plate(notch, "x", X1 - 1, e + 6)
    hw = {"M2 x 6 screw (servo case)": 4}
    # two screws at each end; at the far end above the connectors' notch
    for xe, z in ((X0 - FIT - e / 2, EAR_Z), (X1 + FIT + e / 2, -CASE_D + CONN_H + 4)):
        for y in (-EAR_Y, EAR_Y):
            cup = cup - cyl(M2["clear"], e + 4, "x", (xe, y, z))
    if idler:
        boss = cyl(7.0, GAP, "z", (0, 0, z0 - GAP / 2))
        cup = union(cup, boss) - cyl(M3["insert_d"], 2 * (w + GAP), "z", (0, 0, z0))
        hw["M3 x 3 heat-set insert"] = 1
    return cup, hw


def arm_outline(r: float, reach: list[tuple[float, float, float]]) -> mf.CrossSection:
    """A plate's outline round the shaft axis: a disc of radius r, hulled with each
    (u, v, half-width) point it must reach (servo-frame x, y)."""
    pieces = [circle2(2 * r)]
    for u, v, hw in reach:
        pieces.append(circle2(2 * hw, (u, v)))
    return hull2(*pieces)


def horn_plate(outline: mf.CrossSection, t: float = T) -> tuple[mf.Manifold, dict]:
    """A plate on the horn's face (servo frame), with its screw holes."""
    p = plate(outline, "z", HORN_Z, t)
    p = p - cyl(6.0, 3 * t, "z", (0, 0, HORN_Z))  # over the horn's centre screw

    for k in range(HORN_SCREWS):
        a = math.radians(45 + 360 * k / HORN_SCREWS)
        x, y = HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)
        p = p - cyl(M2["clear"], 3 * t, "z", (x, y, HORN_Z))
        # counterbored so an M2 x 6 still reaches the horn through a thick plate
        cb = max(M2["head_h"], t - UNDER_HEAD)
        p = p - cyl(M2["head"], 2 * cb, "z", (x, y, HORN_Z + t))
    return p, {"M2 x 6 screw (horn)": HORN_SCREWS}


def idler_plate(outline: mf.CrossSection, t: float = T) -> tuple[mf.Manifold, dict]:
    """The far-side plate (servo frame): a 683 bearing pressed into it on the shaft axis."""
    p = plate(outline, "z", IDLER_Z - t, t)
    p = p - cyl(BEARING["od"] + 0.05, 3 * t, "z", (0, 0, IDLER_Z))
    # a lip under the bearing's outer race, so it can't push through
    p = union(
        p,
        cyl(BEARING["od"] + 2, 0.8, "z", (0, 0, IDLER_Z - t - 0.4))
        - cyl(5.2, 3, "z", (0, 0, IDLER_Z - t)),
    )
    return p, {"683ZZ bearing (3 x 7 x 3)": 1, "M3 x 8 screw (idler pin)": 1}


def place(s: Servo, m: mf.Manifold) -> mf.Manifold:
    return transform(m, s.M)


def keepout(s: Servo, clear: float = 0.6) -> mf.Manifold:
    """The servo's room in world coordinates: a part of another link must stay out of it."""
    return place(s, case(clear))


def notch_box(s: Servo, x, y, z) -> mf.Manifold:
    return place(s, span(x, y, z))


__all__ = [
    "box",
    "cradle",
    "horn_plate",
    "idler_plate",
    "arm_outline",
    "case",
    "place",
    "keepout",
    "rect2",
]
