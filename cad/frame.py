"""The printed frame: one part per link, built from the servo features in cad/servo.py.

Drawn in the world frame at the zero pose (mm, left side and centre; cad/layout.py), moved into
each link's frame at the end. Every joint is a cradle on one side and a horn plate (and, where
the joint is carried on both sides, an idler plate with a bearing) on the other, so the robot
shares one set of details: 3 mm plates with 4 mm rounded corners, 2.4 mm cradle walls,
lightening windows with 3 mm corners, M2 screws into the servos, M3 pins into heat-set inserts.

Load-carrying leg and hip parts are nylon-carbon; the arms, torso and head PETG.

Wiring: each part leaves a passage for its bus cable (the cradles' connector notches face the
next servo along the chain) and carries pairs of cable-tie slots either side of each joint for
the service loop (cad/harness.py lists the chain, the slots and the cable lengths).
"""

from __future__ import annotations

import numpy as np

from . import servo as sv
from .layout import JOINTS, ORIGIN, RANGES, SERVOS, to_link
from .lib import (
    Part,
    box,
    circle2,
    cyl,
    half,
    hull,
    hull2,
    plate,
    poly,
    rect2,
    rounded,
    span,
    transform,
    union,
    windowed,
)

T = sv.T
HUB_R = 26.0  # mm, the torso's thick hub round the waist horn
SKIRT_R = 11.0  # mm, the inside of a horn plate's skirt (1 mm clear of the horn)
T_LEG = 4.0  # the legs' horn and idler plates: they carry the landing and the servos' stall torque


def S(j: str):
    return SERVOS[j]


def local_uv(s, p_world) -> tuple[float, float]:
    """A world point's (x, y) in a servo's frame: where a plate on that servo must reach."""
    q = np.linalg.solve(s.M, np.r_[np.asarray(p_world, float), 1.0])
    return float(q[0]), float(q[1])


def outline(s, r: float, reach: list, round_r: float = 4.0):
    """A plate outline round servo s's shaft: a disc of radius r hulled with circles of given
    radius at world points [(point, radius)]."""
    pieces = [circle2(2 * r)]
    for p, rr in reach:
        pieces.append(circle2(2 * rr, local_uv(s, p)))
    return rounded(hull2(*pieces), round_r)


def tie_slots(at, normal: str, along: str, gap: float = 7.0):
    """A pair of 2.2 x 4.2 cable-tie slots through a plate (normal along `normal`), `gap` apart
    along `along`, centred at world point `at`: where a service loop is tied."""
    cuts = []
    for s in (-1, 1):
        c = np.array(at, float)
        c["xyz".index(along)] += s * gap / 2
        size = [4.2, 4.2, 4.2]
        size["xyz".index(along)] = 2.2
        size["xyz".index(normal)] = 20
        cuts.append(box(size, c))
    return union(*cuts)


def add_hw(*ds) -> dict:
    out: dict = {}
    for d in ds:
        for k, v in d.items():
            out[k] = out.get(k, 0) + v
    return out


# The contact patches of the part being built (world, zero pose): where it meets each servo, for
# the stress check (cad/fea.py). Each part function starts with begin() and ends with ifaces().
_IF: list = []


def begin():
    _IF.clear()


def _patch(j, kind, role, s, centre_s, axis, r0=0.0, r1=0.0, span=(-1.0, 1.0)):
    _IF.append(
        dict(
            joint=j,
            kind=kind,
            role=role,
            centre=[float(v) for v in s.at(centre_s)],
            axis=[float(v) for v in axis],
            r0=r0,
            r1=r1,
            span=list(span),
        )
    )


def ifaces(link: str) -> dict:
    """The patches recorded since begin(), in the link's frame."""
    from .layout import ORIGIN

    o = ORIGIN[link]
    return {
        "ifaces": [
            dict(i, centre=[c - oo for c, oo in zip(i["centre"], o, strict=True)]) for i in _IF
        ]
    }


def cradle(j, floor=True, idler=False):
    s = S(j)
    end = sv.END if s.model == "STS3250" else sv.END_LIGHT
    c, hw = sv.cradle(floor=floor, idler=idler, end=end)
    # the case is held by the screws through the end walls (and the idler pin's insert)
    x_axis = s.M[:3, 0]
    w2 = end / 2
    for xe, z in (
        (sv.X0 - sv.FIT - w2, sv.EAR_Z),
        (sv.X1 + sv.FIT + w2, -sv.CASE_D + sv.CONN_H + 4),
    ):
        for y in (-sv.EAR_Y, sv.EAR_Y):
            _patch(j, "cyl", "case", s, (xe, y, z), x_axis, r1=sv.M2["clear"] / 2, span=(-w2, w2))
    # and bears on the cradle's walls: their inner faces (the case is far stiffer than a print)
    zc = -sv.CASE_D / 2
    for ys in (-1, 1):
        _patch(
            j,
            "face",
            "case",
            s,
            ((sv.X0 + sv.X1) / 2, ys * (sv.W2 + sv.FIT), zc),
            s.M[:3, 1],
            r1=12.0,
        )
    for xe in (sv.X0 - sv.FIT, sv.X1 + sv.FIT):
        _patch(j, "face", "case", s, (xe, 0, zc), x_axis, r1=12.0)
    if idler:
        z0 = -sv.CASE_D - sv.WALL
        _patch(
            j,
            "cyl",
            "case",
            s,
            (0, 0, z0),
            s.axis_w,
            r1=sv.M3["insert_d"] / 2,
            span=(-sv.GAP, sv.WALL),
        )
    return sv.place(s, c), hw


def horn(j, r, reach, t=T, patch=True):
    """A horn plate on servo j; `patch=False` for a second, thicker piece of the same plate."""
    s = S(j)
    p, hw = sv.horn_plate(outline(s, r, reach), t)
    if patch:
        _patch(j, "face", "horn", s, (0, 0, sv.HORN_Z), s.axis_w, r0=3.0, r1=sv.HORN_D / 2)
    return sv.place(s, p), hw


def skirt(j, r, reach, gap=1.5):
    """A skirt under a horn plate (same outline), down from the horn's face to `gap` above the
    servo's top face, open round the horn: the plate's depth where a moment across it bends it
    round the horn."""
    s = S(j)
    cs = outline(s, r, reach) - circle2(2 * SKIRT_R)
    return sv.place(s, plate(cs, "z", gap, sv.HORN_Z - gap))


def idler(j, r, reach, t=T):
    s = S(j)
    p, hw = sv.idler_plate(outline(s, r, reach), t)
    _patch(
        j,
        "cyl",
        "bearing",
        s,
        (0, 0, sv.IDLER_Z - t / 2),
        s.axis_w,
        r1=sv.BEARING["od"] / 2 + 0.025,
        span=(-t / 2, t / 2),
    )
    return sv.place(s, p), hw


def swept(m, joint: str, clear: float = 1.0, step: float = 3.0, child: bool = False):
    """Where a part on a joint's parent link goes in its child link's frame (world axes, the
    joint at its zero pose) as the joint turns through its range, grown by about `clear`; with
    `child`, where a part on the child link goes in the parent's frame."""
    j = JOINTS[joint]
    lo, hi = RANGES[joint] if child else [-q for q in reversed(RANGES[joint])]
    pivot = ORIGIN[j["child"]]
    axis = np.asarray(j["axis"], float)
    grown = union(*[m.translate(list(clear * d)) for d in np.vstack([np.eye(3), -np.eye(3)])])
    out = []
    for q in np.arange(lo, hi + step / 2, step):
        R = np.eye(4)
        c, s = np.cos(np.radians(q)), np.sin(np.radians(q))
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        R[:3, :3] = np.eye(3) + s * k + (1 - c) * k @ k
        R[:3, 3] = pivot - R[:3, :3] @ pivot
        out.append(transform(grown, R))
    return union(*out)


def finish(m, keepouts, keep=()):
    """The part clear of the servos near it, then its own cradles added back whole (the
    keepouts' clearance would otherwise thin their walls round their own servos)."""
    for k in keepouts:
        m = m - k
    return union(m, *keep) if keep else m


# ------------------------------------------------------------------------------- pelvis


def pelvis() -> Part:
    # the hip roll bracket's swing, which the gussets below the pelvis keep clear of
    hrb = hip_roll_bracket()
    leg = swept(hrb.solid.translate(list(ORIGIN[hrb.link])), "left_hip_roll", child=True)
    begin()
    w, hip = S("waist_yaw"), S("left_hip_roll")
    c_w, hw_w = cradle("waist_yaw")
    c_l, hw_l = cradle("left_hip_roll", idler=True)
    c_r = c_l.mirror([0, 1, 0])
    # the deck joins the three cradles at their tops; open round the waist horn
    deck_cs = rounded(rect2(-56.0, 14.0, -60.0, 60.0), 6.0)
    # 6 mm, so it overlaps the hip roll cradles' top end walls all through
    deck = plate(windowed(deck_cs, 14.0, 4.0), "z", 31.7, 6.0)
    deck = union(deck, plate(rounded(rect2(-40.0, 14.0, -18.0, 18.0), 4.0), "z", 31.7, 6.0))
    # a front plate for the IMU and the leg bus adapters, between the legs' swing
    front = plate(windowed(rounded(rect2(-14.0, 14.0, -4.0, 31.0), 4.0), 6.0), "x", 13.7, 2.6)
    rib = span((-34.0, 14.0), (-1.5, 1.5), (16.0, 34.7))
    # front and back walls across the hips, above the hip roll horns and idlers: with the deck,
    # a box that carries a landing's moments from one hip to the other and to the waist
    # (python -m cad.fea; the cradles hanging from the deck alone bent at their tops)
    walls = union(
        span((-26.2, -21.2), (-60.0, 60.0), (12.5, 37.7)),
        span((-26.2, -21.2), (-30.0, 30.0), (3.0, 12.5)),
        # below, between the waist cradle and the hip roll cradles, the hip loads' way to the
        # waist: a channel (two walls and a floor), not one wall
        span((-37.0, -19.2), (-31.0, 31.0), (3.0, 6.0)),
        span((-37.0, -33.0), (-31.0, 31.0), (3.0, 12.5)),
        span((-28.2, -19.2), (-31.0, 31.0), (3.0, 12.5)),
        span((-53.0, -48.0), (-60.0, 60.0), (12.5, 37.7)),
    )
    # gussets under the channel, down the hip roll cradles' inner walls
    gusset = plate(poly([(13.0, 3.0), (31.0, -15.0), (31.0, 3.0)]), "x", -37.0, 17.8)
    # and the waist cradle's sides thickened, with fillets where the channel's front wall meets
    # them: the sides carry the hips' loads to the waist servo's screws in the cradle's ends
    fillet = plate(poly([(-19.5, 14.0), (-3.0, 14.0), (-19.5, 20.0)]), "z", 3.8, 18.0)
    pad = span((-37.35, 14.35), (14.0, 17.4), (3.8, 32.0))  # up into the deck
    gusset = union(gusset, fillet, pad) - leg
    gusset = union(gusset, gusset.mirror([0, 1, 0]))
    m = union(c_w, c_l, c_r, deck, front, rib, walls, gusset)
    m = finish(
        m, [sv.keepout(w), sv.keepout(hip), sv.keepout(hip).mirror([0, 1, 0])], keep=[c_w, c_l, c_r]
    )
    m = m - tie_slots((-10, 0, 33.2), "z", "x") - tie_slots((-30, 22, 33.2), "z", "x")
    m = m - tie_slots((-30, -22, 33.2), "z", "x")
    hw = add_hw(hw_w, hw_l, hw_l)
    return Part("pelvis", "pelvis", "pa_cf", to_link(m, "pelvis"), "leg_frame", hardware=hw, step=1)


# --------------------------------------------------------------------------------- legs


def hip_roll_bracket() -> Part:
    begin()
    h, p = S("left_hip_roll"), S("left_hip_pitch")
    below = (-30.0, 45.0, -23.0)
    hp_c, hw1 = cradle("left_hip_pitch", idler=True)
    hn, hw2 = horn(
        "left_hip_roll",
        12.0,
        [((-14.6, 45, -22.6), 12.0), ((-14.6, 31, -14.0), 2.0), ((-14.6, 59, -14.0), 2.0)]
        + [((-14.6, 31, 14.0), 2.0), ((-14.6, 59, 14.0), 2.0)],
        5.0,  # it carries the leg: as thick as the room to the hip pitch servo allows
    )
    idl, hw3 = idler("left_hip_roll", 11.0, [((-58.8, 45, -22.6), 12.0)], T_LEG)
    bridge = span((-60.3, -12.1), (33.0, 57.0), (-25.1, -22.1))
    bridge = bridge - span((-50, -20), (37.0, 53.0), (-30, -20))  # lightening slot
    m = union(hp_c, hn, idl, bridge)
    m = finish(m, [sv.keepout(h), sv.keepout(p)], keep=[hp_c])
    m = m - tie_slots(below, "z", "y", 8.0)
    return Part(
        "left_hip_roll_bracket",
        "left_hip_roll_link",
        "pa_cf",
        to_link(m, "left_hip_roll_link"),
        "leg_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=2,
    )


def hip_pitch_bracket() -> Part:
    begin()
    p, y = S("left_hip_pitch"), S("left_hip_yaw")
    reach = [((0, 45, -37.0), 8.0), ((-14, 45, -43), 2.0), ((14, 45, -43), 2.0)]
    hn, hw1 = horn("left_hip_pitch", 12.0, reach, T_LEG)
    idl, hw2 = idler("left_hip_pitch", 14.0, reach, T_LEG + 1.0)  # the hip yaw's stall torque
    # the floor reaches the arms across its width (narrow necks there broke under a landing),
    # and is 6 mm where the hip pitch cradle leaves room for it as the hip turns (at 4 mm it bent
    # round the hip yaw horn under a landing)
    pts = [((x, y, -42.5), 3.0) for x in (-12.0, 12.0) for y in (26.0, 67.0)]
    floor, hw3 = horn("left_hip_yaw", 14.0, pts, T_LEG)
    thick, _ = horn("left_hip_yaw", 14.0, pts, 6.0, patch=False)
    floor = union(floor, thick - swept(cradle("left_hip_pitch", idler=True)[0], "left_hip_pitch"))
    # and a skirt below it round the horn: the depth a landing's fore-aft moment bends the
    # floor across
    m = union(hn, idl, floor, skirt("left_hip_yaw", 14.0, pts))
    m = finish(m, [sv.keepout(p), sv.keepout(y)])
    return Part(
        "left_hip_pitch_bracket",
        "left_hip_pitch_link",
        "pa_cf",
        to_link(m, "left_hip_pitch_link"),
        "leg_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=3,
    )


def thigh() -> Part:
    begin()
    y, k = S("left_hip_yaw"), S("left_knee")
    c_y, hw1 = cradle("left_hip_yaw")
    c_k, hw2 = cradle("left_knee", idler=True)
    web = span((-15.0, 13.7), (29.0, 58.5), (-84.0, -81.0))
    # a back spine carrying the leg bus down to the knee, with its tie slots
    spine = plate(rounded(rect2(30.0, 58.5, -132.0, -52.0), 3.0), "x", -16.5, 2.4)
    m = union(c_y, c_k, web, spine)
    m = finish(m, [sv.keepout(y), sv.keepout(k)], keep=[c_y, c_k])
    m = m - tie_slots((-15.3, 44, -60), "x", "y") - tie_slots((-15.3, 44, -100), "x", "y")
    return Part(
        "left_thigh",
        "left_thigh_link",
        "pa_cf",
        to_link(m, "left_thigh_link"),
        "leg_frame",
        hardware=add_hw(hw1, hw2),
        step=4,
    )


def shin() -> Part:
    begin()
    k, a = S("left_knee"), S("left_ankle_pitch")
    reach = [((0, 45, -212.0), 13.0)]
    hn, hw1 = horn("left_knee", 12.0, reach, T_LEG)
    idl, hw2 = idler("left_knee", 12.0, reach, T_LEG)
    c_a, hw3 = cradle("left_ankle_pitch", idler=True)
    webs = union(
        span((10.0, 14.9), (24.0, 68.8), (-216.0, -204.0)),
        span((-16.9, -10.0), (24.0, 68.8), (-216.0, -204.0)),  # the ankle's stall torque
    )
    # the shin's front face: a plate between the two arms, below the knee's swing
    face = plate(windowed(rounded(rect2(26.0, 66.0, -205.0, -150.0), 4.0), 7.0), "x", 11.0, 2.4)
    m = union(hn, idl, c_a, webs, face)
    m = finish(m, [sv.keepout(k), sv.keepout(a)], keep=[c_a])
    m = m - tie_slots((12.2, 46, -160), "x", "y")
    return Part(
        "left_shin",
        "left_shin_link",
        "pa_cf",
        to_link(m, "left_shin_link"),
        "leg_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=5,
    )


def ankle_bracket() -> Part:
    begin()
    a, r = S("left_ankle_pitch"), S("left_ankle_roll")
    reach = [((32.0, 45, -262.0), 14.0)]
    hn, hw1 = horn("left_ankle_pitch", 12.0, reach, T_LEG)
    idl, hw2 = idler("left_ankle_pitch", 12.0, reach, T_LEG)
    c_r, hw3 = cradle("left_ankle_roll", idler=True)
    web = span((18.0, 40.0), (23.6, 33.0), (-276.0, -248.0))
    m = union(hn, idl, c_r, web)
    m = finish(m, [sv.keepout(a), sv.keepout(r)], keep=[c_r])
    return Part(
        "left_ankle_bracket",
        "left_ankle_link",
        "pa_cf",
        to_link(m, "left_ankle_link"),
        "leg_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=6,
    )


SOLE_Z = -290.0  # the sole, below the ankle pitch axis by robot.yaml's ankle_height


def foot() -> Part:
    begin()
    r = S("left_ankle_roll")
    z0 = SOLE_Z + 2.0  # on the 2 mm TPU sole
    cs = rounded(rect2(-38.0, 62.0, 17.5, 72.5), 8.0)
    # a 6 mm plate with ribs under the lugs and along the foot: a landing's three body weights go
    # from the sole into the two lugs (python -m cad.fea; the first 4 mm plate broke there)
    base = plate(windowed(cs, 11.0, 5.0), "z", z0, 6.0)
    # (the side ribs stop short of the stretch under the roll servo, which they'd meet rolling)
    ribs = union(
        plate(rect2(-38.0, 62.0, 43.5, 46.5), "z", z0, 8.0),
        plate(rounded(rect2(-30.0, 14.0, 43.5, 46.5), 1.4), "z", z0, 12.0),  # deeper behind the lug
        *[plate(rect2(-38.0, 12.0, y - 1.5, y + 1.5), "z", z0, 8.0) for y in (31.0, 59.0)],
        plate(rect2(12.0, 18.0, 17.5, 72.5), "z", z0, 8.0),
        plate(rect2(56.0, 61.0, 30.0, 60.0), "z", z0, 8.0),
    )
    reach = [((0, 45, z0 + 6.0), 1.0), ((0, 30.0, z0 + 6.0), 5.0), ((0, 64.0, z0 + 6.0), 5.0)]
    hn, hw1 = horn("left_ankle_roll", 12.0, reach, T_LEG)
    idl, hw2 = idler("left_ankle_roll", 10.0, reach, T_LEG)
    # gussets from the ribs up the idler lug's back, where a landing bent it off the plate
    gussets = union(
        *[
            plate(poly([(-2.0, z0 + 8.0), (15.0, z0 + 8.0), (15.0, z0 + 22.0)]), "y", y - 1.5, 3.0)
            if y != 45.0
            else plate(
                poly([(-30.0, z0 + 12.0), (15.0, z0 + 12.0), (15.0, z0 + 27.0)]), "y", y - 1.5, 3.0
            )
            for y in (31.0, 45.0, 59.0)
        ]
    )
    m = union(base, ribs, gussets, hn, idl)
    # bevel the plate's outer top edge under the roll servo, so it clears the case as the foot
    # rolls (robot.yaml: ankle roll +-15°)
    cut = hull(
        span((15.0, 52.0), (59.0, 59.1), (z0 + 6.0, z0 + 8.1)),
        span((15.0, 52.0), (72.4, 72.5), (z0 + 2.2, z0 + 8.1)),
        span((15.0, 52.0), (72.4, 80.0), (z0 + 8.0, z0 + 10.0)),
    )
    m = m - cut
    m = finish(m, [sv.keepout(r)])
    return Part(
        "left_foot",
        "left_foot_link",
        "pa_cf",
        to_link(m, "left_foot_link"),
        "leg_frame",
        hardware=add_hw(hw1, hw2),
        step=7,
    )


def foot_sole() -> Part:
    cs = rounded(rect2(-38.0, 62.0, 17.5, 72.5), 8.0)
    m = plate(cs, "z", SOLE_Z, 2.0)
    return Part(
        "left_foot_sole",
        "left_foot_link",
        "tpu",
        to_link(m, "left_foot_link"),
        "leg_frame",
        notes="glued to the foot",
        step=7,
    )


# -------------------------------------------------------------------------------- torso

TORSO_SIDE_Y = 51.0  # inner face of the torso's side frames (the computer is 100 mm wide)


def torso() -> Part:
    begin()
    w, n, sp = S("waist_yaw"), S("neck_yaw"), S("left_shoulder_pitch")
    # an 8 mm hub round the waist horn, with a skirt over the pelvis's deck: an arm's stall
    # torque reaches the waist horn through the floor (at 3 mm it bent round the horn)
    reach = [((x, y, 43), 4.0) for x in (24.0, -30.0) for y in (40.0, -40.0)]
    floor, hw1 = horn("waist_yaw", 14.0, reach)
    hub, _ = horn("waist_yaw", HUB_R, [], 8.0, patch=False)
    floor = union(
        floor,
        hub,
        skirt("waist_yaw", HUB_R, [], 2.5),
        plate(
            rounded(rect2(-34.0, 28.0, -TORSO_SIDE_Y - 3, TORSO_SIDE_Y + 3), 6.0), "z", 42.0, 3.0
        ),
    )
    floor = floor - plate(rounded(rect2(-26.0, -12.0, -40.0, 40.0), 4.0), "z", 40.0, 10.0)
    # the front edge slants back above the chest, inside the shell's bevelled front corners
    side_cs = rounded(
        poly([(-34.0, 42.0), (28.0, 42.0), (28.0, 150.0), (8.0, 212.0), (-34.0, 212.0)]), 6.0
    )
    side_cs = side_cs - rounded(rect2(-27.0, 21.0, 52.0, 140.0), 4.0)
    sides = union(
        plate(side_cs, "y", TORSO_SIDE_Y, 3.0), plate(side_cs, "y", -TORSO_SIDE_Y - 3.0, 3.0)
    )
    top_cs = poly(
        [(-34.0, -54.0), (8.0, -54.0), (28.0, -34.0), (28.0, 34.0), (8.0, 54.0), (-34.0, 54.0)]
    )
    top = plate(windowed(rounded(top_cs, 6.0), 9.0), "z", 209.0, 2.5)
    back_cs = windowed(rounded(rect2(-TORSO_SIDE_Y - 3, TORSO_SIDE_Y + 3, 42.0, 212.0), 6.0), 9.0)
    back = plate(back_cs, "x", -34.0, 2.5)
    # the middle plate: the computer stands on four standoffs in front of it, the battery is
    # strapped behind it
    mid_cs = rounded(rect2(-TORSO_SIDE_Y, TORSO_SIDE_Y, 45.0, 150.0), 4.0)
    mid = plate(mid_cs - rounded(rect2(-34.0, 34.0, 62.0, 133.0), 6.0), "x", -9.5, 2.0)
    standoffs = union(
        *[
            cyl(6.0, 4.5, "x", (-4.75, yy, zz)) - cyl(2.2, 6.0, "x", (-4.75, yy, zz))
            for yy in (-43.0, 43.0)
            for zz in (72.0, 140.0)
        ]
    )
    c_n, hw2 = cradle("neck_yaw")
    c_sl, hw3 = cradle("left_shoulder_pitch")
    c_sr = c_sl.mirror([0, 1, 0])
    # a front rib across the floor, side to side: with the back plate and the middle plate, it
    # carries an arm's stall torque from the side frames to the waist hub (python -m cad.fea);
    # on the sides' solid front edges, in front of their windows
    rib = span((20.0, 26.0), (-TORSO_SIDE_Y - 1.0, TORSO_SIDE_Y + 1.0), (42.0, 62.0))
    knee = plate(
        poly([(TORSO_SIDE_Y - 12.0, 61.0), (TORSO_SIDE_Y + 1.0, 61.0), (TORSO_SIDE_Y + 1.0, 82.0)]),
        "x",
        20.0,
        6.0,
    )
    rib = union(rib, knee, knee.mirror([0, 1, 0]))  # gussets up the sides from its ends
    m = union(floor, rib, sides, top, back, mid, standoffs, c_n, c_sl, c_sr)
    m = finish(
        m,
        [
            sv.keepout(w),
            sv.keepout(n),
            sv.keepout(sp),
            sv.keepout(sp).mirror([0, 1, 0]),
            span((-7.0, 30.0), (-50.5, 50.5), (65.0, 145.0)),  # the computer
        ],
        keep=[c_n, c_sl, c_sr],
    )
    # strap slots for the battery, inserts for the shell's screws
    for zz in (80.0, 128.0):
        m = m - span((-11.0, -8.0), (-36.0, -30.0), (zz - 1.5, zz + 1.5))
        m = m - span((-11.0, -8.0), (30.0, 36.0), (zz - 1.5, zz + 1.5))
    m = m - tie_slots((-20, 0, 210.5), "z", "y") - tie_slots((-20, 0, 43.5), "z", "y")
    hw = add_hw(
        hw1, hw2, hw3, hw3, {"M2.5 x 6 screw (computer)": 4, "hook-and-loop strap (battery)": 2}
    )
    return Part(
        "torso", "torso_link", "petg", to_link(m, "torso_link"), "torso_frame", hardware=hw, step=8
    )


# --------------------------------------------------------------------------------- arms


def shoulder_bracket() -> Part:
    begin()
    sp, sr = S("left_shoulder_pitch"), S("left_shoulder_roll")
    hn, hw1 = horn("left_shoulder_pitch", 13.0, [((-2, 70, 212.0), 12.0), ((-2, 70, 180.0), 12.0)])
    c_r, hw2 = cradle("left_shoulder_roll", idler=True)
    web = span((-17.0, 12.0), (68.0, 81.0), (178.0, 214.0))
    web = web - span((-10, 5), (60, 90), (186, 206))
    m = union(hn, c_r, web)
    m = finish(m, [sv.keepout(sp), sv.keepout(sr)], keep=[c_r])
    return Part(
        "left_shoulder_bracket",
        "left_shoulder_pitch_link",
        "petg",
        to_link(m, "left_shoulder_pitch_link"),
        "arm_frame",
        hardware=add_hw(hw1, hw2),
        step=9,
    )


def shoulder_yoke() -> Part:
    begin()
    sr, sy = S("left_shoulder_roll"), S("left_shoulder_yaw")
    reach = [((0, 95, 165.0), 8.0), ((0, 81.5, 157.0), 1.0), ((0, 108.5, 157.0), 1.0)]
    hn, hw1 = horn("left_shoulder_roll", 12.0, reach)
    idl, hw2 = idler("left_shoulder_roll", 12.0, reach)
    # 5 mm: a hand catch with the arm out bent it round the shoulder yaw horn at 3 mm
    bottom, hw3 = horn(
        "left_shoulder_yaw", 13.0, [((17.0, 95, 157), 11.0), ((-20.0, 95, 157), 13.0)], 5.0
    )
    m = union(hn, idl, bottom)
    m = finish(m, [sv.keepout(sr), sv.keepout(sy)])
    return Part(
        "left_shoulder_yoke",
        "left_shoulder_roll_link",
        "petg",
        to_link(m, "left_shoulder_roll_link"),
        "arm_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=10,
    )


def upper_arm() -> Part:
    begin()
    y, e = S("left_shoulder_yaw"), S("left_elbow")
    c_y, hw1 = cradle("left_shoulder_yaw")
    c_e, hw2 = cradle("left_elbow", idler=True)
    walls = union(
        plate(rounded(rect2(82.0, 108.0, 103.0, 120.0), 2.0), "x", 11.2, 2.5),
        plate(rounded(rect2(82.0, 108.0, 103.0, 120.0), 2.0), "x", -36.7, 2.5),
    )
    m = union(c_y, c_e, walls)
    m = finish(m, [sv.keepout(y), sv.keepout(e)], keep=[c_y, c_e])
    return Part(
        "left_upper_arm",
        "left_upper_arm_link",
        "petg",
        to_link(m, "left_upper_arm_link"),
        "arm_frame",
        hardware=add_hw(hw1, hw2),
        step=11,
    )


def forearm() -> Part:
    begin()
    e, wy = S("left_elbow"), S("left_wrist_yaw")
    reach = [((-2.0, 95, 47.0), 14.0)]
    hn, hw1 = horn("left_elbow", 12.0, reach)
    idl, hw2 = idler("left_elbow", 12.0, reach)
    c_w, hw3 = cradle("left_wrist_yaw")
    webs = union(
        span((-12.0, 10.0), (108.0, 116.5), (40.0, 50.0)),
        span((-12.0, 10.0), (75.5, 82.0), (40.0, 50.0)),
    )
    m = union(hn, idl, c_w, webs)
    m = finish(m, [sv.keepout(e), sv.keepout(wy)], keep=[c_w])
    return Part(
        "left_forearm",
        "left_forearm_link",
        "petg",
        to_link(m, "left_forearm_link"),
        "arm_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=12,
    )


FINGER_Y = (87.0, 103.0)  # the fingers' width
TIP_Z = 10.0 - 80.0  # fingertips: robot.yaml's hand below the wrist's output face
JAW_Z = -57.0  # the moving jaw's bridge, clear of the gripper's cradle as the jaw opens


def palm() -> Part:
    begin()
    wy, g = S("left_wrist_yaw"), S("left_gripper")
    top, hw1 = horn("left_wrist_yaw", 13.0, [((0, 80.5, 8.0), 13.0), ((0, 109.0, 8.0), 13.0)])
    c_g, hw2 = cradle("left_gripper")
    pad = span((-14.0, 14.0), (78.0, 109.0), (5.5, 7.5))
    # the fixed jaw: from the cradle's lower end down to the fingertip, its gripping face at x -1
    jaw = hull(
        rounded_box((-9.0, -1.0), FINGER_Y, (-46.0, -44.0)),
        rounded_box((-6.0, -1.0), (FINGER_Y[0] + 2, FINGER_Y[1] - 2), (TIP_Z, TIP_Z + 2)),
    )
    stem = span((-14.0, -1.0), (79.0, 108.0), (-47.0, -42.0))
    m = union(top, c_g, pad, jaw, stem)
    m = finish(m, [sv.keepout(wy), sv.keepout(g)], keep=[c_g])
    return Part(
        "left_palm",
        "left_palm_link",
        "petg",
        to_link(m, "left_palm_link"),
        "arm_frame",
        hardware=add_hw(hw1, hw2, {"TPU finger pad (glued)": 1}),
        step=13,
    )


def jaw() -> Part:
    begin()
    g = S("left_gripper")
    arm, hw1 = horn("left_gripper", 11.0, [((8.0, 110.0, JAW_Z + 3), 6.0)])
    bridge = span((1.0, 13.0), (FINGER_Y[0], 119.0), (JAW_Z, JAW_Z + 6))
    finger = hull(
        rounded_box((1.0, 9.0), FINGER_Y, (JAW_Z, JAW_Z + 2)),
        rounded_box((1.0, 6.0), (FINGER_Y[0] + 2, FINGER_Y[1] - 2), (TIP_Z, TIP_Z + 2)),
    )
    m = union(arm, bridge, finger)
    m = finish(m, [sv.keepout(g)])
    return Part(
        "left_jaw",
        "left_jaw_link",
        "petg",
        to_link(m, "left_jaw_link"),
        "arm_frame",
        hardware=add_hw(hw1, {"TPU finger pad (glued)": 1}),
        step=13,
    )


def rounded_box(x, y, z):
    return span(x, y, z)


# --------------------------------------------------------------------------------- head


def neck_yoke() -> Part:
    begin()
    ny, npi = S("neck_yaw"), S("neck_pitch")
    base_reach = [((0, -22.0, 221), 12.0), ((0, 19.0, 221), 12.0)]
    base, hw1 = horn("neck_yaw", 13.0, base_reach)
    # the neck pitch's stall torque bends the base round the neck yaw horn: behind the horn
    # (the face's shell comes down in front of it as the head nods), a skirt under it and 6 mm
    # where the head's servo leaves room
    behind = half("x", -1, 2.0)
    thick, _ = horn("neck_yaw", 13.0, base_reach, 6.0, patch=False)
    thick = (thick - swept(sv.keepout(npi, 1.0), "neck_pitch", child=True)) ^ behind
    reach = [((0, 0, 228.0), 8.0), ((-10, 0, 221.0), 1.0), ((10, 0, 221.0), 1.0)]
    hn, hw2 = horn("neck_pitch", 12.0, reach)
    idl, hw3 = idler("neck_pitch", 12.0, reach)
    m = union(base, thick, skirt("neck_yaw", 13.0, base_reach) ^ behind, hn, idl)
    m = finish(m, [sv.keepout(ny), sv.keepout(npi)])
    return Part(
        "neck_yoke",
        "neck_link",
        "petg",
        to_link(m, "neck_link"),
        "head_frame",
        hardware=add_hw(hw1, hw2, hw3),
        step=14,
    )


def head_frame() -> Part:
    begin()
    npi = S("neck_pitch")
    c, hw = cradle("neck_pitch", idler=True)
    # a spine up the cradle's back to the head shell's top, and the camera's mount at the front
    spine = plate(rounded(rect2(-15.0, 15.0, 236.0, 282.0), 3.0), "x", -39.2, 2.5)
    roof = plate(windowed(rounded(rect2(-39.0, 30.0, -30.0, 30.0), 4.0), 7.0), "z", 279.5, 2.5)
    cam = plate(rounded(rect2(-15.0, 15.0, 262.0, 282.0), 3.0), "x", 30.0, 2.0)
    cam = (
        cam
        - cyl(8.0, 6, "x", (31, 0, 270))
        - cyl(2.2, 6, "x", (31, -10.5, 270))
        - cyl(2.2, 6, "x", (31, 10.5, 270))
    )
    m = union(c, spine, roof, cam)
    m = finish(m, [sv.keepout(npi)], keep=[c])
    return Part(
        "head_frame",
        "head_link",
        "petg",
        to_link(m, "head_link"),
        "head_frame",
        hardware=add_hw(hw, {"M2 x 6 screw (camera)": 2}),
        step=15,
    )


def parts() -> list[Part]:
    out = []
    for f in (
        pelvis,
        hip_roll_bracket,
        hip_pitch_bracket,
        thigh,
        shin,
        ankle_bracket,
        foot,
        foot_sole,
        torso,
        shoulder_bracket,
        shoulder_yoke,
        upper_arm,
        forearm,
        palm,
        jaw,
        neck_yoke,
        head_frame,
    ):
        p = f()
        if f is not foot_sole:
            p.extra.update(ifaces(p.link))  # the patches this part's builder recorded
        if f is foot:  # the ground under the sole, for the landing case
            p.extra["ifaces"].append(
                dict(
                    joint="ground",
                    kind="face",
                    role="ground",
                    rigid=False,
                    centre=[12.0, 0.0, SOLE_Z + 2.0 - ORIGIN_FOOT_Z],
                    axis=[0.0, 0.0, 1.0],
                    r0=0.0,
                    r1=60.0,
                    span=[-1.0, 1.0],
                )
            )
        out.append(p)
    return out


ORIGIN_FOOT_Z = -262.0  # the foot frame's height at the zero pose (world)
