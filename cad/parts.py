"""Printed parts of the legs, pelvis and torso, first pass, in build123d.

Each part is a function returning a build123d solid in its link's frame (millimetres; the link
frames are description/skeleton.yaml's). Left side and centre only: cad/build.py mirrors the
left parts for the right side. The parts are shaped round the servo cases' envelopes
(description/actuators.yaml) with a clearance, as plates and shells of printed wall; they carry
no fasteners, bearing seats or horn bolt patterns yet, and their strength is not checked yet
(cad/fea.py, to come). Every load-carrying leg part is nylon-carbon; the rest PETG.

The servo layout follows skeleton.yaml: a servo's case is centred on its joint (or its
motor_mounts point) with the shaft along the joint axis, and the U-bracket of the next link
wraps it, one arm on the horn and one on the idler on the case's far side.
"""

from build123d import Box, Location, Part

WALL = 3.0  # mm, bracket plates
SHELL = 2.5  # mm, shells round servo cases
CLEAR = 1.0  # mm, between a case and a part round it
SERVO_L, SERVO_H, SERVO_W = 45.3, 35.8, 24.8  # case length, height along the shaft, width
U_GAP = SERVO_H + 2 * CLEAR  # inside width of a U-bracket round a servo, along its shaft


def box(size, centre) -> Part:
    return Box(*size).locate(Location(tuple(centre)))


def shell(size, centre, wall=SHELL, open_z=(True, False)) -> Part:
    """A box shell of the given wall, open at the bottom and/or top (open_z: bottom, top)."""
    lo_open, hi_open = open_z
    z_lo = centre[2] - size[2] / 2 + (-1 if lo_open else wall)
    z_hi = centre[2] + size[2] / 2 - (-1 if hi_open else wall)
    inner = box(
        [size[0] - 2 * wall, size[1] - 2 * wall, z_hi - z_lo],
        (centre[0], centre[1], (z_lo + z_hi) / 2),
    )
    return box(size, centre) - inner


def window_plate(size, centre, border=7.0) -> Part:
    """A flat plate with a lightening window, leaving a border round it. The thinnest of the
    three dimensions is the plate's thickness."""
    t = min(size)
    axis = size.index(t)
    cut = [s - 2 * border if i != axis else t + 2 for i, s in enumerate(size)]
    if min(c for i, c in enumerate(cut) if i != axis) <= 4:
        return box(size, centre)
    return box(size, centre) - box(cut, centre)


def u_plates(y_half, x, z_top, z_bot, x_centre=0.0, border=7.0) -> Part:
    """The two arms of a U-bracket: plates normal to y at +-y_half (inside faces), from z_top to
    z_bot, x wide."""
    h = z_top - z_bot
    zc = (z_top + z_bot) / 2
    yc = y_half + WALL / 2
    return window_plate([x, WALL, h], [x_centre, yc, zc], border) + window_plate(
        [x, WALL, h], [x_centre, -yc, zc], border
    )


# ------------------------------------------------------------------------------- legs


def hip_roll_bracket() -> Part:
    """On the hip roll link: wraps the hip pitch servo (shaft along y, case centred on the
    joint) and joins the hip roll servo's horn behind it (a web normal to x)."""
    g = U_GAP / 2
    arms = u_plates(g, 40, 22, -22, x_centre=-6)
    web = box([WALL, U_GAP + 2 * WALL, 44], [-27.5, 0, 0])
    return arms + web


def hip_pitch_bracket() -> Part:
    """On the hip pitch link: from the hip pitch horn and idler (plates normal to y round the
    hip) down to the hip yaw horn (a plate normal to z, 45 mm below)."""
    g = U_GAP / 2 + WALL + CLEAR  # outside the hip roll bracket's arms
    arms = u_plates(g, 30, 14, -40)
    floor = box([34, 2 * g + 2 * WALL, WALL], [0, 0, -40 - WALL / 2])
    return arms + floor


def thigh() -> Part:
    """On the thigh link: a shell round the hip yaw servo (shaft up, at the top) and down to
    the knee servo (shaft along y), whose case it cradles."""
    upper = shell([SERVO_W + 2 * (SHELL + CLEAR), 50, 44], [0, 0, -32], open_z=(False, True))
    knee = shell(
        [SERVO_L + 2 * (SHELL + CLEAR), U_GAP + 2 * SHELL, SERVO_W + 2 * (SHELL + CLEAR)],
        [0, 0, -75],
        open_z=(True, False),
    )
    return upper + knee


def shin() -> Part:
    """On the shin link: a U round the knee servo (its arms on the horn and idler), a spine down
    the back and a cradle round the ankle pitch servo at the bottom."""
    g = U_GAP / 2 + SHELL + CLEAR
    arms = u_plates(g, 32, 18, -30)
    spine = shell([30, 2 * g + 2 * WALL, 66], [-2, 0, -62], open_z=(True, True))
    ankle = shell(
        [SERVO_L + 2 * (SHELL + CLEAR), U_GAP + 2 * SHELL, SERVO_W + 2 * (SHELL + CLEAR)],
        [0, 0, -120],
        open_z=(True, False),
    )
    return arms + spine + ankle


def ankle_cross() -> Part:
    """On the ankle link: the cross piece between ankle pitch and roll, a U on the pitch servo's
    horn and idler holding the roll servo below and behind the pitch axis."""
    g = U_GAP / 2 + SHELL + CLEAR
    arms = u_plates(g, 26, 12, -12, border=6)
    cradle = box([30, 2 * g + 2 * WALL, WALL], [-8, 0, -14])
    return arms + cradle


def foot() -> Part:
    """On the foot link: the sole plate (the sole 50 mm below the ankle) with two lugs rising to
    the ankle roll horn and idler (plates normal to x round the roll servo)."""
    plate = window_plate([100, 55, 4], [12, 0, -44], border=9)
    rim = box([100, 55, 4], [12, 0, -44]) - box([84, 39, 6], [12, 0, -44])
    ribs = box([84, 4, 4], [12, 0, -44]) + box([4, 39, 4], [12, 0, -44])
    g = U_GAP / 2
    lug = box([WALL, 30, 34], [g + WALL / 2 - 15, 0, -25]) + box(
        [WALL, 30, 34], [-g - WALL / 2 - 15, 0, -25]
    )
    return plate + rim + ribs + lug


def foot_sole() -> Part:
    """A 2 mm TPU sole under the foot plate, for grip."""
    return box([100, 55, 2], [12, 0, -47])


# ------------------------------------------------------------------------------ trunk


def pelvis() -> Part:
    """The pelvis: a shell round the two hip roll servos (shafts along x, behind the hips) and
    the waist servo (shaft up, on top), open underneath for the hip roll brackets."""
    body = shell([60, 120, 44], [-24, 0, 8], open_z=(True, False))
    deck = window_plate([60, 80, WALL], [-24, 0, 30 + WALL / 2], border=10)
    return body + deck


def torso() -> Part:
    """The torso frame: a floor on the waist horn, two side frames carrying the shoulder pitch
    servos, a top plate under the neck servo, and a back plate for the battery and electronics;
    the computer stands on edge in the front bay."""
    side_y = 60 + WALL / 2
    sides = window_plate([70, WALL, 170], [0, side_y, 88], border=9) + window_plate(
        [70, WALL, 170], [0, -side_y, 88], border=9
    )
    span = 2 * (60 + WALL)  # floor, top and back overlap the side frames
    floor = window_plate([70, span, WALL], [0, 0, 5], border=10)
    top = window_plate([70, span, WALL], [0, 0, 172], border=12)
    back = window_plate([WALL, span, 165], [-35 + WALL / 2, 0, 88], border=10)
    return sides + floor + top + back


# part name -> (function, link, material, budget item); left side and centre
PARTS = {
    "left_hip_roll_bracket": (hip_roll_bracket, "left_hip_roll_link", "pa_cf", "leg_frame"),
    "left_hip_pitch_bracket": (hip_pitch_bracket, "left_hip_pitch_link", "pa_cf", "leg_frame"),
    "left_thigh": (thigh, "left_thigh_link", "pa_cf", "leg_frame"),
    "left_shin": (shin, "left_shin_link", "pa_cf", "leg_frame"),
    "left_ankle_cross": (ankle_cross, "left_ankle_link", "pa_cf", "leg_frame"),
    "left_foot": (foot, "left_foot_link", "petg", "leg_frame"),
    "left_foot_sole": (foot_sole, "left_foot_link", "tpu", "leg_frame"),
    "pelvis": (pelvis, "pelvis", "petg", "leg_frame"),
    "torso": (torso, "torso_link", "petg", "torso_frame"),
}
