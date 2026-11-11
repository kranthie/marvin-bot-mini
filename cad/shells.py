"""Clip-on shells in Marvin Bot's design language, and how they attach.

The robot works without them: they give it a finished body and some protection, and carry no
load. One language throughout, Marvin Bot's: a diamond-faceted torso (alternate octagonal
sections turned half a step, the shoulders bevelled, the side walls kept inside the shoulder
servos so those show) and faceted forearms; octagonal upper-arm sleeves with chamfered ends and
cooling slots; smooth box-section thigh and shin shells; a dark hip girdle; toe caps; a wide
rounded-box head with a dark visor; white PETG, 1.6 mm skins 2 mm off the frame.

Each shell is a loft of designed sections, sized from what it covers (the frame parts, servos
and equipment on its link) and kept clear of the neighbouring links' parts through every
joint's range (python -m cad.check). Shells stop short of the joints: the gaps are where the
harness's service loops run.

Fastening. Every shell has two kinds of `Mount` on its link's frame:
- "screw": a boss grows out of the frame part to the shell's inner face, with an M2 heat-set
  insert at its end; an M2 x 6 screw goes through the shell into it;
- "snap": a slot through a frame plate, and a hook on the shell's inside (a 1.4 mm beam with a
  0.9 mm barb) that springs through it and catches behind the plate.
So each shell hooks on at one end and is screwed at the other, and comes off with one tool.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import manifold3d as mf
import numpy as np

from . import frame
from . import servo as sv
from .layout import SERVOS, to_link
from .lib import BIG, Part, box, cyl, half, hull, span, union

SKIN, GAP = 1.4, 2.0  # mm: the shells carry no load (1.4 mm is three 0.45 mm lines)


# ------------------------------------------------------------------ sections and lofts


def octagon(w, d, c):
    c = min(c, w / 2 - 0.5, d / 2 - 0.5)
    x, y = d / 2, w / 2
    return [
        (x, y - c),
        (x - c, y),
        (-x + c, y),
        (-x, y - c),
        (-x, -y + c),
        (-x + c, -y),
        (x - c, -y),
        (x, -y + c),
    ]


def superellipse(w, d, n=4.0, k=48):
    pts = []
    for i in range(k):
        t = 2 * math.pi * i / k
        ct, st = math.cos(t), math.sin(t)
        pts.append(
            (
                d / 2 * math.copysign(abs(ct) ** (2 / n), ct),
                w / 2 * math.copysign(abs(st) ** (2 / n), st),
            )
        )
    return pts


def ngon(n, phase, squash):
    """Marvin Bot's torso section: a unit n-gon pushed out towards the unit square."""
    a = np.radians(phase + 360.0 * np.arange(n) / n)
    x, y = np.cos(a), np.sin(a)
    pts = np.column_stack([np.sign(x) * np.abs(x) ** squash, np.sign(y) * np.abs(y) ** squash])
    return pts[np.argsort(np.arctan2(pts[:, 1], pts[:, 0]), kind="stable")]


def loft(sections, shape) -> mf.Manifold:
    """Hull between neighbouring sections (z, cx, cy, w, d): w across (y), d front to back (x)."""
    segs = []
    for a, b in zip(sections, sections[1:], strict=False):
        pts = []
        for z, cx, cy, w, d in (a, b):
            pts += [[p[0] + cx, p[1] + cy, z] for p in shape(w, d)]
        segs.append(mf.Manifold.hull_points(np.array(pts)))
    return union(*segs)


def diamond(sections, twist=22.5, squash=0.62) -> mf.Manifold:
    """Marvin Bot's faceting: alternate octagonal sections turned half a step, hulled."""
    pts = []
    for i, (z, cx, cy, w, d) in enumerate(sections):
        poly = ngon(8, twist * (i % 2), squash) * [d / 2, w / 2] + [cx, cy]
        pts += [[p[0], p[1], z] for p in poly]
    return mf.Manifold.hull_points(np.array(pts))


def grown(sections, by: float, ends: float = 0.0):
    """Sections grown by `by` across and through, and the end ones moved out by `ends`."""
    out = [(z, cx, cy, w + 2 * by, d + 2 * by) for z, cx, cy, w, d in sections]
    if ends:
        s = 1 if out[-1][0] > out[0][0] else -1
        out[0] = (out[0][0] - s * ends,) + out[0][1:]
        out[-1] = (out[-1][0] + s * ends,) + out[-1][1:]
    return out


def chamfer_ends(sections, c):
    """An inset section past each end: chamfered ends."""
    (z0, x0, y0, w0, d0), (z1, x1, y1, w1, d1) = sections[0], sections[-1]
    s = 1 if z1 > z0 else -1
    return (
        [(z0 - s * c, x0, y0, w0 - 2 * c, d0 - 2 * c)]
        + list(sections)
        + [(z1 + s * c, x1, y1, w1 - 2 * c, d1 - 2 * c)]
    )


def skin(outer_fn, sections, by=SKIN, open_ends=False):
    """The shell between a loft and the same loft shrunk by `by`; and the inside (the void).
    With open_ends, a sleeve: the inside runs out past both ends."""
    outer = outer_fn(sections)
    inner = outer_fn(grown(sections, -by, ends=10.0 if open_ends else -by))
    return outer - inner, inner


# ------------------------------------------------------------------ mounts


@dataclass
class Mount:
    part: str  # the frame part it is on
    at: tuple  # a point on that part's surface (world, zero pose)
    n: tuple  # the outward direction, towards the shell
    kind: str  # "screw" or "snap"
    along: str = "z"  # a snap slot's long side
    t: float = 3.0  # the frame plate's thickness under a snap


def _cyl_along(d, p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    v = p1 - p0
    L = float(np.linalg.norm(v))
    m = mf.Manifold.cylinder(L, d / 2, d / 2, 32)
    z = np.array([0, 0, 1.0])
    a = v / L
    ax = np.cross(z, a)
    if np.linalg.norm(ax) < 1e-9:
        R = np.eye(3) if a[2] > 0 else np.diag([1, -1, -1])
    else:
        ax /= np.linalg.norm(ax)
        ang = math.acos(max(-1, min(1, float(z @ a))))
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    M = np.eye(4)[:3]
    M[:3, :3] = R
    M[:3, 3] = p0
    return m.transform(M.tolist())


def _bar(n, along, p, lo, hi, wa, wt, off=0.0):
    """A bar along n from p+n*lo to p+n*hi, wa wide along `along`, wt thick across both,
    shifted `off` across."""
    n = np.asarray(n, float)
    i_n = int(np.argmax(np.abs(n)))
    i_a = "xyz".index(along)
    i_t = 3 - i_n - i_a
    lo_, hi_ = sorted((lo * n[i_n], hi * n[i_n]))
    size = [0.0, 0.0, 0.0]
    c = np.array(p, float)
    size[i_n] = hi_ - lo_
    c[i_n] = p[i_n] + (lo_ + hi_) / 2
    size[i_a] = wa
    size[i_t] = wt
    c[i_t] += off
    return box(size, c)


def attach(mounts: list[Mount], shell: mf.Manifold, inner: mf.Manifold, frames: dict) -> tuple:
    """Grow each mount's boss or cut its slot in its frame part, and cut its hole in or add its
    hook to the shell. frames: {part name: world solid}, changed in place. Returns the shell
    and the hardware it takes."""
    hw: dict = {}
    for mt in mounts:
        n = np.asarray(mt.n, float)
        p = np.asarray(mt.at, float)
        # how far the shell's inner face is along n
        probe = _cyl_along(1.0, p - n, p + n * 80) ^ inner
        if probe.is_empty():
            raise ValueError(f"mount on {mt.part} at {mt.at} does not reach its shell")
        g = probe.to_mesh()
        v = np.asarray(g.vert_properties)[:, :3]
        L = float(np.max((v - p) @ n))
        # a mount must sit on solid frame, not over one of its lightening windows
        seat = _cyl_along(3.0, p - n * 0.8, p + n * 0.2) ^ frames[mt.part]
        if seat.volume() < 1.0:
            raise ValueError(f"mount on {mt.part} at {mt.at} is not on solid frame")
        if mt.kind == "screw":
            boss = _cyl_along(6.5, p - n * 0.5, p + n * (L - 0.2))
            boss = boss - _cyl_along(3.2, p + n * (L - 4.2), p + n * (L + 1))
            frames[mt.part] = union(frames[mt.part], boss)
            shell = shell - _cyl_along(2.4, p + n * (L - 1), p + n * (L + SKIN + 2))
            for k, c in (("M2 x 3 heat-set insert", 1), ("M2 x 6 screw (shell)", 1)):
                hw[k] = hw.get(k, 0) + c
        else:
            frames[mt.part] = frames[mt.part] - _bar(n, mt.along, p, -mt.t - 2, 1.0, 6.4, 2.6)
            beam = _bar(n, mt.along, p, -mt.t - 1.6, L + 1.0, 5.6, 1.4) ^ inner
            barb = _bar(n, mt.along, p, -mt.t - 1.6, -mt.t - 0.2, 5.6, 0.9, off=1.1)
            shell = union(shell, beam, barb)
    return shell, hw


# ------------------------------------------------------------------ the shells


def slab_bounds(solids, z0, z1, ylim=(-BIG, BIG), xlim=(-BIG, BIG)):
    """(x0, x1, y0, y1) of the solids between heights z0 and z1, within the limits."""
    cut = span(xlim, ylim, (z0, z1))
    bb = None
    for s in solids:
        b = (s ^ cut).bounding_box()
        if b[3] < b[0]:
            continue
        bb = (
            b
            if bb is None
            else (
                min(bb[0], b[0]),
                min(bb[1], b[1]),
                bb[2],
                max(bb[3], b[3]),
                max(bb[4], b[4]),
                bb[5],
            )
        )
    return bb


def fit(
    solids,
    levels,
    shape,
    pad=GAP,
    ylim=(-BIG, BIG),
    xlim=(-BIG, BIG),
    grow=(0, 0, 0, 0),
    deep=False,
):
    """Sections (z, cx, cy, w, d) holding the solids: at each (z, z0, z1) level, the bounds of
    what lies between z0 and z1 (within the limits), plus `pad`, plus `grow` (x back, x front,
    y in, y out), and the section's inside grown until `shape(w, d, i)` (the i-th level's
    outline) holds the bounds' corners: a chamfered or rounded section clears them too. With
    `deep`, only its depth grows (where its width is held between a neighbour's plates)."""
    out = []
    for i, (z, z0, z1) in enumerate(levels):
        b = slab_bounds(solids, z0, z1, ylim, xlim)
        x0, x1 = b[0] - pad - grow[0], b[3] + pad + grow[1]
        y0, y1 = b[1] - pad - grow[2], b[4] + pad + grow[3]
        d0, w0 = x1 - x0, y1 - y0
        corners = [(sx * d0 / 2, sy * w0 / 2) for sx in (-1, 1) for sy in (-1, 1)]
        kw = 1.0
        while not all(
            _inside(np.array(shape(w0 * (1 if deep else kw), d0 * kw, i)), c) for c in corners
        ):
            kw *= 1.01
            if kw > 3.0:
                raise ValueError(f"section at z {z} cannot hold its contents")
        w = w0 * (1 if deep else kw)
        out.append((z, (x0 + x1) / 2, (y0 + y1) / 2, w + 2 * SKIN, d0 * kw + 2 * SKIN))
    return out


def sq4(w, d, i=0):
    return superellipse(w, d, 4.0)


def oct4(w, d, i=0):
    return octagon(w, d, 4.0)


def oct7(w, d, i=0):
    return octagon(w, d, 7.0)


def dia(w, d, i=0):
    return ngon(8, 22.5 * (i % 2), 0.62) * [d / 2, w / 2]


def on_link(frames, part, joints):
    return [frames[part]] + [servo_case(j) for j in joints]


def servo_case(j):
    return sv.place(SERVOS[j], sv.case())


def torso_contents(frames):
    """What the torso shell holds: the frame (inside the side walls), the computer, battery,
    power board and neck servo."""
    tor = frames["torso"] ^ span((-BIG, BIG), (-55.0, 55.0), (-BIG, BIG))
    return [
        tor,
        span((-6.5, 28.5), (-50.0, 50.0), (65.5, 144.5)),  # computer
        span((-29.0, -9.0), (-30.0, 30.0), (70.0, 140.0)),  # battery
        span((-32.5, -15.5), (-26.0, 26.0), (137.0, 182.0)),  # power board
        servo_case("neck_yaw"),
    ]


def faceted_torso(contents, z0, z1, neck_z, gap, levels=9):
    """Marvin Bot's torso: n-gon sections at `levels` heights, alternate ones turned, each
    holding what lies within 14 % of the height either side of it (plus gap), symmetric about
    y = 0; and a small section the shoulders rise to round the neck."""
    span_z = z1 - z0
    pts = []
    for i, f in enumerate(np.linspace(0, 1, levels)):
        b = slab_bounds(contents, z0 + max(0, f - 0.14) * span_z, z0 + min(1, f + 0.14) * span_z)
        x0, x1 = b[0] - gap, b[3] + gap
        w = max(abs(b[1]), abs(b[4])) + gap
        shape = ngon(8, 22.5 * (i % 2), 0.62)
        c, a = np.array([(x0 + x1) / 2, 0.0]), (x1 - x0) / 2
        k = 1.0
        while not all(
            _inside(c + shape * [a * k, w * k], q) for q in [(x0, -w), (x0, w), (x1, -w), (x1, w)]
        ):
            k *= 1.01
        poly = c + shape * [a * k, w * k]
        pts += [[p[0], p[1], z0 + f * span_z] for p in poly]
    b = slab_bounds(contents, z1 - 6, z1)
    c = np.array([(b[0] + b[3]) / 2, 0.0])
    poly = c + ngon(8, 22.5, 0.62) * [(b[3] - b[0]) / 2 * 0.55, 30.0 + gap]
    pts += [[p[0], p[1], neck_z] for p in poly]
    return mf.Manifold.hull_points(np.array(pts))


def _inside(poly, p):
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -1e-9:
            return False
    return True


def side_keep(half_w, z_top, z_from, top_c=12.0, edge_c=9.0, x0=-90.0, x1=90.0):
    """The torso shell kept within |y| <= half_w, its side walls' top edge bevelled in."""

    def layer(z, w, c):
        q = [(x1, w - c, z), (x1 - c, w, z), (x0 + c, w, z), (x0, w - c, z)]
        return q + [(x, -y, z) for x, y, _ in q]

    pts = layer(-BIG, half_w, 0) + layer(z_from, half_w, 0)
    pts += layer(z_top - top_c, half_w, edge_c) + layer(z_top, half_w - top_c, edge_c)
    return mf.Manifold.hull_points(np.array(pts))


def bevel(body, at, slope, z0):
    """The front side corners bevelled, deeper towards the shoulders (Marvin Bot's)."""
    for sy in (1, -1):
        nrm = np.array([1.0, sy, slope])
        k = np.linalg.norm(nrm)
        body = body.trim_by_plane((-nrm / k).tolist(), -(at + slope * z0) / k)
    return body


def vents(m, centre, axis, n=4, pitch=5.0, w=1.8, length=16.0, depth=8.0):
    """Cooling slots through a shell over a servo, `n` across `axis` (the direction they are
    spaced), each `length` long along z."""
    cuts = []
    for k in range(n):
        c = np.array(centre, float)
        c["xyz".index(axis)] += (k - (n - 1) / 2) * pitch
        size = [depth, depth, length]
        size["xyz".index(axis)] = w
        cuts.append(box(size, c))
    return m - union(*cuts)


SHELL_TOP_Z = 214.0  # the torso shell's shoulders (world z)


def torso_shells(frames):
    contents = torso_contents(frames)
    z0, z1 = 40.0, SHELL_TOP_Z
    outer = faceted_torso(contents, z0, z1, 219.0, GAP + SKIN)
    inner = faceted_torso(contents, z0, z1, 219.0, GAP)
    keep = side_keep(57.0, 228.0, 150.0)
    sh = bevel(outer ^ keep, 86.0, 0.35, 150.0) - bevel(
        inner ^ side_keep(57.0 - SKIN, 228.0, 150.0), 86.0 - SKIN, 0.35, 150.0
    )
    sh = sh ^ half("z", 1, z0 + 1.0)
    sh = sh - cyl(64.0, 40.0, "z", (-4.0, 0, 225.0)).scale([0.9, 1.0, 1.0])  # the neck opening
    for sy in (1, -1):  # openings over the shoulder servos, which show through the side walls
        sh = sh - rbox_y((36.0, 30.0, 56.0), (0.0, sy * 55.0, 179.0), 6.0)
    # the chest's status light: a slot for a light pipe
    sh = sh - span((20.0, 40.0), (-12.0, 12.0), (172.0, 175.0))
    inner = bevel(inner ^ side_keep(57.0 - SKIN, 228.0, 150.0), 86.0 - SKIN, 0.35, 150.0)
    front = sh ^ half("x", 1, -8.0)
    back = sh ^ half("x", -1, -8.0)
    mf_ = [
        Mount("torso", (28.0, 20.0, 210.5), (1, 0, 0), "screw"),
        Mount("torso", (28.0, -20.0, 210.5), (1, 0, 0), "screw"),
        Mount("torso", (24.5, 54.0, 60.0), (0, 1, 0), "snap", "z"),
        Mount("torso", (24.5, -54.0, 60.0), (0, -1, 0), "snap", "z"),
    ]
    mb = [
        Mount("torso", (-34.0, 30.0, 207.0), (-1, 0, 0), "screw"),
        Mount("torso", (-34.0, -30.0, 207.0), (-1, 0, 0), "screw"),
        Mount("torso", (-30.5, 54.0, 60.0), (0, 1, 0), "snap", "z"),
        Mount("torso", (-30.5, -54.0, 60.0), (0, -1, 0), "snap", "z"),
    ]
    front, hw_f = attach(mf_, front, inner, frames)
    back, hw_b = attach(mb, back, inner, frames)
    return [
        Part(
            "torso_cover_front",
            "torso_link",
            "petg",
            to_link(front, "torso_link"),
            "shells",
            "cover",
            hw_f,
            step=17,
        ),
        Part(
            "torso_cover_back",
            "torso_link",
            "petg",
            to_link(back, "torso_link"),
            "shells",
            "cover",
            hw_b,
            step=17,
        ),
    ]


def girdle(frames):
    """The dark hip girdle round the waist servo and the pelvis's front, between the hips."""
    c = [frames["pelvis"], servo_case("waist_yaw"), span((16.0, 28.0), (-14.0, 14.0), (-3.0, 27.0))]
    secs = fit(
        c, [(5.0, 4.0, 10.0), (14.0, 10.0, 20.0), (29.0, 20.0, 29.5)], dia, ylim=(-14.0, 14.0)
    )
    sh, inner = skin(lambda s: diamond(s), secs, open_ends=True)
    sh = sh - span((-80, 80), (-30, 30), (29.0, 60))  # open on top, under the pelvis's deck
    sh = sh ^ half("x", 1, -20.6)  # in front of the pelvis's front wall
    mounts = [
        Mount("pelvis", (16.3, 0.0, 27.5), (1, 0, 0), "screw"),
        Mount("pelvis", (16.3, 11.0, 10.0), (1, 0, 0), "snap", "z", 2.6),
    ]
    sh, hw = attach(mounts, sh, inner, frames)
    return [
        Part(
            "pelvis_cover",
            "pelvis",
            "petg_black",
            to_link(sh, "pelvis"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    ]


def leg_shells(frames):
    out = []
    # thigh: smooth box sections, straight taper, from under the hip yaw horn to above the knee
    c = on_link(frames, "left_thigh", ["left_hip_yaw", "left_knee"])
    secs = fit(c, [(-86.0, -88.0, -74.0), (-52.0, -74.0, -52.0)], sq4, grow=(0, 2, 0, 0))
    sh, inner = skin(
        lambda s: loft(chamfer_ends(s, 3.0), lambda w, d: superellipse(w, d, 4.0)),
        secs,
        open_ends=True,
    )
    sh = sh ^ half("x", 1, -12.0)  # open at the back: the spine and the harness run there
    mounts = [
        Mount("left_thigh", (36.8, 45.0, -66.0), (1, 0, 0), "screw"),
        Mount("left_thigh", (15.0, 45.0, -86.0), (1, 0, 0), "snap", "y", 2.4),
    ]
    sh, hw = attach(mounts, sh, inner, frames)
    out.append(
        Part(
            "left_thigh_cover",
            "left_thigh_link",
            "petg",
            to_link(sh, "left_thigh_link"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    )
    # shin: front and sides, from below the knee to above the ankle
    c = on_link(frames, "left_shin", ["left_ankle_pitch"])
    secs = fit(c, [(-198.0, -200.0, -180.0), (-158.0, -180.0, -158.0)], sq4, ylim=(28.0, 64.0))
    sh, inner = skin(
        lambda s: loft(chamfer_ends(s, 3.0), lambda w, d: superellipse(w, d, 4.0)),
        secs,
        open_ends=True,
    )
    sh = sh ^ half("x", 1, 13.6)  # a guard over the shin's front, ahead of its side plates
    mounts = [
        Mount("left_shin", (13.4, 29.5, -180.0), (1, 0, 0), "snap", "z", 2.4),
        Mount("left_shin", (13.4, 46.0, -153.5), (1, 0, 0), "screw"),
    ]
    sh, hw = attach(mounts, sh, inner, frames)
    out.append(
        Part(
            "left_shin_cover",
            "left_shin_link",
            "petg",
            to_link(sh, "left_shin_link"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    )
    # toe cap: over the front of the foot, ahead of the roll servo
    secs = [(-281.5, 58.5, 45.0, 56.0, 14.0), (-246.0, 58.0, 45.0, 48.0, 12.0)]
    sh, inner = skin(
        lambda s: loft(chamfer_ends(s, 2.5), lambda w, d: superellipse(w, d, 4.0)), secs
    )
    sh = sh ^ half("x", 1, 52.0) ^ half("z", 1, -281.8)
    sh = sh - cyl(24.0, 12.0, "x", (55.0, 45.0, -262.0))  # round the roll servo's horn
    mounts = [Mount("left_foot", (59.8, 45.0, -278.0), (1, 0, 0), "screw")]
    sh, hw = attach(mounts, sh, inner, frames)
    out.append(
        Part(
            "left_toe_cover",
            "left_foot_link",
            "petg",
            to_link(sh, "left_foot_link"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    )
    return out


def arm_shells(frames):
    out = []
    # shoulder cap over the roll servo's top, inside the roll yoke's plates
    c = on_link(frames, "left_shoulder_bracket", ["left_shoulder_roll"])
    secs = fit(
        c,
        [(217.0, 216.0, 222.0), (227.0, 222.0, 227.0)],
        oct7,
        xlim=(-15.0, 13.6),
        ylim=(81.0, 111.0),
    )
    sh, inner = skin(lambda s: loft(chamfer_ends(s, 4.0), lambda w, d: octagon(w, d, 7.0)), secs)
    sh = sh ^ half("z", 1, 216.0) ^ span((-18.2, 20.4), (-BIG, 113.5), (-BIG, BIG))
    mounts = [Mount("left_shoulder_bracket", (-2.0, 95.0, 226.8), (0, 0, 1), "screw")]
    sh, hw = attach(mounts, sh, inner, frames)
    out.append(
        Part(
            "left_shoulder_cover",
            "left_shoulder_pitch_link",
            "petg",
            to_link(sh, "left_shoulder_pitch_link"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    )
    # upper arm: octagonal, chamfered ends, slight taper, over the yaw servo and down to the
    # elbow servo's top; cooling slots over the yaw servo
    c = on_link(frames, "left_upper_arm", ["left_shoulder_yaw", "left_elbow"])
    secs = fit(
        c,
        [(124.0, 122.0, 135.0), (150.0, 135.0, 150.0)],
        oct4,
        pad=1.2,
        ylim=(80.0, 110.0),
        deep=True,
    )
    sh, inner = skin(lambda s: loft(chamfer_ends(s, 2.5), oct4), secs, open_ends=True)
    sh = sh ^ span((-BIG, BIG), (77.0, 115.0), (-BIG, BIG))  # between the forearm's plates
    sh = vents(sh, (-12.0, 113.0, 136.0), "x")
    mounts = [
        Mount("left_upper_arm", (13.8, 95.0, 134.0), (1, 0, 0), "screw"),
        Mount("left_upper_arm", (-36.8, 95.0, 133.0), (-1, 0, 0), "snap", "z", 2.4),
    ]
    sh, hw = attach(mounts, sh, inner, frames)
    out.append(
        Part(
            "left_upper_arm_cover",
            "left_upper_arm_link",
            "petg",
            to_link(sh, "left_upper_arm_link"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    )
    # forearm: faceted, from below the elbow's swing to the wrist servo's output
    c = on_link(frames, "left_forearm", ["left_wrist_yaw"])
    secs = fit(c, [(13.0, 14.0, 24.0), (24.0, 22.0, 28.0), (34.0, 28.0, 34.0)], dia, pad=1.5)
    sh, inner = skin(lambda s: diamond(s), secs, open_ends=True)
    sh = vents(sh, (-12.0, 113.0, 22.0), "x", n=3, length=10.0)
    mounts = [
        Mount("left_forearm", (13.8, 95.0, 22.0), (1, 0, 0), "screw"),
        Mount("left_forearm", (-36.8, 95.0, 24.0), (-1, 0, 0), "snap", "z", 2.4),
    ]
    sh, hw = attach(mounts, sh, inner, frames)
    out.append(
        Part(
            "left_forearm_cover",
            "left_forearm_link",
            "petg",
            to_link(sh, "left_forearm_link"),
            "shells",
            "cover",
            hw,
            step=16,
        )
    )
    return out


def head_shells(frames):
    """The head: a wide rounded box (Marvin Bot's W1 proportions) in two pieces, the face and
    the back, and a dark visor across the face with the camera's window."""
    c = (-3.0, 0.0, 261.0)
    size = (86.0, 88.0, 58.0)
    outer = rbox3(size, c, 10.0)
    inner = rbox3([s - 2 * SKIN for s in size], c, 10.0 - SKIN)
    sh = outer - inner
    sh = sh - span((-36.0, 30.0), (-29.0, 29.0), (220.0, 240.0))  # the neck's opening
    sh = sh - span((36.0, 50.0), (-36.0, 36.0), (258.0, 276.0))  # the visor's window
    face = sh ^ half("x", 1, 22.0)
    back = sh ^ half("x", -1, 22.0)
    visor = span((36.6, 39.0), (-37.5, 37.5), (257.0, 277.0)) ^ outer.translate([0.2, 0, 0])
    visor = visor - cyl(8.0, 10, "x", (38, 0, 270.0))
    mf_ = [
        Mount("head_frame", (32.0, 12.0, 280.0), (1, 0, 0), "screw"),
        Mount("head_frame", (32.0, -12.0, 280.0), (1, 0, 0), "screw"),
    ]
    mb = [
        Mount("head_frame", (-39.2, 0.0, 262.0), (-1, 0, 0), "screw"),
        Mount("head_frame", (0.0, 26.5, 282.0), (0, 0, 1), "snap", "x", 2.5),
    ]
    face, hw_f = attach(mf_, face, inner, frames)
    back, hw_b = attach(mb, back, inner, frames)
    return [
        Part(
            "head_cover_face",
            "head_link",
            "petg",
            to_link(face, "head_link"),
            "shells",
            "cover",
            hw_f,
            step=18,
        ),
        Part(
            "head_cover_back",
            "head_link",
            "petg",
            to_link(back, "head_link"),
            "shells",
            "cover",
            hw_b,
            step=18,
        ),
        Part(
            "head_visor",
            "head_link",
            "petg_black",
            to_link(visor, "head_link"),
            "shells",
            "cover",
            {},
            notes="glued into the face",
            step=18,
        ),
    ]


def rbox_y(size, centre, r):
    """A box with its edges along y rounded r."""
    from .lib import rbox

    return rbox(size, centre, r, "y")


def rbox3(size, centre, r):
    """A box with every edge rounded r (the hull of eight spheres)."""
    sx, sy, sz = (s / 2 - r for s in size)
    sph = [
        mf.Manifold.sphere(r, 32).translate(
            (centre[0] + a * sx, centre[1] + b * sy, centre[2] + c * sz)
        )
        for a in (-1, 1)
        for b in (-1, 1)
        for c in (-1, 1)
    ]
    return mf.Manifold.batch_hull(sph)


def build(frame_parts: list[Part]) -> tuple[list[Part], list[Part]]:
    """The shells, and the frame parts with their shell mounts added."""
    from .layout import ORIGIN

    frames = {p.name: p.solid.translate(list(ORIGIN[p.link])) for p in frame_parts}
    shells = (
        torso_shells(frames)
        + girdle(frames)
        + leg_shells(frames)
        + arm_shells(frames)
        + head_shells(frames)
    )
    out = []
    for p in frame_parts:
        out.append(
            Part(
                p.name,
                p.link,
                p.material,
                to_link(frames[p.name], p.link),
                p.budget,
                p.process,
                p.hardware,
                p.notes,
                p.step,
                p.extra,
            )
        )
    return out, shells


def parts() -> list[Part]:
    return build(frame.parts())[1]


__all__ = ["build", "parts", "hull"]
