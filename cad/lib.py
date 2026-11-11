"""Small code-CAD toolkit on manifold3d: primitives, rounded plates, placement, the Part record.

Adapted from Marvin Bot's cad/lib.py. Units are millimetres in the part generators; mass
properties are SI. Every part is a manifold solid, so volume, centre of mass and inertia come
straight from the mesh.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import manifold3d as mf
import numpy as np

SEGMENTS = 48
BIG = 2000.0


def box(size, center=(0, 0, 0)) -> mf.Manifold:
    return mf.Manifold.cube(list(size), center=True).translate(list(center))


def span(x, y, z) -> mf.Manifold:
    """A box between (lo, hi) bounds on each axis."""
    lo = [min(a) for a in (x, y, z)]
    hi = [max(a) for a in (x, y, z)]
    return box(
        [h - lo_ for lo_, h in zip(lo, hi, strict=True)],
        [(a + b) / 2 for a, b in zip(lo, hi, strict=True)],
    )


def cyl(d: float, h: float, axis: str = "z", center=(0, 0, 0), n: int = SEGMENTS) -> mf.Manifold:
    """Cylinder of diameter d and length h along x, y or z, centred at `center`."""
    m = mf.Manifold.cylinder(h, d / 2, d / 2, n, center=True)
    if axis == "x":
        m = m.rotate([0, 90, 0])
    elif axis == "y":
        m = m.rotate([90, 0, 0])
    return m.translate(list(center))


def hull(*parts: mf.Manifold) -> mf.Manifold:
    return mf.Manifold.batch_hull(list(parts))


def union(*parts: mf.Manifold) -> mf.Manifold:
    parts = [p for p in parts if p is not None]
    return mf.Manifold.batch_boolean(parts, mf.OpType.Add)


def empty() -> mf.Manifold:
    return mf.Manifold()


def half(axis: str, sign: int, at: float = 0.0) -> mf.Manifold:
    """The half-space beyond `at` along `axis` (sign +1: coordinate > at)."""
    i = "xyz".index(axis)
    c = [0.0, 0.0, 0.0]
    c[i] = at + sign * BIG / 2
    return box((BIG, BIG, BIG), c)


# ------------------------------------------------------------------ 2-D outlines and plates


def rounded(cs: mf.CrossSection, r: float) -> mf.CrossSection:
    """Round every corner of an outline by r (convex and concave)."""
    if r <= 0:
        return cs
    j = mf.JoinType.Round
    return (
        cs.offset(-r, j, 2.0, SEGMENTS).offset(2 * r, j, 2.0, SEGMENTS).offset(-r, j, 2.0, SEGMENTS)
    )


def poly(pts) -> mf.CrossSection:
    return mf.CrossSection([[(float(u), float(v)) for u, v in pts]])


def rect2(u0, u1, v0, v1) -> mf.CrossSection:
    return poly([(u0, v0), (u1, v0), (u1, v1), (u0, v1)])


def circle2(d, at=(0.0, 0.0)) -> mf.CrossSection:
    return mf.CrossSection.circle(d / 2, SEGMENTS).translate(list(at))


def hull2(*cs: mf.CrossSection) -> mf.CrossSection:
    return mf.CrossSection.batch_hull(list(cs))


def plate(cs: mf.CrossSection, axis: str, at: float, t: float) -> mf.Manifold:
    """Extrude an outline in the plane normal to `axis` (its (u, v) = the other two axes in
    x, y, z order) from `at` to `at + t`."""
    if axis == "x":  # (u, v, w) = (y, z, x): a cyclic swap, no mirroring
        m = cs.extrude(t).translate([0, 0, at])
        return m.transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
    if axis == "y":  # (u, v) = (x, z): turned about x, which sends w to -y
        return cs.extrude(t).translate([0, 0, -(at + t)]).rotate([90, 0, 0])
    return cs.extrude(t).translate([0, 0, at])


def rbox(size, center=(0, 0, 0), r: float = 3.0, axis: str = "z") -> mf.Manifold:
    """A box with its edges along `axis` rounded by r."""
    i = "xyz".index(axis)
    u, v = [k for k in range(3) if k != i]
    cs = rounded(
        rect2(
            center[u] - size[u] / 2,
            center[u] + size[u] / 2,
            center[v] - size[v] / 2,
            center[v] + size[v] / 2,
        ),
        min(r, size[u] / 2 - 0.05, size[v] / 2 - 0.05),
    )
    return plate(cs, axis, center[i] - size[i] / 2, size[i])


def windowed(cs: mf.CrossSection, border: float, r: float = 3.0, min_w: float = 6.0):
    """An outline with a lightening window inside it, leaving `border` all round."""
    inner = cs.offset(-border, mf.JoinType.Round, 2.0, SEGMENTS)
    if inner.is_empty() or inner.area() < min_w * min_w:
        return cs
    return cs - rounded(inner, r)


def transform(m: mf.Manifold, M: np.ndarray) -> mf.Manifold:
    return m.transform(np.asarray(M)[:3, :4].tolist())


# ------------------------------------------------------------------ parts


@dataclass
class Part:
    """One made part, in its link's frame (mm). `hardware` counts the bought items it takes
    (screws, inserts, bearings, cable ties); `notes` say how it is made or fitted."""

    name: str
    link: str
    material: str  # pa_cf, petg, tpu
    solid: mf.Manifold
    budget: str  # mass_budget.yaml item
    process: str = "print"  # "print" or "cover" (an add-on shell)
    hardware: dict = field(default_factory=dict)
    notes: str = ""
    step: int = 0  # assembly step (docs/assembly.md)
    extra: dict = field(default_factory=dict)  # e.g. "ifaces": contact patches (cad/fea.py)

    def mirrored(self) -> Part:
        assert self.name.startswith("left_") and self.link.startswith("left_")
        return Part(
            "right_" + self.name[5:],
            "right_" + self.link[5:],
            self.material,
            self.solid.mirror([0, 1, 0]),
            self.budget,
            self.process,
            dict(self.hardware),
            self.notes,
            self.step,
            {
                k: (
                    [
                        dict(
                            i,
                            centre=[i["centre"][0], -i["centre"][1], i["centre"][2]],
                            axis=[i["axis"][0], -i["axis"][1], i["axis"][2]],
                        )
                        for i in v
                    ]
                    if k == "ifaces"
                    else v
                )
                for k, v in self.extra.items()
            },
        )

    def mesh(self) -> tuple[np.ndarray, np.ndarray]:
        g = self.solid.to_mesh()
        return np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts)
