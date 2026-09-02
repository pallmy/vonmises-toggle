#!/usr/bin/env python3
"""Parametric laser-cut frame generator (DXF, mm).

Rebuilds the fiberglass frame planform from parameters, following the
design rules measured off the original vonmisesfiberglass2dcutout.dxf:

  - pivot P at the origin; band anchor S at distance b, direction phi
  - ONE standoff radius w everywhere: the corner arcs around P and S,
    the edge fillets, and the offsets of every straight edge (top edge
    tangent to the S circle, bottom edge tangent to the P circle,
    right edge parallel to P-S at w) -- this is what lets the clevis
    corner ride the channel wall and the stops land flush
  - vertical spine, mounting tab with a 4-hole pattern
  - lightening cutouts: the interior web offset inward by `margin`,
    split by a diagonal rib, corners rounded

With the default parameters the output reproduces the original frame
(P/S spacing, all standoffs, tab and hole pattern). Change b, phi, w,
or anything else and the laser file, the clevis stops, and the
assembly all stay consistent.

Usage:
    python generate_frame.py                        # reproduce original
    python generate_frame.py --b 120 --phi 120 --out custom_frame.dxf
"""

import argparse
import math
from pathlib import Path

import ezdxf
import numpy as np
from shapely.geometry import LineString, Polygon, box


def arc_bulge(center, start, end, cw=True):
    """DXF bulge for the arc start->end about center (cw negative)."""
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])
    sweep = a1 - a0
    if cw and sweep > 0:
        sweep -= 2 * math.pi
    if not cw and sweep < 0:
        sweep += 2 * math.pi
    return math.tan(sweep / 4.0)


def arc_points(center, start, end, cw=True, seg=24):
    """Discretized arc (start exclusive) for the shapely outline."""
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])
    sweep = a1 - a0
    if cw and sweep > 0:
        sweep -= 2 * math.pi
    if not cw and sweep < 0:
        sweep += 2 * math.pi
    r = math.hypot(start[0] - center[0], start[1] - center[1])
    return [(center[0] + r * math.cos(a0 + sweep * i / seg),
             center[1] + r * math.sin(a0 + sweep * i / seg))
            for i in range(1, seg + 1)]


def build_outline(b, phi_deg, w, spine, tab_l, tab_y0, tab_y1):
    """Outline as (vertices-with-bulges, dense point loop). P at origin.

    Returns [(x, y, bulge), ...] in the original file's clockwise
    order, and the discretized loop for shapely."""
    phi = math.radians(phi_deg)
    u = (math.cos(phi), math.sin(phi))
    n = (u[1], -u[0])                      # offset side of the P-S edge
    S = (b * u[0], b * u[1])
    P = (0.0, 0.0)

    v1 = (spine + w, S[1] + w)             # end of top-left fillet
    v2 = (S[0], S[1] + w)                  # top edge -> S circle tangent
    v3 = (S[0] + w * n[0], S[1] + w * n[1])
    v4 = (w * n[0], w * n[1])              # right edge -> P circle
    v5 = (0.0, -w)                         # P circle -> bottom edge
    v6 = (spine + w, -w)
    v7 = (spine, 0.0)                      # bottom-left fillet end
    v8 = (spine, tab_y0)
    v9 = (spine - tab_l, tab_y0)
    v10 = (spine - tab_l, tab_y1)
    v11 = (spine, tab_y1)
    v12 = (spine, S[1])
    c_bl = (spine + w, 0.0)                # fillet centers
    c_tl = (spine + w, S[1])

    verts = [
        (*v1, 0.0),
        (*v2, arc_bulge(S, v2, v3)),
        (*v3, 0.0),
        (*v4, arc_bulge(P, v4, v5)),
        (*v5, 0.0),
        (*v6, arc_bulge(c_bl, v6, v7)),
        (*v7, 0.0), (*v8, 0.0), (*v9, 0.0), (*v10, 0.0), (*v11, 0.0),
        (*v12, arc_bulge(c_tl, v12, v1)),
    ]

    loop = [v1, v2]
    loop += arc_points(S, v2, v3)
    loop += [v4]
    loop += arc_points(P, v4, v5)
    loop += [v6]
    loop += arc_points(c_bl, v6, v7)
    loop += [v8, v9, v10, v11, v12]
    loop += arc_points(c_tl, v12, v1)
    return verts, loop, S


def build_cutouts(loop, S, spine, w, margin, rib_w, fillet, min_area):
    """Interior web minus a diagonal rib, corners rounded. The web is
    clipped to the main body so the mounting tab stays solid."""
    outer = Polygon(loop)
    web = outer.buffer(-margin, quad_segs=12)
    web = web.intersection(box(spine + margin, -1e4, 1e4, 1e4))
    rib = LineString([(spine + w, 0.0), (S[0] / 2, S[1] / 2)]) \
        .buffer(rib_w / 2, cap_style=2)
    pieces = web.difference(rib)
    geoms = getattr(pieces, "geoms", [pieces])
    out = []
    for g in geoms:
        g = g.buffer(-fillet, quad_segs=10).buffer(fillet, quad_segs=10)
        for gg in getattr(g, "geoms", [g]):
            if gg.area >= min_area:
                out.append(gg.simplify(0.05))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--b", type=float, default=104.545,
                    help="pivot-to-anchor spacing (mm)")
    ap.add_argument("--phi", type=float, default=127.5794,
                    help="anchor direction from pivot (deg)")
    ap.add_argument("--w", type=float, default=7.532,
                    help="universal standoff/fillet radius (mm) -- must "
                         "match the clevis --edge-standoff")
    ap.add_argument("--spine", type=float, default=-105.208,
                    help="spine x position relative to the pivot (mm)")
    ap.add_argument("--tab-l", type=float, default=45.189,
                    help="mounting tab length past the spine (mm)")
    ap.add_argument("--tab-y0", type=float, default=27.615)
    ap.add_argument("--tab-y1", type=float, default=55.231)
    ap.add_argument("--hole-d", type=float, default=3.3,
                    help="all hole diameters (pivot, anchor, mounting)")
    ap.add_argument("--mount-dx", type=float, default=35.0)
    ap.add_argument("--mount-dy", type=float, default=15.0)
    ap.add_argument("--mount-x0", type=float, default=6.19,
                    help="near mount column offset from the spine (mm)")
    ap.add_argument("--mount-y0", type=float, default=5.47,
                    help="first mount row offset above the tab bottom")
    ap.add_argument("--margin", type=float, default=12.5,
                    help="web rib width along the outer edges (mm)")
    ap.add_argument("--rib-w", type=float, default=8.0,
                    help="diagonal rib width (mm)")
    ap.add_argument("--fillet", type=float, default=4.5,
                    help="cutout corner radius (mm)")
    ap.add_argument("--no-cutouts", action="store_true")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "output"
                    / "frame_generated.dxf")
    a = ap.parse_args()
    a.out.parent.mkdir(parents=True, exist_ok=True)

    verts, loop, S = build_outline(a.b, a.phi, a.w, a.spine,
                                   a.tab_l, a.tab_y0, a.tab_y1)

    doc = ezdxf.new("R2010", setup=False)
    doc.header["$INSUNITS"] = 4
    msp = doc.modelspace()
    msp.add_lwpolyline(verts, format="xyb", close=True,
                       dxfattribs={"layer": "CUT"})
    r = a.hole_d / 2
    msp.add_circle((0, 0), r, dxfattribs={"layer": "CUT"})
    msp.add_circle(S, r, dxfattribs={"layer": "CUT"})
    x1 = a.spine - a.mount_x0
    y1 = a.tab_y0 + a.mount_y0
    for cx in (x1, x1 - a.mount_dx):
        for cy in (y1, y1 + a.mount_dy):
            msp.add_circle((cx, cy), r, dxfattribs={"layer": "CUT"})

    n_cut = 0
    if not a.no_cutouts:
        cuts = build_cutouts(loop, S, a.spine, a.w, a.margin, a.rib_w,
                             a.fillet, min_area=150.0)
        for c in cuts:
            msp.add_lwpolyline(list(c.exterior.coords)[:-1], close=True,
                               dxfattribs={"layer": "CUT"})
        n_cut = len(cuts)

    doc.saveas(a.out)
    print(f"frame: b={a.b} phi={a.phi} w={a.w}  "
          f"S=({S[0]:.2f},{S[1]:.2f})  cutouts={n_cut}  -> {a.out}")


if __name__ == "__main__":
    main()
