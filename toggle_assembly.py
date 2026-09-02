#!/usr/bin/env python3
"""Posed assembly builder for the von Mises toggle.

Uses the REAL fiberglass frame cutout (frame_cutout.dxf, mm): outer
profile with arc bulges, lightening cutouts, and the six 3.3 mm holes.
The pivot P and band anchor S are two of those holes; the clevis slot
straddles the plate at P, the flush snap pin crosses it, and the
carbon-tube arm rod presses onto the clevis stem.

Assembly plane is XY (the DXF plane), pin axis +Z, pivot at origin.
The clevis + rod group is posed at gamma degrees from the P->S frame
direction, exactly like the 2D model. Rod length is anchored to the
real frame: scale = |PS|_dxf / b, so the tip lands at R * scale from P.

Exports one STEP per gamma (opens as an assembly in Fusion 360 and
SolidWorks) plus a combined STL for quick viewing.

Usage:
    python toggle_assembly.py --gamma 104
    python toggle_assembly.py --gamma 104 256
"""

import argparse
import math
from pathlib import Path

import ezdxf
from build123d import (
    Align, BuildPart, BuildSketch, Box, Circle, Compound, Cylinder,
    Location, Locations, Mode, Plane, Polygon, extrude, export_step,
    export_stl,
)

from clevis_generator import BOSS_L, make_clevis, make_pin, socket_depth

SOCKET_D = 10.5                  # nominal socket depth (pre-cap)

C = Align.CENTER
MIN = Align.MIN

def detect_datums(path):
    """Find the pivot and anchor holes in a frame DXF.

    Rule: among the small circles, the four mounting holes cluster
    within 42 mm of each other; the pivot and anchor stand alone.
    The pivot is the isolated hole with the largest x (fork corner)."""
    doc = ezdxf.readfile(str(path))
    centers = [(e.dxf.center.x, e.dxf.center.y)
               for e in doc.modelspace()
               if e.dxftype() == "CIRCLE" and e.dxf.radius < 3.0]
    # x-order invariant: the 4 mounting holes sit on the tab, left of
    # the spine, and the anchor can never reach that far left
    # (S_x >= -b > spine - mount offset); the pivot is the fork corner
    # at max x. So: 4 leftmost = mounts, rightmost = pivot, the
    # remaining one = anchor.
    if len(centers) != 6:
        raise ValueError(f"expected 6 holes, got {len(centers)} "
                         f"in {path}")
    centers.sort(key=lambda c: c[0])
    return centers[5], centers[4]    # pivot (max x), anchor


def bulge_points(p1, p2, b, seg=16):
    """Intermediate points of a DXF bulge arc from p1 to p2 (exclusive)."""
    (x1, y1), (x2, y2) = p1, p2
    theta = 4.0 * math.atan(b)
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    if L < 1e-12 or abs(b) < 1e-12:
        return []
    r = L / (2.0 * math.sin(theta / 2.0))
    ang = math.atan2(dy, dx) + (math.pi - theta) / 2.0
    cx, cy = x1 + r * math.cos(ang), y1 + r * math.sin(ang)
    a1 = math.atan2(y1 - cy, x1 - cx)
    pts = []
    for i in range(1, seg):
        a = a1 + theta * i / seg
        pts.append((cx + abs(r) * math.cos(a), cy + abs(r) * math.sin(a)))
    return pts


def polyline_points(e, seg=16):
    """Discretize a closed LWPOLYLINE (with bulges) to a point loop."""
    vs = [(v[0], v[1], v[4]) for v in e]      # x, y, bulge
    pts = []
    n = len(vs)
    for i in range(n):
        x, y, b = vs[i]
        nxt = vs[(i + 1) % n]
        pts.append((x, y))
        if abs(b) > 1e-12:
            pts.extend(bulge_points((x, y), (nxt[0], nxt[1]), b, seg))
    return pts


def make_frame_from_dxf(path, blade_t, pivot):
    """Extrude the DXF cutout to a plate; pivot hole moved to origin."""
    doc = ezdxf.readfile(str(path))
    loops, circles = [], []
    for e in doc.modelspace():
        if e.dxftype() == "LWPOLYLINE":
            loops.append(polyline_points(e))
        elif e.dxftype() == "CIRCLE":
            c = e.dxf.center
            circles.append(((c.x, c.y), e.dxf.radius))
    area = lambda pts: (max(p[0] for p in pts) - min(p[0] for p in pts)) \
        * (max(p[1] for p in pts) - min(p[1] for p in pts))
    loops.sort(key=area, reverse=True)
    px, py = pivot
    with BuildPart() as p:
        with BuildSketch(Plane.XY):
            Polygon(*[(x - px, y - py) for x, y in loops[0]], align=None)
            for hole in loops[1:]:
                Polygon(*[(x - px, y - py) for x, y in hole], align=None,
                        mode=Mode.SUBTRACT)
            for (cx, cy), r in circles:
                with Locations((cx - px, cy - py)):
                    Circle(r, mode=Mode.SUBTRACT)
        extrude(amount=blade_t / 2, both=True)
    return p.part


def make_rod(od, wall, length):
    """Rod stand-in along -X (solid when wall >= od/2)."""
    with BuildPart() as p:
        Cylinder(od / 2, length, rotation=(0, -90, 0), align=(C, C, MIN))
        if wall < od / 2:
            Cylinder(od / 2 - wall, length * 2, rotation=(0, -90, 0),
                     mode=Mode.SUBTRACT)
    return p.part


def build_assembly(gamma, a, frame, pivot, anchor):
    bore_d = a.pin_d + 2 * a.clr_r
    slot_w = a.blade_t + 2 * a.clr_f
    pin_len = a.body_w - a.cb_dep - 0.1
    r = a.body_h / 2
    bore_off = a.body_l - r

    b_dxf = math.hypot(anchor[0] - pivot[0], anchor[1] - pivot[1])
    phi = math.degrees(math.atan2(anchor[1] - pivot[1],
                                  anchor[0] - pivot[0]))
    scale = b_dxf / a.b
    R_mm = a.R * scale
    tab_stop = getattr(a, "tab_stop", 105.4)
    sock = socket_depth(tab_stop, a.rod_od, SOCKET_D,
                        a.body_l, a.body_h)
    seat = sock - BOSS_L                   # socket floor, from body face
    rod_len = R_mm - bore_off + seat
    theta = phi - gamma                        # arm direction in XY

    clevis = make_clevis(a.body_l, a.body_w, a.body_h, slot_w, a.slot_d,
                         bore_d, a.cb_d, a.cb_dep, a.stem_d, a.stem_l,
                         rib_r=0.45, rib_pitch=1.6,
                         tab_l=a.tab_l, tab_h=a.tab_h,
                         tab_stop_deg=tab_stop,
                         rod_mount="socket", rod_od=a.rod_od,
                         socket_d=SOCKET_D)
    pin = make_pin(a.pin_d, pin_len, a.cb_d - 0.2, a.cb_dep - 0.05,
                   tip_ch=0.5, ridge_z=pin_len - a.cb_dep / 2)
    rod = make_rod(a.rod_od, a.rod_wall, rod_len)

    # clevis part frame: bore at (bore_off, 0, 0), axis Y, stem -> -X.
    # rotate X+90 to put the bore axis on Z (slot width follows), then
    # rotate about Z so the stem/rod point along the arm direction.
    place = (Location((0, 0, 0), (0, 0, 1), theta - 180)
             * Location((0, 0, 0), (1, 0, 0), 90)
             * Location((-bore_off, 0, 0)))
    clevis_w = clevis.moved(place)
    rod_w = rod.moved(place * Location((seat, 0, 0)))
    pin_w = pin.moved(Location((0, 0, -a.body_w / 2 + a.cb_dep + 0.05)))

    asm = Compound(children=[frame, clevis_w, pin_w, rod_w])
    asm.label = f"vonmises_toggle_g{gamma:g}"
    print(f"  scale {scale:.1f} mm/unit   b {b_dxf:.1f}   R {R_mm:.1f}   "
          f"phi {phi:.1f}   arm at {theta:.1f} deg")
    return asm


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--gamma", type=float, nargs="+", default=[104.0],
                    help="pose angle(s) between frame and arm, deg")
    ap.add_argument("--b", type=float, default=0.8803)
    ap.add_argument("--R", type=float, default=1.7178)
    ap.add_argument("--dxf", type=Path,
                    default=Path(__file__).parent / "frame_cutout.dxf")
    ap.add_argument("--pin-d", type=float, default=3.0)
    ap.add_argument("--clr-r", type=float, default=0.15)
    ap.add_argument("--blade-t", type=float, default=2.0)
    ap.add_argument("--clr-f", type=float, default=0.1)
    ap.add_argument("--body-l", type=float, default=20.0)
    ap.add_argument("--body-w", type=float, default=9.75)
    ap.add_argument("--body-h", type=float, default=16.9)
    ap.add_argument("--slot-d", type=float, default=13.0)
    ap.add_argument("--cb-d", type=float, default=5.0)
    ap.add_argument("--cb-dep", type=float, default=1.0)
    ap.add_argument("--stem-d", type=float, default=7.25)
    ap.add_argument("--stem-l", type=float, default=6.0)
    ap.add_argument("--tab-l", type=float, default=6.0)
    ap.add_argument("--tab-h", type=float, default=4.0)
    ap.add_argument("--rod-od", type=float, default=5.0)
    ap.add_argument("--rod-wall", type=float, default=0.75)
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "output" / "assembly")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    pivot, anchor = detect_datums(a.dxf)
    frame = make_frame_from_dxf(a.dxf, a.blade_t, pivot)
    for g in a.gamma:
        asm = build_assembly(g, a, frame, pivot, anchor)
        bb = asm.bounding_box()
        sz = bb.max - bb.min
        stem = f"toggle_assembly_g{g:g}"
        export_step(asm, str(a.out / f"{stem}.step"))
        export_stl(asm, str(a.out / f"{stem}.stl"))
        print(f"gamma {g:6g}   envelope {sz.X:6.1f} x {sz.Y:6.1f} x "
              f"{sz.Z:5.1f} mm   -> {stem}.step/.stl")


if __name__ == "__main__":
    main()
