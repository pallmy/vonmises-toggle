#!/usr/bin/env python3
"""Parametric von Mises truss generator.

Generates watertight, print-ready STLs of a two-bar von Mises truss.
The driving parameter is the angle of the top spans (bars) with the horizon.

Derived geometry, matching the classic formulation:
    half-span   a = span / 2
    apex height h = a * tan(alpha)
    bar length  L = a / cos(alpha)

Usage:
    python generate_truss.py --angles 15 25 35 45
    python generate_truss.py --angles 30 --span 140 --bar-w 8 --bar-h 6
"""

import argparse
import math
from pathlib import Path

import numpy as np
import trimesh


def box(size, center):
    """Axis-aligned box of `size` (x, y, z) centered at `center`."""
    b = trimesh.creation.box(extents=size)
    b.apply_translation(center)
    return b


def bar(p0, p1, width, height, embed=2.0):
    """Rectangular bar from p0 to p1 (in the XZ plane), cross-section
    height x width, extended by `embed` at both ends so it fuses into
    the support and apex blocks."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = np.linalg.norm(d)
    b = trimesh.creation.box(extents=[length + 2 * embed, width, height])
    angle = math.atan2(d[2], d[0])
    b.apply_transform(trimesh.transformations.rotation_matrix(angle, [0, -1, 0]))
    b.apply_translation((p0 + p1) / 2)
    return b


def build_truss(angle_deg, span, bar_w, bar_h, base_t, base_margin,
                support_h, support_w, apex_size):
    """Assemble one truss solid for a given top-span angle (degrees)."""
    alpha = math.radians(angle_deg)
    a = span / 2.0
    apex_h = a * math.tan(alpha)
    bar_len = a / math.cos(alpha)

    z_pivot = base_t + support_h          # height of the bar end points
    apex = np.array([0.0, 0.0, z_pivot + apex_h])
    pivot_l = np.array([-a, 0.0, z_pivot])
    pivot_r = np.array([+a, 0.0, z_pivot])

    depth = max(support_w, bar_w + 4)     # y-extent of base and supports

    parts = [
        box([span + 2 * base_margin, depth, base_t],
            [0, 0, base_t / 2]),
        box([support_w, depth, support_h],
            [-a, 0, base_t + support_h / 2]),
        box([support_w, depth, support_h],
            [+a, 0, base_t + support_h / 2]),
        bar(pivot_l, apex, bar_w, bar_h),
        bar(pivot_r, apex, bar_w, bar_h),
        box([apex_size, bar_w + 2, apex_size],
            apex),
    ]

    solid = trimesh.boolean.union(parts, engine="manifold")
    n_bodies = len(solid.split(only_watertight=False))
    return solid, {"apex_height": apex_h, "bar_length": bar_len,
                   "overall_height": z_pivot + apex_h + apex_size / 2,
                   "bodies": n_bodies}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--angles", type=float, nargs="+",
                    default=[15, 25, 35, 45],
                    help="top-span angles with the horizon, in degrees")
    ap.add_argument("--span", type=float, default=120.0,
                    help="distance between support pivots (mm)")
    ap.add_argument("--bar-w", type=float, default=8.0,
                    help="bar width, horizontal cross-section dimension (mm)")
    ap.add_argument("--bar-h", type=float, default=6.0,
                    help="bar height, vertical cross-section dimension (mm)")
    ap.add_argument("--base-t", type=float, default=4.0,
                    help="baseplate thickness (mm)")
    ap.add_argument("--base-margin", type=float, default=10.0,
                    help="baseplate overhang past each support (mm)")
    ap.add_argument("--support-h", type=float, default=10.0,
                    help="support block height above baseplate (mm)")
    ap.add_argument("--support-w", type=float, default=12.0,
                    help="support block width along the span (mm)")
    ap.add_argument("--apex-size", type=float, default=12.0,
                    help="apex joint block size (mm)")
    ap.add_argument("--out", type=Path, default=Path(__file__).parent / "output",
                    help="output directory for STLs")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)

    print(f"{'angle':>6} {'apex h':>8} {'bar L':>8} {'height':>8} "
          f"{'watertight':>10} {'bodies':>6}  file")
    for angle in args.angles:
        if not 0 < angle < 90:
            print(f"{angle:>6}  skipped: angle must be between 0 and 90")
            continue
        solid, dims = build_truss(
            angle, args.span, args.bar_w, args.bar_h, args.base_t,
            args.base_margin, args.support_h, args.support_w, args.apex_size)
        name = f"vonmises_truss_a{angle:g}.stl"
        solid.export(args.out / name)
        print(f"{angle:>6g} {dims['apex_height']:>8.2f} "
              f"{dims['bar_length']:>8.2f} {dims['overall_height']:>8.2f} "
              f"{str(solid.is_watertight):>10} {dims['bodies']:>6}  {name}")


if __name__ == "__main__":
    main()
