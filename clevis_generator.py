#!/usr/bin/env python3
"""Parametric clevis joint generator for the von Mises toggle (build123d).

Modeled on the proven "adjusteed joiny" design (measured from the 3MF):
a solid clevis body with a narrow blade slot for the flat arm, a small
cross bore with counterbores on BOTH outer faces so the pin sits fully
flush, and a ribbed stem on the back for mounting. Reference part:
body 25.8 x 9.75 x 16.9 mm, slot 2.2 mm, bore 3.3 mm, counterbore
5.0 x 1.0 mm, stem 7.25 mm.

Generates as STEP (for Fusion assemblies) and STL (for the slicer):

    clevis.step/.stl   slotted body + counterbored cross bore + ribbed stem
    pin.step/.stl      flush pin: recessed head knub + chamfered tail
    arm.step/.stl      optional printed arm (flat bar, eye + band slot)

Usage:
    python clevis_generator.py                    # match measured joint
    python clevis_generator.py --pin-d 4 --blade-t 3
"""

import argparse
import math
from pathlib import Path

from build123d import (
    Align, Axis, BuildPart, BuildSketch, Circle, Cylinder, Box, GeomType,
    Locations, Mode, Plane, Polygon, SlotOverall, Torus, chamfer, extrude,
    export_step, export_stl,
)

C = Align.CENTER
MIN = Align.MIN
MAX = Align.MAX


BOSS_L = 8.0                     # socket boss length behind the body


def socket_depth(tab_stop_deg, rod_od, socket_d=10.5, body_l=20.0,
                 body_h=16.9, edge_standoff=7.53, stop_clr=0.25,
                 socket_clr=0.15):
    """Effective socket depth after the stop-wall cap. The single
    source of truth: the clevis cut, the rod length, and the build
    sheet all call this."""
    d = math.radians(180.0 - tab_stop_deg)
    x0 = (body_l - body_h / 2) - (edge_standoff + stop_clr) / math.sin(d)
    rs = rod_od / 2 + socket_clr
    wall_min = x0 - abs(rs * math.cos(d) / math.sin(d))
    return max(4.0, min(socket_d, BOSS_L + wall_min - 0.8))


def make_clevis(body_l, body_w, body_h, slot_w, slot_d, bore_d,
                cb_d, cb_dep, stem_d, stem_l, rib_r, rib_pitch,
                tab_l, tab_h, tab_stop_deg=105.4, edge_standoff=7.53,
                stop_clr=0.25, rod_mount="socket", rod_od=8.0,
                socket_d=10.5, socket_clr=0.15):
    """Clevis body: fork end at +X (rounded, bore concentric), ribbed
    stem at -X, stopper tab wedge on top. Pin axis along Y.

    The tab's stop face is inclined so it lands FLUSH on the frame
    plate's edge (edge_standoff mm from the pivot) exactly at the
    pivot angle tab_stop_deg -- that contact dictates the state-1
    interior angle."""
    r = body_h / 2
    bore_x = body_l - r
    # stop-wall plane: inclined so the frame-plate edge (edge_standoff
    # from the pivot) lands flush on it at gamma = tab_stop_deg. The
    # channel end wall below and the tab face above share this plane.
    d = math.radians(180.0 - tab_stop_deg)
    x0 = bore_x - (edge_standoff + stop_clr) / math.sin(d)  # wall at z=0
    f = lambda z: x0 + z * math.cos(d) / math.sin(d)
    with BuildPart() as p:
        Box(body_l - r, body_w, body_h, align=(MIN, C, C))
        with Locations((bore_x, 0, 0)):
            Cylinder(r, body_w, rotation=(90, 0, 0))
        if tab_h > 0:
            with BuildSketch(Plane.XZ):
                Polygon((f(r), r), (f(r + tab_h), r + tab_h),
                        (f(r) - tab_l, r), align=None)
            extrude(amount=body_w / 2, both=True)
        if rod_mount == "socket":
            # female socket: flush-sided boss, the solid rod glues in.
            # Depth is auto-capped so the socket floor clears the
            # inclined channel wall everywhere on its circumference
            # (steep stop angles pull the wall toward the boss).
            eff_d = socket_depth(tab_stop_deg, rod_od, socket_d,
                                 body_l, body_h, edge_standoff,
                                 stop_clr, socket_clr)
            if eff_d < socket_d:
                print(f"  socket depth capped {socket_d:.1f} -> "
                      f"{eff_d:.1f} mm (stop wall at "
                      f"{tab_stop_deg:.1f} deg)")
            Box(BOSS_L, body_w, rod_od + 4.4, align=(MAX, C, C))
            with Locations((-BOSS_L + eff_d / 2, 0, 0)):
                Cylinder(rod_od / 2 + socket_clr, eff_d,
                         rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        else:
            with Locations((0, 0, 0)):
                Cylinder(stem_d / 2, stem_l, rotation=(0, -90, 0),
                         align=(C, C, MIN))
            n_ribs = max(1, int((stem_l - rib_r) / rib_pitch))
            for i in range(n_ribs):
                with Locations((-(i + 1) * rib_pitch, 0, 0)):
                    Torus(stem_d / 2, rib_r, rotation=(0, 90, 0))
        H = body_h / 2 + tab_h + stem_d + 2      # channel cut half-height
        xf = body_l + r                          # past the fork tip
        with BuildSketch(Plane.XZ):
            Polygon((f(-H), -H), (f(H), H), (xf, H), (xf, -H),
                    align=None)
        extrude(amount=slot_w / 2, both=True, mode=Mode.SUBTRACT)
        with Locations((body_l - r, 0, 0)):
            Cylinder(bore_d / 2, body_w * 2, rotation=(90, 0, 0),
                     mode=Mode.SUBTRACT)
        for s in (-1, 1):
            with Locations((body_l - r, s * body_w / 2, 0)):
                Cylinder(cb_d / 2, 2 * cb_dep, rotation=(90, 0, 0),
                         mode=Mode.SUBTRACT)
    return p.part


def make_pin(shaft_d, pin_len, head_d, head_h, tip_ch, ridge_z=None,
             ridge_r=0.22):
    """Flush clevis pin along +Z: head knub sits inside the counterbore;
    optional snap ridge that clicks into the far counterbore."""
    with BuildPart() as p:
        Cylinder(shaft_d / 2, pin_len, align=(C, C, MIN))
        tip = p.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.Z)[-1]
        chamfer(tip, length=tip_ch)
        Cylinder(head_d / 2, head_h, align=(C, C, MAX))
        if ridge_z is not None:
            with Locations((0, 0, ridge_z)):
                Torus(shaft_d / 2, ridge_r)
    return p.part


def make_arm(R_mm, arm_w, arm_t, bore_d, slot_l, slot_w, slot_inset):
    """Optional printed arm: eye center at origin, tip at +X R_mm."""
    with BuildPart() as p:
        with BuildSketch(Plane.XY):
            with Locations((R_mm / 2, 0)):
                SlotOverall(R_mm + arm_w, arm_w)
        extrude(amount=arm_t)
        with BuildSketch(Plane.XY):
            Circle(bore_d / 2)
            with Locations((R_mm - slot_inset, 0)):
                SlotOverall(slot_l, slot_w, rotation=90)
        extrude(amount=arm_t, mode=Mode.SUBTRACT)
    return p.part


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pin-d", type=float, default=3.0,
                    help="pin shaft diameter (mm)")
    ap.add_argument("--clr-r", type=float, default=0.15,
                    help="radial clearance, bore over pin (mm)")
    ap.add_argument("--blade-t", type=float, default=2.0,
                    help="arm blade thickness the slot accepts (mm)")
    ap.add_argument("--clr-f", type=float, default=0.1,
                    help="slot clearance per face over the blade (mm)")
    ap.add_argument("--body-l", type=float, default=20.0,
                    help="clevis body length, stem face to fork tip (mm)")
    ap.add_argument("--body-w", type=float, default=9.75,
                    help="clevis body width across the pin axis (mm)")
    ap.add_argument("--body-h", type=float, default=16.9,
                    help="clevis body height (mm)")
    ap.add_argument("--slot-d", type=float, default=13.0,
                    help="blade slot depth from fork tip (mm)")
    ap.add_argument("--cb-d", type=float, default=5.0,
                    help="counterbore diameter for the flush pin head (mm)")
    ap.add_argument("--cb-dep", type=float, default=1.0,
                    help="counterbore depth per face (mm)")
    ap.add_argument("--rod-mount", choices=["socket", "stem"],
                    default="socket",
                    help="socket: solid rod glues INTO the clevis; "
                         "stem: ribbed spigot presses into a tube")
    ap.add_argument("--rod-od", type=float, default=5.0,
                    help="rod outer diameter for the socket (mm)")
    ap.add_argument("--socket-d", type=float, default=10.5,
                    help="socket depth (mm); capped so the socket floor "
                         "keeps ~1 mm to the blade channel")
    ap.add_argument("--socket-clr", type=float, default=0.15,
                    help="radial clearance, socket over rod (mm)")
    ap.add_argument("--stem-d", type=float, default=7.25,
                    help="stem-mode: mounting stem diameter (mm)")
    ap.add_argument("--stem-l", type=float, default=6.0,
                    help="stem-mode: mounting stem length (mm)")
    ap.add_argument("--tab-l", type=float, default=6.0,
                    help="stopper tab length along the body (mm)")
    ap.add_argument("--tab-h", type=float, default=4.0,
                    help="stopper tab height above the body (0 = none)")
    ap.add_argument("--tab-stop", type=float, default=105.4,
                    help="flush-face angle (deg); with the default 0.25 mm "
                         "running clearance the hard stop lands ~1.4 deg lower "
                         "(105.4 -> measured 104.0 state-1 interior angle)")
    ap.add_argument("--edge-standoff", type=float, default=7.53,
                    help="frame edge distance from the pivot hole (mm), "
                         "measured from the frame DXF")
    ap.add_argument("--R", type=float, default=1.7178,
                    help="arm length in sketch units (printed arm)")
    ap.add_argument("--scale", type=float, default=100.0,
                    help="mm per sketch unit")
    ap.add_argument("--arm-w", type=float, default=12.0,
                    help="printed arm width (mm)")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "output" / "clevis",
                    help="output directory")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    bore_d = a.pin_d + 2 * a.clr_r
    slot_w = a.blade_t + 2 * a.clr_f
    pin_len = a.body_w - a.cb_dep - 0.1       # head base to tip: flush
    head_d = a.cb_d - 0.2                     # slides into the counterbore
    head_h = a.cb_dep - 0.05

    parts = {
        "clevis": make_clevis(a.body_l, a.body_w, a.body_h, slot_w,
                              a.slot_d, bore_d, a.cb_d, a.cb_dep,
                              a.stem_d, a.stem_l, rib_r=0.45,
                              rib_pitch=1.6, tab_l=a.tab_l, tab_h=a.tab_h,
                              tab_stop_deg=a.tab_stop,
                              edge_standoff=a.edge_standoff,
                              rod_mount=a.rod_mount, rod_od=a.rod_od,
                              socket_d=a.socket_d,
                              socket_clr=a.socket_clr),
        "pin": make_pin(a.pin_d, pin_len, head_d, head_h, tip_ch=0.5,
                        ridge_z=pin_len - a.cb_dep / 2),
        "arm": make_arm(a.R * a.scale, a.arm_w, a.blade_t, bore_d,
                        slot_l=8.0, slot_w=3.0, slot_inset=8.0),
    }

    print(f"pin {a.pin_d} -> bore {bore_d:.2f}   blade {a.blade_t} -> "
          f"slot {slot_w:.2f}   counterbore {a.cb_d}x{a.cb_dep}   "
          f"pin length {pin_len:.2f}")
    for name, part in parts.items():
        bb = part.bounding_box()
        sz = bb.max - bb.min
        export_step(part, str(a.out / f"{name}.step"))
        export_stl(part, str(a.out / f"{name}.stl"))
        print(f"  {name:8s} {sz.X:7.1f} x {sz.Y:6.1f} x {sz.Z:6.1f} mm   "
              f"vol {part.volume/1000:6.1f} cm3  -> {name}.step/.stl")


if __name__ == "__main__":
    main()
