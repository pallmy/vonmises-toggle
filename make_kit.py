#!/usr/bin/env python3
"""One-command fabrication kit for the von Mises toggle.

Runs the whole parametric chain and sorts the outputs by process:

    fab/laser/frame_cutout.dxf     laser-cut fiberglass frame (as-is)
    fab/print/clevis.stl           3D prints
    fab/print/pin.stl
    fab/cad/toggle_assembly_*.step posed assemblies (Fusion / SolidWorks)
    fab/BUILD_SHEET.md             parameters, cut lengths, stop angles

The frame DXF is the fabrication master for the plate; the model
parameters are cross-checked against it (pivot-anchor spacing and
direction) so the printed parts always match the laser-cut frame.

Usage:
    python make_kit.py                     # defaults: states 104 / 233
    python make_kit.py --gamma 104 233 --pin-d 3
"""

import argparse
import math
import shutil
import subprocess
import sys
from datetime import date
from pathlib import Path

from build123d import export_step, export_stl

import toggle_assembly as ta
from clevis_generator import make_clevis, make_pin, socket_depth

HERE = Path(__file__).parent


def render_preview(frame_dxf, pivot, anchor, R_mm, g1, g2, out_png):
    """Draw the configuration: frame planform, arm in both states,
    angle arcs and spans labeled. This is the 'look before you cut'."""
    import ezdxf
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    px, py = pivot
    doc = ezdxf.readfile(str(frame_dxf))
    fig, ax = plt.subplots(figsize=(11, 9))
    for e in doc.modelspace():
        if e.dxftype() == "LWPOLYLINE":
            pts = ta.polyline_points(e)
            xs = [p[0] - px for p in pts] + [pts[0][0] - px]
            ys = [p[1] - py for p in pts] + [pts[0][1] - py]
            ax.plot(xs, ys, color="#8a8070", lw=1.6)
        elif e.dxftype() == "CIRCLE":
            c = e.dxf.center
            ax.add_patch(plt.Circle((c.x - px, c.y - py), e.dxf.radius,
                                    fill=False, color="#8a8070", lw=1.2))
    S = (anchor[0] - px, anchor[1] - py)
    phi = math.degrees(math.atan2(S[1], S[0]))
    b_mm = math.hypot(*S)
    for g, col, ls, name in ((g1, "#4d47a3", "-", "state 1"),
                             (g2, "#b0436a", "--", "state 2")):
        th = math.radians(phi - g)
        T = (R_mm * math.cos(th), R_mm * math.sin(th))
        ax.plot([0, T[0]], [0, T[1]], ls, color=col, lw=3)
        ax.plot(*T, "o", color=col, ms=10)
        ax.plot([S[0], T[0]], [S[1], T[1]], ":", color="#1D9E75", lw=1.4)
        c = math.hypot(T[0] - S[0], T[1] - S[1])
        ax.annotate(f"{name}\nγ = {g:g}°\nspan = {c:.1f} mm",
                    T, textcoords="offset points", xytext=(12, 8),
                    fontsize=11, color=col)
        arc_r = 22 + (8 if g == g2 else 0)
        aa = [math.radians(phi - g + i * g / 60) for i in range(61)]
        ax.plot([arc_r * math.cos(a) for a in aa],
                [arc_r * math.sin(a) for a in aa], color=col, lw=0.9)
    ax.plot(0, 0, "o", ms=8, mfc="w", mec="#555", mew=1.6)
    ax.plot(*S, "o", ms=7, mfc="w", mec="#555", mew=1.6)
    ax.annotate("P", (0, 0), textcoords="offset points", xytext=(-16, -14))
    ax.annotate("S (anchor)", S, textcoords="offset points",
                xytext=(-30, 12))
    ax.set_title(f"configuration:  b = {b_mm:.1f} mm   R = {R_mm:.1f} mm"
                 f"   φ = {phi:.1f}°   states γ = {g1:g}° / {g2:g}°",
                 fontsize=12)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--b", type=float, default=0.8803,
                    help="frame leg P-S, sketch units (your AutoCAD b)")
    ap.add_argument("--R", type=float, default=1.7178,
                    help="arm length P-T, sketch units")
    ap.add_argument("--gamma1", type=float, default=104.0,
                    help="designed state-1 interior angle (deg); sets "
                         "the clevis stop face, compensation included")
    ap.add_argument("--scale", type=float, default=118.76,
                    help="mm per sketch unit (sets the physical size)")
    ap.add_argument("--gamma", type=float, nargs="+", default=None,
                    help="poses to export (default: gamma1 and the "
                         "measured state-2 stop)")
    ap.add_argument("--gamma2", type=float, default=None,
                    help="state-2 interior angle. Default: matched to "
                         "state 1 (mirror, 360 - gamma1). Specify to "
                         "override -- e.g. 233, the measured plate-"
                         "corner stop for the default geometry")
    ap.add_argument("--pin-d", type=float, default=3.0)
    ap.add_argument("--clr-r", type=float, default=0.15)
    ap.add_argument("--blade-t", type=float, default=2.0)
    ap.add_argument("--clr-f", type=float, default=0.1)
    ap.add_argument("--rod-od", type=float, default=5.0)
    ap.add_argument("--frame-mode", choices=["generated", "original"],
                    default="generated",
                    help="generate the laser DXF from parameters, or "
                         "copy the original hand-drawn file")
    ap.add_argument("--phi", type=float, default=127.5794,
                    help="anchor direction for the generated frame (deg)")
    ap.add_argument("--standoff", type=float, default=7.532,
                    help="frame edge standoff = clevis stop calibration")
    ap.add_argument("--dxf", type=Path, default=HERE / "frame_cutout.dxf",
                    help="source file for --frame-mode original")
    ap.add_argument("--preview", action="store_true",
                    help="draw the configuration (frame + both arm "
                         "states, angles and spans labeled), open it, "
                         "and stop -- no parts generated")
    ap.add_argument("--no-open", action="store_true",
                    help="with --preview: write the PNG but don't open")
    ap.add_argument("--out", type=Path, default=HERE / "fab")
    a = ap.parse_args()

    # fixed clevis geometry (measured from the proven joint)
    body_l, body_w, body_h = 20.0, 9.75, 16.9
    slot_d, cb_d, cb_dep = 13.0, 5.0, 1.0
    stem_d, stem_l = 7.25, 6.0
    tab_l, tab_h = 6.0, 4.0

    laser = a.out / "laser"
    prints = a.out / "print"
    cadd = a.out / "cad"
    for d in (laser, prints, cadd):
        d.mkdir(parents=True, exist_ok=True)

    # the initial geometry, in mm
    b_mm_req = a.b * a.scale
    tab_stop = a.gamma1 + 1.4          # flush face -> hard stop offset
    g2_matched = a.gamma2 is None
    gamma2 = (360.0 - a.gamma1) if g2_matched else a.gamma2
    poses = a.gamma if a.gamma else [a.gamma1, gamma2]

    # frame: generate from parameters (or copy the hand-drawn master)
    frame_dxf = laser / "frame_cutout.dxf"
    if a.frame_mode == "generated":
        subprocess.run(
            [sys.executable, str(HERE / "generate_frame.py"),
             "--b", str(b_mm_req), "--phi", str(a.phi),
             "--w", str(a.standoff), "--out", str(frame_dxf)],
            check=True)
    else:
        shutil.copy(a.dxf, frame_dxf)

    pivot, anchor = ta.detect_datums(frame_dxf)
    b_dxf = math.hypot(anchor[0] - pivot[0], anchor[1] - pivot[1])
    phi = math.degrees(math.atan2(anchor[1] - pivot[1],
                                  anchor[0] - pivot[0]))
    scale = b_dxf / a.b
    R_mm = a.R * scale

    preview_png = a.out / "config_preview.png"
    render_preview(frame_dxf, pivot, anchor, R_mm, a.gamma1, gamma2,
                   preview_png)
    if a.preview:
        print(f"configuration: b = {b_dxf:.1f} mm  phi = {phi:.1f} deg  "
              f"R = {R_mm:.1f} mm  states {a.gamma1:g}/{gamma2:g} deg")
        print(f"preview -> {preview_png}")
        if not a.no_open and sys.platform == "darwin":
            subprocess.run(["open", str(preview_png)])
        return
    bore_d = a.pin_d + 2 * a.clr_r
    slot_w = a.blade_t + 2 * a.clr_f
    pin_len = body_w - cb_dep - 0.1
    sock = socket_depth(tab_stop, a.rod_od, ta.SOCKET_D, body_l, body_h)
    seat = sock - ta.BOSS_L
    rod_cut = R_mm - (body_l - body_h / 2) + seat

    clevis = make_clevis(body_l, body_w, body_h, slot_w, slot_d, bore_d,
                         cb_d, cb_dep, stem_d, stem_l, rib_r=0.45,
                         rib_pitch=1.6, tab_l=tab_l, tab_h=tab_h,
                         tab_stop_deg=tab_stop,
                         edge_standoff=a.standoff,
                         rod_mount="socket", rod_od=a.rod_od,
                         socket_d=ta.SOCKET_D)
    pin = make_pin(a.pin_d, pin_len, cb_d - 0.2, cb_dep - 0.05,
                   tip_ch=0.5, ridge_z=pin_len - cb_dep / 2)
    export_stl(clevis, str(prints / "clevis.stl"))
    export_stl(pin, str(prints / "pin.stl"))
    export_step(clevis, str(cadd / "clevis.step"))
    export_step(pin, str(cadd / "pin.step"))

    frame = ta.make_frame_from_dxf(frame_dxf, a.blade_t, pivot)
    for g in poses:
        asm = ta.build_assembly(g, argparse.Namespace(
            b=a.b, R=a.R, pin_d=a.pin_d, clr_r=a.clr_r,
            blade_t=a.blade_t, clr_f=a.clr_f, body_l=body_l,
            body_w=body_w, body_h=body_h, slot_d=slot_d, cb_d=cb_d,
            cb_dep=cb_dep, stem_d=stem_d, stem_l=stem_l, tab_l=tab_l,
            tab_h=tab_h, rod_od=a.rod_od, rod_wall=0.75,
            tab_stop=tab_stop), frame,
            pivot, anchor)
        export_step(asm, str(cadd / f"toggle_assembly_g{g:g}.step"))

    sheet = a.out / "BUILD_SHEET.md"
    sheet.write_text(f"""# Von Mises toggle — build sheet ({date.today()})

## Model
| quantity | value |
|---|---|
| frame leg b (P-S, from DXF) | {b_dxf:.2f} mm |
| frame direction phi | {phi:.2f} deg |
| scale | {scale:.2f} mm / sketch unit |
| arm length R (pivot to tip) | {R_mm:.1f} mm |
| state-1 interior angle (designed hard stop) | {a.gamma1} deg |
| state-2 interior angle ({"matched mirror" if g2_matched else "specified"}) | {gamma2:g} deg |

## Laser (fab/laser)
- `frame_cutout.dxf` ({a.frame_mode}) -- {a.blade_t} mm fiberglass.
  Pivot and anchor holes are 3.3 mm; frame edges and the corner arc
  sit {a.standoff} mm from the pivot, and the clevis stop faces are
  calibrated to that same number -- change one, change both.

## Print (fab/print)
- `clevis.stl` -- slot {slot_w:.2f} mm (blade {a.blade_t} + {a.clr_f}/face),
  bore {bore_d:.2f} mm, counterbores {cb_d} x {cb_dep} mm both faces,
  stop face flush at {tab_stop:.1f} deg (hard stop ~{a.gamma1} deg with
  0.25 mm running clearance). Print slot-up, 0.1-0.15 mm layers.
- `pin.stl` -- {a.pin_d} mm shaft, flush head, snap ridge. Print several;
  they are small.

## Cut stock
- rod Ø{a.rod_od} mm (solid): cut to **{rod_cut:.1f} mm**. It glues
  {sock:.1f} mm deep into the clevis socket (CA or epoxy); the
  tip then lands at R = {R_mm:.1f} mm from the pivot.
- TPU band: free length TBD pending the k hang test.

## Assemble
1. Clevis slot over the frame at the pivot hole, bores aligned.
2. Pin through: head seats flush in one counterbore, snap ridge clicks
   into the far one.
3. Glue the rod into the clevis socket. Band from anchor S to rod tip.

## CAD (fab/cad)
- `toggle_assembly_g104.step` / `g233.step`: both resting states, open
  as assemblies in Fusion or SolidWorks.
""")
    print(f"kit written to {a.out}")
    for p in sorted(a.out.rglob("*")):
        if p.is_file():
            print(f"  {p.relative_to(a.out)}  ({p.stat().st_size//1024} KB)")


if __name__ == "__main__":
    main()
