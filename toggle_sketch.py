#!/usr/bin/env python3
"""Parametric von Mises over-center toggle sketch generator.

Model (matches the AutoCAD sketch):
    P = main pivot (origin, fixed)
    S = spring anchor, fixed to the frame at distance b from P,
        in direction phi (degrees from horizontal)
    T = arm tip (mass), at distance R from P
    gamma = driving angle at P between frame leg P-S and arm P-T

Derived:
    top span  c   = |S - T|          (spring length)
    span/horizon  = angle of S->T with the horizontal
    angle at S    = between S->P and S->T
    angle at T    = between T->S and T->P

Reference config: b=0.8803, R=1.7178, gamma=104 -> span=2.1055,
angle at S = 52 deg, angle at T = 24 deg, span horizontal.

Exports a DXF (importable into Fusion 360 as a sketch: Insert > Insert DXF)
and a PNG preview per gamma value.

Usage:
    python toggle_sketch.py --gammas 104 120 140
    python toggle_sketch.py --gammas 104 --R 2.0 --b 0.88 --phi 127.5
"""

import argparse
import math
from pathlib import Path

import ezdxf
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def solve(gamma_deg, R, b, phi_deg):
    """Return point coordinates and derived dimensions for one config."""
    phi = math.radians(phi_deg)
    P = np.array([0.0, 0.0])
    S = b * np.array([math.cos(phi), math.sin(phi)])
    theta = phi - math.radians(gamma_deg)      # arm direction
    T = R * np.array([math.cos(theta), math.sin(theta)])

    c = float(np.linalg.norm(T - S))
    horizon = math.degrees(math.atan2(T[1] - S[1], T[0] - S[0]))
    ang_S = math.degrees(math.acos(
        max(-1.0, min(1.0, (b * b + c * c - R * R) / (2 * b * c)))))
    ang_T = 180.0 - gamma_deg - ang_S

    # conjugate bistable state: arm mirrored across the frame line P-S
    # (gamma' = 360 - gamma), same span -> same spring energy
    theta2 = phi + math.radians(gamma_deg)
    T2 = R * np.array([math.cos(theta2), math.sin(theta2)])
    return {"P": P, "S": S, "T": T, "T2": T2, "span": c, "horizon": horizon,
            "ang_S": ang_S, "ang_T": ang_T,
            "gamma2": (360.0 - gamma_deg) % 360.0}


def gamma_from_horizon(horizon_deg, R, b, phi_deg):
    """Invert the model: given the span's angle with the horizon,
    return the pivot angle gamma that produces it.

    The span leaves S in direction beta; T is where that ray crosses
    the arm circle |T| = R (positive root -> T on the far side of S).
    """
    phi = math.radians(phi_deg)
    S = b * np.array([math.cos(phi), math.sin(phi)])
    beta = math.radians(horizon_deg)
    u = np.array([math.cos(beta), math.sin(beta)])
    su = float(S @ u)
    disc = su * su - b * b + R * R
    if disc < 0:
        raise ValueError(f"no arm position gives a {horizon_deg} deg span")
    c = -su + math.sqrt(disc)
    T = S + c * u
    theta = math.degrees(math.atan2(T[1], T[0]))
    return (phi_deg - theta) % 360.0


def spring_points(S, T, coils=24, amp=0.04):
    """Zigzag polyline from S to T for drawing the spring."""
    S, T = np.asarray(S), np.asarray(T)
    d = T - S
    u = d / np.linalg.norm(d)
    n = np.array([-u[1], u[0]])
    pts = [S]
    for i in range(1, coils):
        f = i / coils
        off = amp * (1 if i % 2 else -1) if 1 < i < coils - 1 else 0
        pts.append(S + d * f + n * off)
    pts.append(T)
    return np.array(pts)


def export_dxf(sol, path, two_states=False):
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    P, S, T = sol["P"], sol["S"], sol["T"]
    msp.add_line(P, S, dxfattribs={"layer": "FRAME"})
    msp.add_line(P, T, dxfattribs={"layer": "ARM"})
    msp.add_lwpolyline(spring_points(S, T), dxfattribs={"layer": "SPRING"})
    for pt, r in ((P, 0.03), (S, 0.025), (T, 0.05)):
        msp.add_circle(pt, r, dxfattribs={"layer": "JOINTS"})
    if two_states:
        T2 = sol["T2"]
        msp.add_line(P, T2, dxfattribs={"layer": "STATE2"})
        msp.add_lwpolyline(spring_points(S, T2),
                           dxfattribs={"layer": "STATE2"})
        msp.add_circle(T2, 0.05, dxfattribs={"layer": "STATE2"})
    doc.saveas(path)


def export_png(sol, gamma, path, two_states=False):
    P, S, T = sol["P"], sol["S"], sol["T"]
    fig, ax = plt.subplots(figsize=(7, 5))
    if two_states:
        T2 = sol["T2"]
        ax.plot(*zip(P, T2), color="#D4537E", lw=2.5, ls="--", alpha=0.8)
        sp2 = spring_points(S, T2)
        ax.plot(sp2[:, 0], sp2[:, 1], color="#1D9E75", lw=1, alpha=0.35)
        ax.plot(*T2, "o", color="#D4537E", ms=11, mfc="none")
        ax.annotate(f"state 2 (γ={sol['gamma2']:.1f}°)", T2,
                    textcoords="offset points", xytext=(8, 8), fontsize=9)
        dead = -np.asarray(S) / np.linalg.norm(S) * np.linalg.norm(T)
        ax.plot(*zip(S, dead), color="gray", lw=0.8, ls=":", alpha=0.6)
    ax.plot(*zip(P, S), color="gray", lw=2)
    ax.plot(*zip(P, T), color="#7F77DD", lw=3)
    sp = spring_points(S, T)
    ax.plot(sp[:, 0], sp[:, 1], color="#1D9E75", lw=1.2)
    ax.plot(*P, "o", color="gray", ms=9, mfc="none")
    ax.plot(*S, "o", color="gray", ms=7, mfc="none")
    ax.plot(*T, "o", color="#7F77DD", ms=11)
    for pt, name in ((P, "P"), (S, "S"), (T, "T")):
        ax.annotate(name, pt, textcoords="offset points", xytext=(-12, 8))
    ax.set_title(
        f"γ={gamma:g}°   span={sol['span']:.4f}   "
        f"horizon={sol['horizon']:+.1f}°   S={sol['ang_S']:.1f}°   "
        f"T={sol['ang_T']:.1f}°")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--gammas", type=float, nargs="+", default=None,
                    help="drive by pivot angle(s) gamma, degrees")
    ap.add_argument("--horizons", type=float, nargs="+", default=None,
                    help="drive by span-to-horizon angle(s), degrees "
                         "(positive = tip end higher than anchor); "
                         "gamma is derived")
    ap.add_argument("--R", type=float, default=1.7178,
                    help="arm length P-T")
    ap.add_argument("--b", type=float, default=0.8803,
                    help="frame leg length P-S")
    ap.add_argument("--phi", type=float, default=127.53,
                    help="frame leg direction from horizontal, degrees "
                         "(default makes the span horizontal at gamma=104)")
    ap.add_argument("--two-states", action="store_true",
                    help="also draw the conjugate bistable state "
                         "(arm mirrored across the frame line, gamma' = "
                         "360 - gamma) in the PNG and on DXF layer STATE2")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "output",
                    help="output directory")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.gammas and args.horizons:
        ap.error("use either --gammas or --horizons, not both")
    if args.horizons is not None:
        runs = [(gamma_from_horizon(h, args.R, args.b, args.phi),
                 f"toggle_h{h:+g}") for h in args.horizons]
    else:
        gammas = args.gammas if args.gammas is not None else [104.0]
        runs = [(g, f"toggle_g{g:g}") for g in gammas]

    print(f"{'gamma':>7} {'span':>8} {'horizon':>8} {'ang S':>7} {'ang T':>7}"
          f"  files")
    for gamma, stem in runs:
        sol = solve(gamma, args.R, args.b, args.phi)
        export_dxf(sol, args.out / f"{stem}.dxf", args.two_states)
        export_png(sol, gamma, args.out / f"{stem}.png", args.two_states)
        print(f"{gamma:>7g} {sol['span']:>8.4f} {sol['horizon']:>+8.2f} "
              f"{sol['ang_S']:>7.2f} {sol['ang_T']:>7.2f}  {stem}.dxf/.png")


if __name__ == "__main__":
    main()
