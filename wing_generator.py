#!/usr/bin/env python3
"""Parametric wing-board hardware generator.

Rebuilds the foam-board wing stack from parameters, reverse-measured
from the as-built v1 CAD ('new attempt at von mises assembley.step'):

    foam board   length x chord x 5 foam, full-length spar slits
    cantilever   bridge: two feet standing on the foam (spar slots cut
                 up through them), 3 mm deck spanning between, clamp
                 holes, deck holes, top pocket, cross holes
    end caps     mirrored channel caps: foam pocket + a slit per spar
    clamp plate  1 mm laser plate matching the cantilever footprint

Spars are 10.02 x 1 mm carbon strips on edge: engagement 2.7 mm into
the foam, 7.5 mm proud into cantilever feet / cap slits.

Defaults reproduce v1 (chord 50, two spars at +/-3.5). Example v2:

    python wing_generator.py --chord 98 --spars -7 0 7 --suffix _3spar
"""

import argparse
from pathlib import Path

import ezdxf
from build123d import (
    Align, BuildPart, BuildSketch, Box, Cylinder, Locations,
    Mode, Plane, RectangleRounded, export_stl, extrude,
)

C = Align.CENTER
MIN = Align.MIN

SPAR_W, SPAR_T = 10.02, 1.0        # carbon strip, on edge
FOAM_T = 5.0
ENGAGE_FOAM = 0.0                  # SURFACE MOUNT: spar sits ON the foam
PROUD = SPAR_W                     # full spar height above the surface
SLIT_W = 1.0                       # slit width in feet/caps
SLOT_CLR = 0.1                     # slot ceiling clearance over the spar


def make_cantilever(spars, width=24.56, length=37.62, height=None,
                    foot_l=8.0, deck_t=3.0,
                    clamp_hole_d=2.1, clamp_dx=17.35, clamp_dy=10.78,
                    hub_hole_d=3.2, hub_pitch=5.45, hub_flange=21.94,
                    hub_seat_clr=0.15, hub_seat_depth=1.5,
                    clamp_boss_wall=1.8,
                    cross_hole_d=1.7, cross_hole_z=3.3):
    """Bridge cantilever. Origin: footprint center, feet bottoms z=0,
    length along X, width (chord direction) along Y, up +Z."""
    if height is None:
        height = PROUD + SLOT_CLR + deck_t   # slot ceiling flush datum
    hl = length / 2
    with BuildPart() as p:
        for s in (-1, 1):
            with Locations(((hl - foot_l / 2) * s, 0, 0)):
                Box(foot_l, width, height, align=(C, C, MIN))
        with Locations((0, 0, height - deck_t)):
            Box(length, width, deck_t, align=(C, C, MIN))
        # spar slots up through the feet; ceiling sits SLOT_CLR above
        # the spar so the spar rests flush on the foam surface
        for y in spars:
            with Locations((0, y, 0)):
                Box(length + 2, SLIT_W, PROUD + SLOT_CLR,
                    align=(C, C, MIN), mode=Mode.SUBTRACT)
        # bosses that wrap each clamp hole (the outer edge was too thin
        # to print) -- add material first, then drill through it
        boss_d = clamp_hole_d + 2 * clamp_boss_wall
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((sx * clamp_dx, sy * clamp_dy, 0)):
                    Cylinder(boss_d / 2, height, align=(C, C, MIN))
        # clamp holes through the feet + bosses
        with Locations(*[(sx * clamp_dx, sy * clamp_dy, 0)
                         for sx in (-1, 1) for sy in (-1, 1)]):
            Cylinder(clamp_hole_d / 2, height * 2 + 2,
                     mode=Mode.SUBTRACT)
        # hub bolt holes through the deck (match the 5 mm hub: four
        # O3 bolts on a +/-5.45 square, dead center)
        with Locations(*[(sx * hub_pitch, sy * hub_pitch, 0)
                         for sx in (-1, 1) for sy in (-1, 1)]):
            Cylinder(hub_hole_d / 2, height * 2 + 2,
                     mode=Mode.SUBTRACT)
        # hub flange seat recessed into the deck top
        seat = hub_flange + 2 * hub_seat_clr
        with BuildSketch(Plane.XY.offset(height)):
            RectangleRounded(seat, seat, 2)
        extrude(amount=-hub_seat_depth, mode=Mode.SUBTRACT)
        # horizontal cross holes through each foot
        for s in (-1, 1):
            with Locations(((hl - foot_l / 2) * s, 0, cross_hole_z)):
                Cylinder(cross_hole_d / 2, width + 2,
                         rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    return p.part


def make_end_cap(chord, spars, mirror=False, length=49.75, wall=4.0,
                 floor=4.0, end_wall=4.5, slit_clr=0.05,
                 guide_w=3.0, rise=10.0, guide_rise=None, end_wall_rise=None,
                 bolt_d=3.2, bolt_x_fracs=(0.4,), bolt_above=6.0,
                 outer_frame=True, truss_hole_d=0.0, truss_dy=30.0,
                 truss_dz=4.0, face_hole_d=3.3, face_dx=14.0,
                 face_dy=17.5):
    """Spar-holder block: a floor plate carrying a pair of TALL guide
    walls (spar holders) flanking each spar slit, rising the full spar
    height. Opening at -X; origin: open end center, floor bottom z=0.

    With `outer_frame` False (default) there are no side/end walls -- the
    bolt runs along Y through the spar holders and the spars, and the nut
    seats in the open space right after each holder. Set outer_frame True
    to add the solid side/end walls back (foam grip / a mounting face)."""
    if guide_rise is None:
        guide_rise = SPAR_W                # spar holders as tall as spar
    if end_wall_rise is None:
        end_wall_rise = rise
    height = floor + FOAM_T + rise
    outer_w = chord + 2 * wall
    depth = length - (end_wall if outer_frame else 0)
    bolt_z = floor + FOAM_T + bolt_above
    with BuildPart() as p:
        Box(length, chord, floor, align=(MIN, C, MIN))
        if outer_frame:
            for s in (-1, 1):
                with Locations((0, s * (chord / 2 + wall / 2), 0)):
                    Box(length, wall, height, align=(MIN, C, MIN))
            ew_h = floor + FOAM_T + end_wall_rise
            with Locations((length - end_wall, 0, 0)):
                Box(end_wall, outer_w, ew_h, align=(MIN, C, MIN))
        for y in spars:
            yy = -y if mirror else y
            off = SLIT_W / 2 + slit_clr + guide_w / 2
            for s in (-1, 1):
                with Locations((0, yy + s * off, floor + FOAM_T)):
                    Box(length, guide_w, guide_rise, align=(MIN, C, MIN))
        # bolt hole: through the spar holders + spars only, spanning just
        # the spar-holder band so the nut seats in open space after it.
        y_off = max(abs(s) for s in spars) + SLIT_W / 2 + slit_clr \
            + guide_w + 2
        for xf in bolt_x_fracs:
            with Locations((xf * depth, 0, bolt_z)):
                Cylinder(bolt_d / 2, 2 * y_off, rotation=(90, 0, 0),
                         mode=Mode.SUBTRACT)
        # 4 vertical hold-down holes through the floor (2x2 on the top
        # face), straddling the spar-holder band
        if face_hole_d > 0:
            for sx in (-1, 1):
                for sy in (-1, 1):
                    with Locations((length / 2 + sx * face_dx,
                                    sy * face_dy, 0)):
                        Cylinder(face_hole_d / 2,
                                 (floor + FOAM_T + guide_rise) * 2,
                                 mode=Mode.SUBTRACT)
        if outer_frame and truss_hole_d > 0:
            tz = floor + FOAM_T / 2
            for sy in (-1, 1):
                for sz in (-1, 1):
                    with Locations((length, sy * truss_dy / 2,
                                    tz + sz * truss_dz / 2)):
                        Cylinder(truss_hole_d / 2, end_wall * 3,
                                 rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part


def make_grip_clamp(length=42.0, width=26.0, base_t=5.0,
                    tooth_amp=0.6, tooth_pitch=2.0,
                    pilot_d=1.6, dx=17.35, dy=10.78):
    """Printed bottom clamp the top screws bite into: thick base with
    self-tap pilot holes on the cantilever corner pattern and a
    serrated top face that grips the foam underside. Print flat,
    teeth up."""
    import math
    hl, hw = length / 2, width / 2
    pts = [(-hl, 0), (hl, 0), (hl, base_t)]
    n = int(length // tooth_pitch)
    x = hl
    for i in range(n):
        pts.append((x - tooth_pitch / 2, base_t + tooth_amp))
        x -= tooth_pitch
        pts.append((x, base_t))
    pts.append((-hl, base_t))
    with BuildPart() as p:
        with BuildSketch(Plane.XZ):
            from build123d import Polygon
            Polygon(*pts, align=None)
        extrude(amount=width / 2, both=True)
        with Locations(*[(sx * dx, sy * dy, 0)
                         for sx in (-1, 1) for sy in (-1, 1)]):
            Cylinder(pilot_d / 2, base_t * 3, mode=Mode.SUBTRACT)
    return p.part


def write_board_dxf(path, length, chord, spars, cant_len, cant_w,
                    cap_len, end_wall=1.75):
    doc = ezdxf.new('R2010')
    doc.header['$INSUNITS'] = 4
    for n, c in (('CUT', 7), ('SLOT', 1), ('REF', 8)):
        doc.layers.add(n, color=c)
    msp = doc.modelspace()
    hl, hc = length / 2, chord / 2
    msp.add_lwpolyline([(-hl, -hc), (hl, -hc), (hl, hc), (-hl, hc)],
                       close=True, dxfattribs={'layer': 'CUT'})
    # surface-mounted spars: footprint marking lines only, no cuts
    for y in spars:
        msp.add_lwpolyline([(-hl, y - SPAR_T / 2), (hl, y - SPAR_T / 2),
                            (hl, y + SPAR_T / 2), (-hl, y + SPAR_T / 2)],
                           close=True, dxfattribs={'layer': 'REF'})
    cl, cw = cant_len / 2, cant_w / 2
    msp.add_lwpolyline([(-cl, -cw), (cl, -cw), (cl, cw), (-cl, cw)],
                       close=True, dxfattribs={'layer': 'REF'})
    cap_cover = cap_len - end_wall
    for x0, x1 in ((hl - cap_cover, hl), (-hl, -hl + cap_cover)):
        msp.add_lwpolyline([(x0, -hc), (x1, -hc), (x1, hc), (x0, hc)],
                           close=True, dxfattribs={'layer': 'REF'})
    doc.saveas(path)


def write_clamp_dxf(path, length=37.62, width=24.56, hole_d=2.1,
                    dx=17.35, dy=10.78):
    doc = ezdxf.new('R2010')
    doc.header['$INSUNITS'] = 4
    msp = doc.modelspace()
    hl, hw = length / 2, width / 2
    msp.add_lwpolyline([(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)],
                       close=True)
    for sx in (-1, 1):
        for sy in (-1, 1):
            msp.add_circle((sx * dx, sy * dy), hole_d / 2)
    doc.saveas(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--length', type=float, default=300.0,
                    help='board span (mm)')
    ap.add_argument('--chord', type=float, default=50.0,
                    help='board chord / width (mm)')
    ap.add_argument('--spars', type=float, nargs='+', default=[-3.5, 3.5],
                    help='spar slit centerlines, offsets from board '
                         'centerline (mm)')
    ap.add_argument('--cap-len', type=float, default=18.0)
    ap.add_argument('--suffix', default='',
                    help="filename suffix, e.g. _3spar")
    ap.add_argument('--out', type=Path,
                    default=Path(__file__).parent / 'fab')
    a = ap.parse_args()
    (a.out / 'laser').mkdir(parents=True, exist_ok=True)
    (a.out / 'print').mkdir(parents=True, exist_ok=True)

    cant = make_cantilever(a.spars)
    capA = make_end_cap(a.chord, a.spars)
    capB = make_end_cap(a.chord, a.spars, mirror=True)
    grip = make_grip_clamp()
    sfx = a.suffix
    for name, part in (('cantilever', cant), ('end_cap_A', capA),
                       ('end_cap_B', capB), ('clamp_grip', grip)):
        export_stl(part, str(a.out / 'print' / f'gen_{name}{sfx}.stl'))
        print(f'gen_{name}{sfx}.stl  vol {part.volume/1000:.2f} cm3')
    write_board_dxf(a.out / 'laser' / f'foam_board{sfx}.dxf',
                    a.length, a.chord, a.spars, 37.62, 24.56, a.cap_len)
    write_clamp_dxf(a.out / 'laser' / f'clamp_plate{sfx}.dxf')
    print(f'foam_board{sfx}.dxf + clamp_plate{sfx}.dxf written '
          f'(chord {a.chord}, spars {a.spars})')


if __name__ == '__main__':
    main()
