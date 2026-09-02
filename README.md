# Von Mises toggle — CAD toolchain

Parametric model, analysis, and one-command fabrication for the
bistable over-center toggle experiment. Everything derives from three
sketch numbers — `b` (frame leg P–S), `R` (arm P–T), `gamma1`
(state-1 interior angle) — plus `scale` (mm per sketch unit) and
`phi` (anchor direction). ~1850 lines across 10 source files.

## Quick start

    ./kit --preview            # look at the configuration, then stop
    ./kit                      # generate the fabrication kit -> fab/
    open config_designer.html  # slider version of --preview

(`kit` is the venv wrapper in the Mac working copy; in the repo run
`python make_kit.py` with build123d/trimesh/ezdxf/shapely installed.)

## The model (one triangle)

P = pivot, S = band anchor (fixed at distance b, direction phi),
T = arm tip on a circle of radius R. One degree of freedom: gamma.

    span    c(g) = sqrt(b^2 + R^2 - 2 b R cos g)     law of cosines
    energy  E(g) = 1/2 k (c - L0)^2  for c > L0, else 0 (band slack)
    torque  tau(g) = k (c - L0) * b R sin g / c      = dE/dg

Dead center at g = 180 (max stretch, zero moment arm). The REAL
resting states are hardware stops, not the energy symmetry:
- default geometry (b .8803, R 1.7178, phi 127.58): measured
  state 1 = 104.0, state 2 = 233.0 (plate corner)
- current experimental geometry (b .875, R 2.53, phi 150,
  gamma1 135): measured state 1 = 134.9; the mirror 225 is
  UNREACHABLE — plate corner stop lands at 287.4. Decision open:
  engineered second stop vs accept vs retune phi.

Design rules measured from the original frame: ONE standoff radius
w = 7.532 mm for every frame edge, corner arc, and fillet around P
and S; the clevis stop faces are calibrated to it. Stop-face
compensation: flush angle = gamma1 + 1.4 (the 0.25 mm running
clearance costs ~1.4 deg of over-travel).

## Fabrication pipeline

    config_designer.html   sliders -> the kit command
            |
        make_kit.py        orchestrator: resolves params, writes fab/
            +-- generate_frame.py    frame planform -> fab/laser/*.dxf
            |     make_kit READS THE DXF BACK (detect_datums) and
            |     re-measures b/phi from it: the file is the truth
            +-- clevis_generator.py  clevis + pin -> fab/print/*.stl
            |                        (+ STEP into fab/cad/)
            +-- toggle_assembly.py   posed states -> fab/cad/*.step

## File-by-file (current state)

### make_kit.py (264) — the conductor
Inputs are the sketch geometry (b, R, gamma1; gamma2 None = mirrored),
hardware knobs (pin-d, clearances, blade-t, rod-od), plumbing
(frame-mode, phi, standoff, preview). Fixed clevis body dims are code
constants (the measured joint). Flow: resolve -> frame -> detect
datums -> config preview (--preview exits) -> parts -> assemblies ->
BUILD_SHEET.md -> file listing. Uses socket_depth() so the sheet's
rod length always matches the actual printed socket.

### generate_frame.py (198) — the laser master
build_outline(): 12-vertex planform, clockwise; every arc radius w,
every edge offset w from a datum. Emits exact bulge arcs
(tan(sweep/4)) for the DXF and a discretized loop for shapely.
build_cutouts(): inward offset by margin (12.5), tab clipped solid,
8 mm diagonal rib subtracted, corners rounded by buffer(-f).buffer(f).
Defaults reproduce the original hand-drawn cutout.

### clevis_generator.py (247) — the joint
make_clevis(): body + rounded fork + counterbored Ø3.3 cross bore.
The stop plane (one inclined plane at standoff+clearance from the
bore, direction set by tab_stop_deg) is used twice: channel end wall
(cut) and tab wedge face (add) — the frame edge lands flush at
exactly one rotation. Rod mount is a FEMALE SOCKET by default
(--rod-mount socket): flush-sided boss, Ø rod_od+0.3 bore; the old
ribbed stem survives behind --rod-mount stem. socket_depth() is the
single source of truth for the depth cap (steep stop angles pull the
wall into the boss; e.g. 105.4 deg -> 10.0 mm, 136.4 deg -> 4.7 mm).
make_pin(): flush pin, chamfered tip, torus snap ridge into the far
counterbore. make_arm(): legacy printed arm.

### toggle_assembly.py (223) — DXF in, posed STEP out
detect_datums(): x-order invariant — 4 tab mount holes leftmost,
pivot at max x, the remaining hole is the anchor (survives any phi;
the old neighbor-clustering rule did not). bulge_points()/
polyline_points() discretize DXF arcs; make_frame_from_dxf() extrudes
the true planform, pivot at origin. build_assembly(): one composite
Location per part (translate bore to origin, rotX +90 to put the pin
axis on Z, rotZ so the stem points along phi - gamma); rod seated at
the socket floor. Exports a labeled Compound (separate components in
Fusion/SolidWorks).

### config_designer.html (252) — the eyes
Self-contained slider page (no server): JS mirror of the outline and
triangle math. Drive gamma1 directly or by beta (span-to-horizon,
ray/circle inversion); gamma2 mirrors unless unchecked; corner-angle
arcs at S and T; span, rod-cut readouts; emits the exact kit command.

### toggle_sketch.py (194) — the original 2D solver
--gammas or --horizons (beta -> gamma inversion), derived-dims table,
DXF/PNG sketches, --two-states mirrored layer.

### MATLAB (105 + 120 + 122)
toggle_energy_torque.m: E(g) over tau(g), asymmetric stops marked,
both barriers printed. toggle_paper_figure.m: linkage + energy
two-panel paper figure. toggle_model_sheet.m: beige planform
presentation view. Run: matlab -batch "cd('cad'); <name>"
(R2025b native at ~/Desktop/MATLAN; R2024b needs arch -x86_64).

### generate_truss.py (125) — legacy
First-session symmetric two-bar truss STL sweep. Superseded.

### Data
frame_cutout.dxf (original hand-drawn master, used by
--frame-mode original) · fab/ (regenerable kit incl.
fit_test_plate.stl: clevis pre-rotated fork-up + 4 pins, one-import
print file) · output/ (analysis exports).

## Verification status

- Generated frame overlays the original DXF exactly (outline/holes)
- Collision sweeps: default geometry stops measured 104.0/233.0;
  135-deg geometry state 1 measured 134.9, state 2 = 287.4
  (mirror unreachable), zero mid-travel interference in both
- Print staging: fit-test plate sliced clean, supportless
  (fork-up orientation), ~22 min / ~3.5 g
- NOT yet validated: print-fit clearances on the actual printer;
  k hang test pending (all energies still k = 1 units)

## Open items

- Print + fit-check the kit (pin bore, blade slot, rod socket)
- State-2 decision for the 135-deg geometry (engineered stop /
  accept 287 / retune phi)
- k measurement -> real joules on the analysis and band free length
- Optional: --verify sweep inside make_kit; kit.bat for Windows;
  lattice tip mass (picoGK)

## Installation

    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
    .venv/bin/python make_kit.py --preview

Paths in this repository are flat (no cad/ prefix). MATLAB scripts
run on R2024b+ with no toolboxes.

## Results

results/ holds validation artifacts: the generated-vs-original frame
overlay, the energy/torque analysis with measured stop angles, and
configuration previews.

## AI assistance disclosure

The implementation of this toolchain was developed with the
assistance of an AI coding tool (Claude, Anthropic), directed by the
author. All design specifications, physical measurements (frame
datum geometry, reference joint dimensions), experimental design,
and hardware validation are the author's.

## Citation

If you use this toolchain, please cite the repository:
https://github.com/pallmy/vonmises-toggle
