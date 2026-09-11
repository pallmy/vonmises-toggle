Von Mises toggle rig, repo pointers for Yingchao

Code is in cad/ on branch parametric-geometry-cad. I'll walk you through the purpose and the build in person, this is just a map so you can poke around first.

Start here
- cad/README.md, the file by file walkthrough and current status
- fab/BUILD_SHEET.md, what's on the bench right now (numbers, cut stock, assembly steps)

Source files (cad/)
- config_designer.html, slider page to pick the geometry, no server
- make_kit.py, runs the whole pipeline and writes fab/
- generate_frame.py, the laser frame, writes fab/laser/frame_cutout.dxf
- clevis_generator.py, clevis, pin, rod socket, writes fab/print/*.stl
- toggle_assembly.py, poses the resting states, writes fab/cad/*.step
- wing_generator.py, the wing parts (foam board, cantilever, clamp, end caps)
- toggle_energy_torque.m and the other .m files, the MATLAB analysis

Outputs (cad/fab/)
- laser/ the DXFs
- print/ the STLs and the sliced wing plate
- cad/ the STEP assemblies (open in Fusion or SolidWorks)

Run it
- ./kit --preview to see the config, ./kit to write the kit
- clean checkout: python make_kit.py with build123d, trimesh, ezdxf, shapely installed
- MATLAB is R2025b

Open threads
- k hang test (top priority, everything downstream needs it)
- state 2 stop for the 135 geometry (mirror unreachable)
- print fit check on the real printer
