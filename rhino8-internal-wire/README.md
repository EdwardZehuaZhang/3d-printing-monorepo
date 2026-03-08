# Rhino 8 Internal Wire Router

This folder contains a Rhino 8 script-plugin MVP that generates a wire path inside an arbitrary **closed** solid.

## What it does

- lets the user select one closed target geometry
- lets the user pick node points in order on that geometry
- builds an interior voxel graph
- routes one continuous wire through the nodes
- adds a centerline curve and a pipe solid back into Rhino

## Current scope

This version is intended for:

- closed meshes
- closed Breps / polysurfaces / extrusions that can be meshed cleanly
- SubD objects that can be converted to a closed mesh

It does **not** yet do surface-only routing on open geometry.

## Files

- [Commands/GenerateInternalWire.py](Commands/GenerateInternalWire.py) — Rhino command entry point
- [Libraries/wire_router/core.py](Libraries/wire_router/core.py) — pure Python A* grid router
- [Libraries/wire_router/rhino_router.py](Libraries/wire_router/rhino_router.py) — RhinoCommon integration

## Load into Rhino 8

1. Open Rhino 8.
2. Run `ScriptEditor`.
3. Create a new project (`File > Create Project`).
4. Add [Commands/GenerateInternalWire.py](Commands/GenerateInternalWire.py) under `Commands/`.
5. Add [Libraries/wire_router](Libraries/wire_router) under `Libraries/` as a Python library.
6. Save the project.
7. Run or publish the project.

## Use the command

1. Start `GenerateInternalWire`.
2. Select one closed target object.
3. Adjust:
   - `VoxelSize`
   - `Clearance`
   - `WireRadius`
4. Pick node points in order.
5. Press Enter after the last node.

The command adds:

- a polyline centerline
- a pipe Brep representing the wire

## Tuning guidance

- Smaller `VoxelSize` = better detail, slower solve.
- Larger `Clearance` = path stays farther from the shell, but may fail in thin regions.
- Larger `WireRadius` only changes the output pipe size; it does not change the routing graph.

## Suggested next upgrades

- surface routing mode for open geometry
- branch / multi-wire support
- automatic node ordering
- live preview while picking nodes
- export directly to fabrication workflow

## Suggested commit message

`feat(rhino): add Rhino 8 internal wire routing plugin MVP`
