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
7. For editor testing, run the command from Script Editor after saving the project.
8. For Rhino command-line use, publish the project to build the plugin artifacts.
9. Install or load the generated plugin output before calling `GenerateInternalWire` in Rhino.

## Important publish behavior

Publishing builds the package and plugin files, but it does not automatically make
the Rhino command available in the current Rhino session.

After `Publish Project > Build Package`, use one of these paths:

- run the project directly from Script Editor while developing
- install the generated `.yak` package into Rhino
- or load the generated `.rhp` plugin into Rhino

If you skip the install/load step, Rhino will report `Unknown command:
GenerateInternalWire`.

## Load the published plugin in Rhino

After `Publish Project > Build Package`, load the generated Rhino plugin into Rhino:

1. In Rhino, run `PlugInManager`.
2. In the Plug-ins window, click `Install...`
3. Browse to the build output folder selected during publish.
4. Open the Rhino 8 build subfolder if one was created, for example `build/rh8/`.
5. Select the generated `.rhp` file. This file is usually named after the project.
6. Click `Open`.
7. Find the plugin in the list. If it is not loaded yet, right-click it and choose `Load plug-in`.
8. Close the Plug-ins window.
9. Run `GenerateInternalWire` in the Rhino command line.

If the command is still not available, open `PlugInManager`, right-click the
plugin, and choose `Commands` to verify that `GenerateInternalWire` is included
in the loaded plugin.

## Generated geometry

The command currently creates two outputs:

- a polyline centerline named `GenerateInternalWire_Centerline`
- a pipe Brep named `GenerateInternalWire_ConductivePath`

The pipe Brep is the conductive-path volume that follows the routed centerline.
The selected solid is not modified, and the command does not currently boolean
subtract the path from the host body or generate a separate outer casing body.

If you want a printable channel or cavity inside another body, that would be a
separate modeling step after the path is generated.

## Routing options

- `VoxelSize`: the spacing of the internal routing grid. Smaller values capture
   more detail and can route through tighter features, but they make the solve
   slower and increase memory use.
- `Clearance`: the minimum offset from the outside shell when marking valid
   routing cells. Larger values keep the path farther away from the part surface
   and leave more wall thickness, but can eliminate valid routes in thin regions.
- `WireRadius`: the radius of the generated pipe Brep. This controls the size of
   the visualized conductive path volume. It does not change the routing grid.

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
