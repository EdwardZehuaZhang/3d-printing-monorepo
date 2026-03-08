# Rhino 8 Internal Wire Router

This folder contains a Rhino 8 script-plugin MVP that generates a wire path inside an arbitrary **closed** solid.

## What it does

- lets the user select one closed target geometry
- lets the user pick node points in order on that geometry with live node preview
- validates node size and node spacing during picking
- builds an interior voxel graph
- routes one continuous wire through the nodes
- adds separate conductive path, casing, and node solids back into Rhino

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

## Update an already installed plugin

If Rhino reports `Unable to load ... .rhp: ID already in use`, Rhino already has
this plugin registered and you should update the existing installed copy instead
of installing a second one.

1. In Rhino, run `PlugInManager`.
2. Find the plugin in the list and select it.
3. Right-click the plugin and choose `Open Containing Folder`.
4. Leave that folder open. This is the installed plugin location Rhino is using.
5. Close Rhino completely.
6. From the newly published build output, copy the updated `.rhp` file.
7. Paste it into the installed plugin folder and replace the old file.
8. Reopen Rhino.
9. Run `PlugInManager` and, if needed, right-click the plugin and choose `Load plug-in`.
10. Optionally right-click the plugin and choose `Commands` to confirm that `GenerateInternalWire` is available.

For repeat updates, publish to a temporary build folder and then replace the
installed `.rhp` file after Rhino is closed.

## Generated geometry

The command currently creates four outputs:

- a polyline centerline named `GenerateInternalWire_Centerline`
- conductive path solid(s) named `GenerateInternalWire_ConductivePath`
- casing solid(s) named `GenerateInternalWire_Casing`
- node solid(s) named `GenerateInternalWire_Nodes`

The selected object is used as the host body for routing and node placement, but
it is not modified automatically. The generated solids are separate objects that
can be used for downstream boolean operations or fabrication workflows.

## Routing options

- `VoxelSize`: the spacing of the internal routing grid. Smaller values capture
   more detail and can route through tighter features, but they make the solve
   slower and increase memory use.
- `Clearance`: the casing thickness around the conductive path. The routed
   centerline stays at least `WireRadius + Clearance` away from the outside shell
   so the casing fits inside the host object.
- `WireRadius`: the radius of the conductive path solid. The command enforces a
   minimum conductive path diameter of 0.5 mm.
- `NodeSize`: the node diameter. The command enforces a minimum node diameter of
   0.5 mm and rejects nodes that come within 0.5 mm of another node.

## Use the command

1. Start `GenerateInternalWire`.
2. Select one closed target object.
3. Adjust:
   - `VoxelSize`
   - `Clearance`
   - `WireRadius`
   - `NodeSize`
4. Pick node points in order.
5. Watch the live node preview while placing each node.
6. Press Enter after the last node.

The command adds:

- a polyline centerline
- conductive path solid(s)
- casing solid(s)
- node solid(s)

Node placement rules:

- node diameter must be at least 0.5 mm
- nodes must stay at least 0.5 mm apart from each other
- node solids must fit inside the selected object at the picked location

Routing rules:

- conductive path diameter must be at least 0.5 mm
- later routed segments are kept away from earlier segments to preserve at least
  0.5 mm separation where possible

## Tuning guidance

- Smaller `VoxelSize` = better detail, slower solve.
- Larger `Clearance` = thicker casing and more distance from the conductive path to the casing wall, but may fail in thin regions.
- Larger `WireRadius` = thicker conductive path and a larger minimum routing envelope.
- Larger `NodeSize` = larger conductive nodes, but fewer valid placements on tight geometry.

## Suggested next upgrades

- surface routing mode for open geometry
- branch / multi-wire support
- automatic node ordering
- export directly to fabrication workflow

## Suggested commit message

`feat(rhino): add Rhino 8 internal wire routing plugin MVP`
