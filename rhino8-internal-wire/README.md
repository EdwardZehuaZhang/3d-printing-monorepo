# Rhino 8 Internal Wire Router

This folder contains a Rhino 8 script-plugin MVP that generates a wire path inside an arbitrary **closed** solid.

## What it does

- lets the user select one closed target geometry
- lets the user pick start and end terminal connectors on the object first
- lets the user pick touch nodes on the object with live preview
- trims touch nodes flush to the object boundary
- builds an interior routing grid automatically
- routes one continuous conductive path from start to end through all touch nodes in an optimized order
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

## Standardized geometry

The command now uses fixed fabrication dimensions instead of prompting for
`VoxelSize`, `Clearance`, `WireRadius`, or `NodeSize`:

- conductive path diameter: `0.5 mm`
- conductive path casing thickness: `0.5 mm`
- minimum clearance from casing to host-object wall: `0.5 mm`
- minimum conductive path spacing from other conductive paths: `0.5 mm`
- touch node diameter: `3.0 mm`
- start and end terminal connector diameter: `3.0 mm`
- start and end terminal connector length: `6.0 mm`

The routing grid resolution is computed automatically in the background.

## Electrical estimate

The command prints an estimated conductive resistance range using ProtoPasta
Conductive PLA, scaled from the published `2.0-3.5 kohm per 10 cm` value for
`1.75 mm` filament.

It reports:

- estimated total start-to-end resistance for the generated path
- estimated resistance from the start terminal to each touch node along the final route

## Use the command

1. Start `GenerateInternalWire`.
2. Select one closed target object.
3. Pick the `Start` terminal on the object.
4. Choose whether that terminal is `Flush` or `Protrude`.
5. Pick the `End` terminal on the object.
6. Choose whether that terminal is `Flush` or `Protrude`.
7. Pick touch nodes on the object surface.
8. Watch the live node preview while placing each touch node.
9. Press Enter after the last touch node.

The command adds:

- a polyline centerline
- conductive path solid(s)
- casing solid(s)
- node solid(s)

Terminal rules:

- start and end connectors are cylinders of diameter `3.0 mm` and length `6.0 mm`
- each terminal can be placed as either `Flush` or `Protrude`
- a terminal must fit on the chosen surface area or the command rejects that placement

Touch-node rules:

- touch nodes are spherical conductive solids trimmed flush to the object surface
- nodes must stay at least 0.5 mm apart from each other
- nodes are optimized into the final routing order automatically and do not use the click order as the final electrical sequence

Routing rules:

- conductive path diameter must be at least 0.5 mm
- conductive path casing thickness is fixed at 0.5 mm
- conductive paths stay at least 0.5 mm away from other conductive paths where the geometry allows
- path casing stays at least 0.5 mm away from the host-object wall
- if a path between two successive optimized nodes cannot fit, the command reports that the pathway between those nodes is not big enough
- routing uses orthogonal grid motion to encourage a zig-zag style path instead of the shortest straight line

## Tuning guidance

- Larger host geometries allow longer zig-zag conductive paths and more separation between touch nodes.
- Tight corners, thin walls, and narrow necks are the main reasons routing fails.
- If the command reports that a pathway is not big enough, move the touch nodes farther apart or use a larger host body.

## Suggested next upgrades

- surface routing mode for open geometry
- branch / multi-wire support
- export directly to fabrication workflow

## Suggested commit message

`feat(rhino): add Rhino 8 internal wire routing plugin MVP`
