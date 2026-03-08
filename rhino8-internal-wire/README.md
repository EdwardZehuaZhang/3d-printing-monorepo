# Rhino 8 Internal Wire Router

This folder contains a Rhino 8 script-plugin MVP that generates a wire path inside an arbitrary **closed** solid.

## What it does

- lets the user select one closed target geometry
- lets the user pick start and end terminal connectors on the object first
- lets the user pick touch nodes on the object with live preview
- trims touch nodes flush to the object boundary
- builds an interior routing grid automatically
- ranks possible node orders by how close their touch-to-touch resistance steps are to the user-selected target value
- tries target-close orders until it finds the nearest routable order that fits the geometry constraints
- actively lengthens touch-to-touch routed legs through available interior volume with chained detours when the direct route is too short for the requested resistance target
- routes one continuous conductive path from start to end through all touch nodes in that selected order
- adds separate conductive path and conductive pathway node solids back into Rhino

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

## Quick Start In Rhino 8

Prerequisites:

- Rhino 8 installed
- this repository available on your computer

Try the plugin locally on your machine:

1. Open Rhino 8.
2. Run `ScriptEditor`.
3. Choose `File > Create Project`.
4. Fill in the required project metadata such as author, then save the project.
5. In the Script Editor project tree, add [Commands/GenerateInternalWire.py](Commands/GenerateInternalWire.py) into the `Commands` folder.
6. In the Script Editor project tree, add the folder [Libraries/wire_router](Libraries/wire_router) as a Python library under `Libraries`.
7. Save the project.
8. Use Script Editor run/debug for quick iteration while editing the source.
9. When you want the command available from the Rhino command line, publish the project with `Publish Project > Build Package`.
10. After publish completes, install the generated `.rhp` with `PlugInManager > Install...`.
11. If Rhino asks, load the plugin.
12. Run `GenerateInternalWire` in Rhino.

First-use workflow inside Rhino:

1. Start `GenerateInternalWire`.
2. Select one closed solid, closed mesh, extrusion, or SubD.
3. Enter the conductive pathway node sphere diameter in millimeters. Press Enter to accept the default `10.0 mm` finger-sized conductive node.
4. Enter the conductive path diameter in millimeters. Press Enter to accept the default `0.5 mm` single printable line.
5. Enter the desired touch-to-touch resistance between successive nodes in kohm. Press Enter to accept the recommended default based on the selected conductive path diameter. With the default `0.5 mm` single printable line, the suggested target is `200.0 kohm` for stronger noise separation.
6. Pick the `Start` terminal.
7. Set that terminal to `Flush` or `Protrude`.
8. Pick the `End` terminal.
9. Set that terminal to `Flush` or `Protrude`.
10. Pick conductive pathway nodes on the surface.
11. Press Enter when all nodes are placed.
12. Inspect the generated conductive path and conductive nodes.
13. Check the Rhino command history for the selected order, the estimated touch-to-touch resistance steps before routing, the actual touch-to-touch resistance steps after routing, and the total start-to-end resistance.

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

The command currently creates three outputs:

- a polyline centerline named `GenerateInternalWire_Centerline`
- conductive path solid(s) named `GenerateInternalWire_ConductivePath`
- conductive pathway node solid(s) named `GenerateInternalWire_ConductivePathwayNodes`

The selected object is used as the host body for routing and node placement, but
it is not modified automatically. The generated solids are separate objects that
can be used for downstream boolean operations or fabrication workflows.

## Standardized geometry

The command now uses fixed fabrication rules instead of prompting for
`VoxelSize`, `Clearance`, or legacy `WireRadius` settings:

- conductive path diameter: user-defined per command run, but never smaller than `0.5 mm` and never larger than the selected conductive pathway node diameter
- retained virtual clearance margin around conductive paths: `0.5 mm`
- minimum clearance from that retained margin to the host-object wall: `0.5 mm`
- minimum conductive path spacing from other conductive paths: `0.5 mm`
- conductive pathway node sphere diameter: user-defined per command run, but defaults to `10.0 mm` and is never smaller than `0.5 mm`
- start and end terminal connector diameter: `3.0 mm`
- start and end terminal connector length: `6.0 mm`
- default desired touch-to-touch resistance target: recommended from conductive path diameter, using `50.0 kohm` at `1.0 mm` as the reference value and `200.0 kohm` for the default `0.5 mm` single printable line

The routing grid resolution is computed automatically in the background.

## Electrical estimate

The command prints an estimated conductive resistance range using ProtoPasta
Conductive PLA, scaled from the published `2.0-3.5 kohm per 10 cm` value for
`1.75 mm` filament and the user-selected conductive path diameter.

It reports:

- estimated total start-to-end resistance for the generated path
- estimated touch-to-touch resistance steps before routing for the chosen node order
- actual touch-to-touch resistance steps after routing for the final routed path
- whether the closest target-ranked node order was routable or whether a nearby alternative had to be used
- a suggested Arduino send-pin resistor range of `470 kohm` to `2.2 Mohm`, with `1 Mohm` as the starting point
- a detailed routing failure explanation when no node order can fit the geometry constraints

## Use the command

1. Start `GenerateInternalWire`.
2. Select one closed target object.
3. Enter the conductive pathway node sphere diameter in millimeters.
4. Enter the conductive path diameter in millimeters.
5. Enter the desired touch-to-touch resistance target in kohm.
6. Pick the `Start` terminal on the object.
7. Choose whether that terminal is `Flush` or `Protrude`.
8. Pick the `End` terminal on the object.
9. Choose whether that terminal is `Flush` or `Protrude`.
10. Pick touch nodes on the object surface.
11. Watch the live node preview while placing each touch node.
12. Press Enter after the last touch node.

The command adds:

- a polyline centerline
- conductive path solid(s)
- node solid(s)

Terminal rules:

- start and end connectors are cylinders of diameter `3.0 mm` and length `6.0 mm`
- each terminal can be placed as either `Flush` or `Protrude`
- a terminal must fit on the chosen surface area or the command rejects that placement

Touch-node rules:

- conductive pathway nodes are conductive solids, not casing
- the conductive node sphere diameter is chosen by the user for the entire command run
- if the sphere diameter is smaller than `0.5 mm`, the command rejects it as too small
- if the sphere diameter is too large for the selected location, overlaps another node or terminal, or leaves no protected routing corridor, the command rejects it as too big
- the picked surface point becomes the center of the conductive node sphere, and any part of that sphere outside the host geometry is trimmed away
- no non-conductive casing is generated around conductive pathway nodes
- corners are valid placement targets when the selected sphere diameter still fits the routing clearance
- nodes are optimized into the final routing order automatically to improve distinguishability and do not use the click order as the final electrical sequence

Routing rules:

- conductive path diameter is chosen by the user for the entire command run
- conductive path diameter must be at least 0.5 mm
- conductive path diameter cannot exceed the selected conductive pathway node diameter
- if a chosen conductive path diameter is geometrically too large and would force overlaps with nodes or pathways, routing fails and the command reports that the pathway is not big enough
- no casing geometry is generated, but a virtual 0.5 mm clearance margin is still enforced in the routing logic
- conductive paths stay at least 0.5 mm away from other conductive paths where the geometry allows
- conductive paths reserve protected space around every terminal and every conductive pathway node so the route cannot cut through another anchor
- the retained virtual clearance margin stays at least 0.5 mm away from the host-object wall
- if a path between two successive optimized nodes cannot fit, the command reports that the pathway between those nodes is not big enough
- the user chooses the desired touch-to-touch resistance between successive conductive pathway nodes in kohm for each run
- only touch-node-to-touch-node resistance steps are used to rank node orders; the start terminal leg and end terminal leg are excluded from that scoring rule
- the router searches for the node order whose estimated touch-to-touch steps are closest to the requested value and then tries nearby alternatives until it finds a routable order
- once an order is chosen, the router tries to spend free interior space on the touch-to-touch legs with chained detours so the actual routed resistance can move closer to the requested value instead of always collapsing to the shortest corridor
- those chained detours are biased toward roomy interior regions and away from narrow connectors, so extra snaking happens in larger chambers before it spends scarce corridor space
- each target-seeking touch leg also avoids folding back into itself at the configured path-spacing radius, which prevents self-overlap while it is trying to gain resistance length
- if the routed result does not exactly match the requested touch-to-touch resistance, the command still succeeds and reports the actual touch-to-touch values and total start-to-end resistance
- if no candidate order can be routed, the command explains that the blocking issue is a local corridor problem caused by path diameter, clearance, spacing, and protected node or terminal zones, not necessarily the overall object size
- routing uses orthogonal grid motion to encourage a zig-zag style path instead of the shortest straight line

## Tuning guidance

- Larger host geometries allow longer zig-zag conductive paths and more separation between touch nodes.
- Tight corners, thin walls, and narrow necks are the main reasons routing fails.
- Lower the desired touch-to-touch resistance when you mainly want a route candidate quickly.
- Raise the desired touch-to-touch resistance when physical testing shows nodes are being confused by noise or inconsistent touches.
- If the command reports that a pathway is not big enough, move the touch nodes farther apart or use a larger host body.

## Suggested next upgrades

- surface routing mode for open geometry
- branch / multi-wire support
- export directly to fabrication workflow

## Suggested commit message

`feat(rhino): add Rhino 8 internal wire routing plugin MVP`
